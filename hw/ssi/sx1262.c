/*
 * SX1262 LoRa transceiver as a QEMU SSI peripheral.
 *
 * It models none of the radio itself. The chip is MeshBench's virtual-sx1262,
 * an MIT library loaded at runtime, and the physics, the channel and the
 * demodulator all live in the simulator this device connects to. Duplicating
 * any of that here would give us two models to keep in agreement.
 *
 * MeshCore's radio_init() drives this chip over GP-SPI. Without something
 * answering, RadioLib waits on a chip that never replies and the ESP32 task
 * watchdog resets the board, which is where the emulated backend stopped.
 *
 * Attach it the way the board already attaches flash:
 *
 *     -device sx1262,bus=spi2,cs=0,bridge=127.0.0.1:38217
 *
 * The library comes from MESHBENCH_RADIO_LIB and the receiver's noise seed from
 * MESHBENCH_NOISE_SEED, because both are decided by whoever started this
 * emulator rather than by the board being emulated.
 *
 * There used to be a third process here. The chip lived in `radioserver`, and
 * this device forwarded every clocked byte to it over a socket and read the
 * answering byte back. That cost more than the round trips. It framed SPI a
 * byte at a time, which is how the model came to answer GetRssiInst correctly
 * on the native path and with zero on this one for months; it left DIO1 with no
 * way to be pushed, so this device sampled the pin on a millisecond timer, long
 * enough that a packet delivered and acknowledged inside one gap raised the pin
 * never; and it put three clocks in three processes in front of anyone asking
 * what happened when. So the chip is in here now, and the one socket that
 * remains carries what genuinely is somewhere else: the simulated air.
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/ssi/ssi.h"
#include "hw/qdev-properties.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "qemu/main-loop.h"
#include "io/channel-socket.h"
#include "qemu/sockets.h"

#include <gmodule.h>

#define TYPE_SX1262 "sx1262"
OBJECT_DECLARE_SIMPLE_TYPE(SX1262State, SX1262)

/*
 * The chip library's ABI, declared here rather than included.
 *
 * The header lives in MeshBench/virtual-sx1262 and this is a fork of QEMU: a
 * submodule would make every rebase of this branch carry one, and a chip fix
 * would then mean rebuilding the emulator instead of replacing one file.
 * Renode's peripheral declares the same surface by hand for the same reason,
 * which is why that repository treats the ABI as a contract: append only, never
 * reorder, and bump the minor when you add.
 *
 * ABI 1.3 is the floor. It is the version that added the byte-at-a-time path,
 * and an SSI peripheral has nothing else it can use: transfer_raw is called
 * once per clocked byte and must answer that byte before the next is clocked.
 */
#define SX1262_ABI_MAJOR 1
#define SX1262_ABI_MINOR 3

typedef struct vsx_chip vsx_chip;
typedef void (*vsx_dio1_fn)(void *user, int asserted);

typedef struct {
    uint32_t freq_hz;
    uint32_t bandwidth_hz;
    uint16_t preamble_syms;
    uint16_t irq_mask;
    uint16_t irq_flags;
    uint8_t spreading_factor;
    uint8_t coding_rate;
    uint8_t mode;
    int8_t tx_power_dbm;
    uint8_t rx_gain_reg;
    uint8_t fem_at_tx;
    uint16_t dio1_mask;
} vsx_state;

typedef struct {
    uint32_t irq_reads;
    uint32_t busy_reads;
    uint32_t busy_ms;
    uint32_t spurious_raises;
    uint32_t preamble_raises;
    uint32_t frames_dropped;
} vsx_counters;

/*
 * Both structs are append-only, and a library newer than this file writes as
 * many bytes as *it* knows about into the pointer it is given. Reading into a
 * bare vsx_state would then be a buffer overflow that only appears after
 * somebody upgrades the chip, which is the worst moment to find it. So every
 * read goes into one of these, and the fields past what is declared above are
 * ignored on purpose.
 */
typedef union {
    vsx_state s;
    uint8_t pad[128];
} SX1262StateBuf;

typedef union {
    vsx_counters c;
    uint8_t pad[128];
} SX1262CountersBuf;

typedef struct {
    GModule *module;

    void (*abi_version)(int *major, int *minor);
    vsx_chip *(*create)(void);
    void (*destroy)(vsx_chip *chip);
    void (*set_dio1_callback)(vsx_chip *chip, vsx_dio1_fn fn, void *user);
    void (*spi_begin)(vsx_chip *chip);
    uint8_t (*spi_byte)(vsx_chip *chip, uint8_t out);
    void (*spi_end)(vsx_chip *chip);
    int (*busy)(const vsx_chip *chip);
    void (*tick)(vsx_chip *chip, uint64_t now_ms);
    void (*set_channel_busy)(vsx_chip *chip, int busy);
    void (*deliver_frame)(vsx_chip *chip, const uint8_t *frame, size_t len);
    void (*transmit_finished)(vsx_chip *chip);
    size_t (*take_tx)(vsx_chip *chip, uint8_t *dst, size_t cap);
    void (*set_fem_enabled)(vsx_chip *chip, int enabled);
    void (*get_state)(const vsx_chip *chip, vsx_state *out);
    void (*get_counters)(const vsx_chip *chip, vsx_counters *out);
    void (*set_noise_seed)(vsx_chip *chip, uint64_t seed);
} SX1262Lib;

/*
 * The engine's wire protocol: [kind:1][length:2 big-endian][payload].
 *
 * Shared with the native firmware bridge and with the simulator's Go half, and
 * frozen by both. Only the kinds an emulated node can act on are named; the
 * rest are skipped rather than fatal, because the framing carries a length, so
 * an unrecognised kind costs nothing and cannot desynchronise the stream.
 */
enum {
    ENGINE_FRAME        = 0x01, /* a radio frame, either direction */
    ENGINE_TICK         = 0x02, /* engine to node: advance to this instant */
    ENGINE_ACK          = 0x03, /* node to engine: that tick is processed */
    ENGINE_TX_DONE      = 0x04, /* engine to node: your waveform has ended */
    ENGINE_CHANNEL_BUSY = 0x08, /* engine to node: somebody else is on the air */
    ENGINE_RADIO_STATS  = 0x09, /* node to engine: what the chip has been asked */
};

/* The stats record, whose layout the engine reads on length. */
#define ENGINE_STATS_LEN 39

struct SX1262State {
    SSIPeripheral parent_obj;

    char *bridge;               /* host:port of the RF engine */

    SX1262Lib lib;
    vsx_chip *chip;

    QIOChannelSocket *sock;
    uint32_t sim_ms;            /* the last instant the engine ticked us to */

    bool cs_active;
    bool fem_level;

    /* NSS in, BUSY and DIO1 out. All ordinary GPIOs on these boards rather
     * than the SPI controller's own lines, so the board wires them pin to
     * pin. */
    qemu_irq busy_out;
    qemu_irq dio1_out;

    /* Reported once. A radio that has lost its engine should say so on the
     * console rather than repeat itself on every tick. */
    bool warned;
};

/* ---- the chip library ---- */

static bool sx1262_bind(SX1262Lib *lib, const char *path, Error **errp)
{
    int major = 0, minor = 0;
    size_t i;
    const struct {
        const char *name;
        gpointer *slot;
    } syms[] = {
        { "vsx_abi_version",       (gpointer *)&lib->abi_version },
        { "vsx_create",            (gpointer *)&lib->create },
        { "vsx_destroy",           (gpointer *)&lib->destroy },
        { "vsx_set_dio1_callback", (gpointer *)&lib->set_dio1_callback },
        { "vsx_spi_begin",         (gpointer *)&lib->spi_begin },
        { "vsx_spi_byte",          (gpointer *)&lib->spi_byte },
        { "vsx_spi_end",           (gpointer *)&lib->spi_end },
        { "vsx_busy",              (gpointer *)&lib->busy },
        { "vsx_tick",              (gpointer *)&lib->tick },
        { "vsx_set_channel_busy",  (gpointer *)&lib->set_channel_busy },
        { "vsx_deliver_frame",     (gpointer *)&lib->deliver_frame },
        { "vsx_transmit_finished", (gpointer *)&lib->transmit_finished },
        { "vsx_take_tx",           (gpointer *)&lib->take_tx },
        { "vsx_set_fem_enabled",   (gpointer *)&lib->set_fem_enabled },
        { "vsx_get_state",         (gpointer *)&lib->get_state },
        { "vsx_get_counters",      (gpointer *)&lib->get_counters },
        { "vsx_set_noise_seed",    (gpointer *)&lib->set_noise_seed },
    };

    /* Not lazy: every symbol is resolved below anyway, and a missing one should
     * be named here rather than crash the guest at the first SPI byte. */
    lib->module = g_module_open(path, 0);
    if (!lib->module) {
        error_setg(errp, "sx1262: cannot load the chip model at %s: %s",
                   path, g_module_error());
        return false;
    }
    for (i = 0; i < ARRAY_SIZE(syms); i++) {
        if (!g_module_symbol(lib->module, syms[i].name, syms[i].slot)) {
            error_setg(errp, "sx1262: %s has no %s: is it virtual-sx1262?",
                       path, syms[i].name);
            g_module_close(lib->module);
            lib->module = NULL;
            return false;
        }
    }

    lib->abi_version(&major, &minor);
    /* A major that does not match means every host has to be rebuilt, and the
     * library is saying so; a minor below the floor means an entry point this
     * device calls is simply not there. Refusing is the whole point of asking:
     * an unchecked mismatch is a chip that answers plausible nonsense. */
    if (major != SX1262_ABI_MAJOR || minor < SX1262_ABI_MINOR) {
        error_setg(errp, "sx1262: %s is ABI %d.%d, and this device needs %d.%d "
                   "or a later minor", path, major, minor,
                   SX1262_ABI_MAJOR, SX1262_ABI_MINOR);
        g_module_close(lib->module);
        lib->module = NULL;
        return false;
    }
    return true;
}

/* ---- the engine ---- */

/* Whole messages, on a blocking channel, so a short read cannot leave the
 * stream half-consumed: the framing has no way to resynchronise, and a device
 * that answered from the middle of a frame would look like a chip returning
 * plausible nonsense. */
static bool sx1262_read_all(SX1262State *s, void *buf, size_t n)
{
    return s->sock &&
        qio_channel_read_all(QIO_CHANNEL(s->sock), buf, n, NULL) == 0;
}

static bool sx1262_send_msg(SX1262State *s, uint8_t kind,
                            const uint8_t *payload, size_t n)
{
    uint8_t hdr[3] = { kind, (uint8_t)(n >> 8), (uint8_t)n };
    QIOChannel *ioc;

    if (!s->sock) {
        return false;
    }
    ioc = QIO_CHANNEL(s->sock);
    if (qio_channel_write_all(ioc, (const char *)hdr, sizeof(hdr), NULL) < 0) {
        return false;
    }
    return n == 0 ||
        qio_channel_write_all(ioc, (const char *)payload, n, NULL) == 0;
}

static void sx1262_put32(uint8_t *p, uint32_t v)
{
    p[0] = v >> 24;
    p[1] = v >> 16;
    p[2] = v >> 8;
    p[3] = v;
}

static void sx1262_put16(uint8_t *p, uint16_t v)
{
    p[0] = v >> 8;
    p[1] = v;
}

/*
 * Everything this radio has been configured to be, and what it has counted.
 *
 * It exists because a board profile is a datasheet claim about hardware and not
 * a claim about the firmware running on it: until the chip reported its own
 * state there was no way to tell a node configured correctly from one that was
 * not. The native bridge writes the same payload in the same order, because an
 * emulated node and a native one reporting different shapes would make every
 * comparison between them a comparison of our own code.
 */
static void sx1262_send_stats(SX1262State *s)
{
    SX1262StateBuf st = { };
    SX1262CountersBuf ct = { };
    uint8_t sb[ENGINE_STATS_LEN];

    s->lib.get_state(s->chip, &st.s);
    s->lib.get_counters(s->chip, &ct.c);

    sx1262_put32(&sb[0], ct.c.irq_reads);
    sx1262_put32(&sb[4], ct.c.busy_reads);
    sx1262_put32(&sb[8], ct.c.busy_ms);
    sx1262_put32(&sb[12], ct.c.spurious_raises);

    sb[16] = st.s.rx_gain_reg;
    sb[17] = (uint8_t)st.s.tx_power_dbm;
    /* The line as it stands now, which this device knows because the board
     * wires it here: the module sits beside the chip, not inside it, so the
     * chip has no view of it and does not report one. */
    sb[18] = s->fem_level ? 1 : 0;
    sb[19] = st.s.mode;
    sb[20] = st.s.spreading_factor;
    sb[21] = st.s.coding_rate;
    sx1262_put32(&sb[22], st.s.freq_hz);
    sx1262_put32(&sb[26], st.s.bandwidth_hz);
    sx1262_put16(&sb[30], st.s.preamble_syms);
    sx1262_put16(&sb[32], st.s.irq_mask);
    sx1262_put16(&sb[34], st.s.irq_flags);
    /* Three states, because "has not transmitted" is not "transmitted with the
     * module out": 0 no transmission yet, 1 module out, 2 module in. */
    sb[36] = st.s.fem_at_tx;
    /* The DIO1 routing mask, which is not the enable mask above. Reported
     * separately because confusing the two is a fault that has already happened
     * here: HeaderValid raised DIO1 part-way through a carrier, the pin was
     * still high when RxDone arrived, and a driver that attaches on the rising
     * edge never learned the packet existed. */
    sx1262_put16(&sb[37], st.s.dio1_mask);

    sx1262_send_msg(s, ENGINE_RADIO_STATS, sb, sizeof(sb));
}

static void sx1262_engine_down(SX1262State *s, const char *why)
{
    if (!s->sock) {
        return;
    }
    qemu_set_fd_handler(s->sock->fd, NULL, NULL, NULL);
    qio_channel_close(QIO_CHANNEL(s->sock), NULL);
    object_unref(OBJECT(s->sock));
    s->sock = NULL;
    if (!s->warned) {
        warn_report("sx1262: %s; this node is now deaf and mute", why);
        s->warned = true;
    }
}

/* Anything the firmware handed its radio goes out to the engine now.
 *
 * A transmission reaches the channel immediately and is *not* immediately
 * complete: the chip stays in transmit until the engine sends TX_DONE, exactly
 * as a native node does, because that is what stops a node talking over
 * itself. */
static void sx1262_drain_tx(SX1262State *s)
{
    uint8_t frame[512];
    size_t n = s->lib.take_tx(s->chip, frame, sizeof(frame));

    if (n == 0) {
        return;
    }
    if (n > sizeof(frame)) {
        /* Truncated, and said out loud. A frame this long is not something
         * MeshCore sends, so it means the chip and this device disagree about
         * the buffer rather than that a node had a lot to say. */
        warn_report("sx1262: the chip offered a %zu byte frame", n);
        n = sizeof(frame);
    }
    sx1262_send_msg(s, ENGINE_FRAME, frame, n);
}

static void sx1262_tick(SX1262State *s, uint32_t to_ms)
{
    uint8_t ack[4];

    /* A millisecond at a time, as a native node is stepped. Stepping rather
     * than jumping is what keeps the chip's own timeouts behaving: a preamble
     * flag that should clear after 66 ms does not, if time arrives in 500 ms
     * lumps. */
    while (s->sim_ms < to_ms) {
        s->sim_ms++;
        s->lib.tick(s->chip, s->sim_ms);
        sx1262_drain_tx(s);
    }
    s->lib.tick(s->chip, s->sim_ms);
    sx1262_drain_tx(s);

    sx1262_send_stats(s);
    sx1262_put32(ack, to_ms);
    if (!sx1262_send_msg(s, ENGINE_ACK, ack, sizeof(ack))) {
        sx1262_engine_down(s, "the engine stopped listening");
    }
}

/*
 * One message from the engine.
 *
 * Registered with the main loop rather than serviced on a thread of our own, so
 * this runs under the big QEMU lock and so does every SPI byte the guest clocks
 * out. That is why the chip needs no lock here: the two things that touch it
 * cannot run at once, and the DIO1 callback below can drive an interrupt line
 * directly instead of bouncing through a bottom half.
 */
static void sx1262_engine_readable(void *opaque)
{
    SX1262State *s = SX1262(opaque);
    uint8_t hdr[3];
    g_autofree uint8_t *payload = NULL;
    size_t n;

    if (!sx1262_read_all(s, hdr, sizeof(hdr))) {
        sx1262_engine_down(s, "the engine went away");
        return;
    }
    n = ((size_t)hdr[1] << 8) | hdr[2];
    if (n > 0) {
        payload = g_malloc(n);
        if (!sx1262_read_all(s, payload, n)) {
            sx1262_engine_down(s, "the engine went away mid-message");
            return;
        }
    }

    switch (hdr[0]) {
    case ENGINE_FRAME:
        /* A packet the channel delivered. Only frames that passed CRC arrive
         * here, exactly as on hardware: everything else was recorded and
         * withheld. */
        if (n > 0) {
            s->lib.deliver_frame(s->chip, payload, n);
        }
        break;

    case ENGINE_TX_DONE:
        s->lib.transmit_finished(s->chip);
        break;

    case ENGINE_CHANNEL_BUSY:
        if (n >= 1) {
            s->lib.set_channel_busy(s->chip, payload[0] != 0);
        }
        break;

    case ENGINE_TICK:
        if (n == 4) {
            sx1262_tick(s, ((uint32_t)payload[0] << 24) |
                           ((uint32_t)payload[1] << 16) |
                           ((uint32_t)payload[2] << 8) | payload[3]);
        }
        break;

    default:
        /* Skipped, not fatal. Console traffic reaches an emulated node over the
         * emulator's own serial port, so the engine's console messages arrive
         * here and are meant to be ignored. Treating an unknown kind as fatal
         * once killed the radio the moment anybody typed at the fleet, and the
         * node then reported "radio init failed: -2", which points at wiring. */
        break;
    }
}

/* ---- the pins ---- */

/*
 * DIO1, pushed by the chip.
 *
 * MeshCore reads a received packet only from the interrupt this line raises:
 * RadioLibWrapper::recvRaw is gated on a flag set solely by
 * setPacketReceivedAction. A chip that receives perfectly and has no wire to
 * say so is a node that never forwards anything.
 */
static void sx1262_dio1(void *user, int asserted)
{
    SX1262State *s = SX1262(user);

    qemu_set_irq(s->dio1_out, asserted);
}

/* The driver toggling the chip select by hand. This is what frames a command:
 * the controller clocks bytes one transfer at a time, and an SX1262 command
 * carries no length, so without these edges the chip has no way to tell where
 * one command ends and the next begins. */
static void sx1262_nss(void *opaque, int n, int level)
{
    SX1262State *s = SX1262(opaque);

    if (s->cs_active == !level) {
        return;
    }
    s->cs_active = !level;                      /* active low */
    if (s->cs_active) {
        s->lib.spi_begin(s->chip);
    } else {
        s->lib.spi_end(s->chip);
    }
}

/* The board's front-end module enable line: an external amplifier and antenna
 * switch that the firmware drives as an ordinary GPIO. The radio cannot see it
 * over SPI - the module sits beside the chip, not inside it, and the chip has
 * no way to know whether its output reaches an antenna. But whether the line is
 * asserted decides how much power leaves the board, so the model needs it. */
static void sx1262_fem(void *opaque, int n, int level)
{
    SX1262State *s = SX1262(opaque);

    if (s->fem_level == !!level) {
        return;
    }
    s->fem_level = !!level;
    s->lib.set_fem_enabled(s->chip, s->fem_level);
}

static uint32_t sx1262_transfer(SSIPeripheral *dev, uint32_t val)
{
    SX1262State *s = SX1262(dev);

    /* Bytes on this bus are not necessarily ours. A board can hang a display
     * and a card reader off the same controller as the radio, told apart only
     * by which chip select is low, and a peripheral that answered while
     * another was addressed would corrupt both. */
    if (!s->cs_active) {
        return 0x00;
    }
    return s->lib.spi_byte(s->chip, (uint8_t)val);
}

/* ---- lifecycle ---- */

static bool sx1262_join_engine(SX1262State *s, Error **errp)
{
    SocketAddress *addr = socket_parse(s->bridge, errp);

    if (!addr) {
        error_prepend(errp, "sx1262: cannot make sense of bridge=%s: ",
                      s->bridge);
        return false;
    }
    s->sock = qio_channel_socket_new();
    if (qio_channel_socket_connect_sync(s->sock, addr, errp) < 0) {
        error_prepend(errp, "sx1262: cannot reach the engine at %s: ",
                      s->bridge);
        qapi_free_SocketAddress(addr);
        object_unref(OBJECT(s->sock));
        s->sock = NULL;
        return false;
    }
    qapi_free_SocketAddress(addr);
    /* Frames are small and latency is the whole game: a tick that waits on
     * Nagle is a node that answers late for no reason. */
    socket_set_nodelay(s->sock->fd);
    /* Blocking on purpose. A message is read whole once the socket has said one
     * arrived, and the guest is stopped meanwhile, which is what keeps the
     * emulated node in step with the engine rather than racing it. */
    qio_channel_set_blocking(QIO_CHANNEL(s->sock), true, NULL);
    /* On the main loop rather than on a thread of our own: see the note above
     * sx1262_engine_readable. */
    qemu_set_fd_handler(s->sock->fd, sx1262_engine_readable, NULL, s);
    return true;
}

static void sx1262_realize(SSIPeripheral *dev, Error **errp)
{
    SX1262State *s = SX1262(dev);
    const char *lib_path = getenv("MESHBENCH_RADIO_LIB");
    const char *seed = getenv("MESHBENCH_NOISE_SEED");

    if (!lib_path || !*lib_path) {
        error_setg(errp, "sx1262: set MESHBENCH_RADIO_LIB to the "
                   "virtual-sx1262 shared library this node's chip comes from");
        return;
    }
    if (!sx1262_bind(&s->lib, lib_path, errp)) {
        return;
    }
    s->chip = s->lib.create();
    if (!s->chip) {
        error_setg(errp, "sx1262: the chip model would not start");
        return;
    }
    /* The seed for this node's receiver noise, which is where its firmware gets
     * its entropy: RadioLib reads the chip's instantaneous RSSI for random bits
     * and MeshCore derives its identity from them. Every node needs its own
     * stream or every node comes up with the same keypair, which is what
     * happened: two different boards reported the same public key. */
    if (seed && *seed) {
        s->lib.set_noise_seed(s->chip, g_ascii_strtoull(seed, NULL, 10));
    }
    s->lib.set_dio1_callback(s->chip, sx1262_dio1, s);

    if (s->bridge && *s->bridge) {
        if (!sx1262_join_engine(s, errp)) {
            s->lib.destroy(s->chip);
            s->chip = NULL;
            return;
        }
    } else {
        /* Worth saying. Without the engine this chip transmits into nowhere and
         * never receives, so the firmware comes up and then waits for ever on a
         * transmission that cannot complete, which looks like a hang rather
         * than like a missing argument. */
        warn_report("sx1262: no bridge=, so this node is deaf and mute");
    }

    qdev_init_gpio_in_named(DEVICE(dev), sx1262_nss, "sx1262-nss", 1);
    qdev_init_gpio_in_named(DEVICE(dev), sx1262_fem, "sx1262-fem", 1);
    qdev_init_gpio_out_named(DEVICE(dev), &s->busy_out, "sx1262-busy", 1);
    qdev_init_gpio_out_named(DEVICE(dev), &s->dio1_out, "sx1262-dio1", 1);
    /* Nothing to report until the chip says otherwise. BUSY is held low for the
     * life of the node: the model does not represent the time a real part
     * spends digesting a command, and answering differently here would give an
     * emulated node a different radio from a native one. */
    qemu_set_irq(s->busy_out, s->lib.busy(s->chip));
    qemu_set_irq(s->dio1_out, 0);
}

static void sx1262_unrealize(DeviceState *dev)
{
    SX1262State *s = SX1262(dev);

    sx1262_engine_down(s, "the node is shutting down");
    if (s->chip) {
        s->lib.destroy(s->chip);
        s->chip = NULL;
    }
    if (s->lib.module) {
        g_module_close(s->lib.module);
        s->lib.module = NULL;
    }
}

/*
 * Not migratable, and this says so rather than pretending.
 *
 * The chip's state lives inside a library this device loaded, reachable only
 * through an ABI with no way to serialise it, and half of what decides this
 * node's behaviour is in the simulator at the other end of a socket. A snapshot
 * that restored the guest and not the radio would come back as a board whose
 * firmware believes it is mid-transaction with a chip that has never heard of
 * it. The device it replaces claimed to be migratable and was not.
 */
static const VMStateDescription vmstate_sx1262 = {
    .name = "sx1262",
    .unmigratable = 1,
};

static Property sx1262_properties[] = {
    DEFINE_PROP_STRING("bridge", SX1262State, bridge),
    DEFINE_PROP_END_OF_LIST(),
};

static void sx1262_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);

    k->realize = sx1262_realize;
    dc->unrealize = sx1262_unrealize;
    /* transfer_raw, not transfer: the chip select arrives on a GPIO line
     * rather than through the SSI bus, so qdev would never select this
     * device. Framing comes from the NSS handler above instead. */
    k->transfer_raw = sx1262_transfer;

    dc->desc = "SX1262 LoRa transceiver, MeshBench's virtual-sx1262";
    dc->vmsd = &vmstate_sx1262;
    device_class_set_props(dc, sx1262_properties);
}

static const TypeInfo sx1262_info = {
    .name          = TYPE_SX1262,
    .parent        = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(SX1262State),
    .class_init    = sx1262_class_init,
};

static void sx1262_register_types(void)
{
    type_register_static(&sx1262_info);
}

type_init(sx1262_register_types)
