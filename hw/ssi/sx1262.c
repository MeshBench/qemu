/*
 * SX1262 LoRa transceiver as a QEMU SSI peripheral.
 *
 * It models none of the radio itself. Every byte the firmware clocks out is
 * forwarded to MeshBench's own SX1262 model over a socket, and the byte that
 * model returns is clocked back in. The physics, the channel and the IRQ
 * timing all live there already, validated against RadioLib's unmodified
 * driver across hundreds of runs, and duplicating any of it here would give us
 * two models to keep in agreement.
 *
 * MeshCore's radio_init() drives this chip over GP-SPI. Without something
 * answering, RadioLib waits on a chip that never replies and the ESP32 task
 * watchdog resets the board, which is where the emulated backend stopped.
 *
 * Attach it the way the board already attaches flash:
 *
 *     -device sx1262,bus=spi2,cs=0,path=/run/user/1000/meshbench-radio-7.sock
 *     -device sx1262,bus=spi2,cs=0,path=127.0.0.1:38217
 *
 * The path is a filesystem path for a Unix socket, or anything QEMU's own
 * socket_parse() understands - so host:port everywhere else. Windows
 * is the reason: it has no Unix socket QEMU can use, and the radio model
 * already speaks TCP for the Renode backend, so one transport now serves all
 * three platforms.
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/ssi/ssi.h"
#include "hw/qdev-properties.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "io/channel-socket.h"
#include "qemu/sockets.h"
#include "qemu/timer.h"

/* How often DIO1 is sampled, in milliseconds of guest time. */
#define DIO1_POLL_MS 1

#define TYPE_SX1262 "sx1262"
OBJECT_DECLARE_SIMPLE_TYPE(SX1262State, SX1262)

/* Wire protocol to the radio model. One byte of tag, then a payload.
 *
 * Deliberately trivial: this runs inside the emulator's SPI path, so it is on
 * the hot loop of every transaction. Anything richer than "byte out, byte in"
 * would be paid for on every clock. */
enum {
    RADIO_CS_ASSERT   = 0x01,
    RADIO_CS_RELEASE  = 0x02,
    RADIO_XFER        = 0x03,   /* one byte out, one byte back */
    RADIO_READ_BUSY   = 0x04,   /* the BUSY line, which RadioLib spins on */
    /* Whether DIO1 is asserted. MeshCore reads a received packet only from the
     * interrupt this line raises - RadioLibWrapper::recvRaw is gated on a flag
     * set solely by setPacketReceivedAction - so a chip that receives
     * perfectly and has no wire to say so is a node that never forwards
     * anything. The radio model has answered this opcode since the nRF52
     * needed it; this device simply never asked. */
    RADIO_READ_IRQ    = 0x05,
    RADIO_SET_FEM     = 0x06,   /* front-end module enable, level in byte 2 */
};

struct SX1262State {
    SSIPeripheral parent_obj;

    char *path;                 /* unix socket of the radio model */
    QIOChannelSocket *sock;
    bool connected;
    bool cs_active;
    bool fem_level;

    /* Reported once. A radio that is not there is a configuration mistake and
     * should say so on the console rather than by hanging the guest. */
    bool warned;

    /* NSS in, BUSY and DIO1 out. All ordinary GPIOs on these boards rather
     * than the SPI controller's own lines, so the board wires them pin to
     * pin. */
    qemu_irq busy_out;
    qemu_irq dio1_out;

    /* DIO1 is polled rather than pushed, because the wire protocol is
     * request-response and the radio model has no way to call back. A
     * kilohertz is far finer than anything the radio times: the shortest
     * thing DIO1 signals is a preamble detection, tens of milliseconds at
     * these spreading factors. */
    QEMUTimer *dio1_timer;
    bool dio1_level;
};

static bool sx1262_rpc(SX1262State *s, const uint8_t *req, size_t req_len,
                       uint8_t *rsp, size_t rsp_len)
{
    Error *err = NULL;

    if (!s->connected) {
        return false;
    }
    if (qio_channel_write_all(QIO_CHANNEL(s->sock),
                              (const char *)req, req_len, &err) < 0) {
        goto fail;
    }
    if (rsp_len &&
        qio_channel_read_all(QIO_CHANNEL(s->sock),
                             (char *)rsp, rsp_len, &err) < 0) {
        goto fail;
    }
    return true;

fail:
    /* Drop the link rather than retry. A half-written transaction cannot be
     * recovered, and a radio that answers half a command is worse for the
     * firmware than one that answers nothing. */
    if (!s->warned) {
        error_reportf_err(err, "sx1262: radio model link lost: ");
        s->warned = true;
    } else {
        error_free(err);
    }
    s->connected = false;
    return false;
}

/* The driver toggling the chip select by hand. This is what frames a command:
 * the controller clocks bytes one transfer at a time, so without these edges
 * the model has no way to tell where one command ends and the next begins. */
static void sx1262_nss(void *opaque, int n, int level)
{
    SX1262State *s = SX1262(opaque);
    uint8_t req = level ? RADIO_CS_RELEASE : RADIO_CS_ASSERT;   /* active low */

    if (s->cs_active == !level) {
        return;
    }
    s->cs_active = !level;
    sx1262_rpc(s, &req, 1, NULL, 0);
}

/* The board's front-end module enable line: an external amplifier and antenna
 * switch that the firmware drives as an ordinary GPIO. The radio cannot see it
 * over SPI - the module sits beside the chip, not inside it, and the chip has
 * no way to know whether its output reaches an antenna. But whether the line is
 * asserted decides how much power leaves the board, so the model needs it. */
static void sx1262_fem(void *opaque, int n, int level)
{
    SX1262State *s = SX1262(opaque);
    uint8_t req[2] = { RADIO_SET_FEM, level ? 1 : 0 };

    if (s->fem_level == !!level) {
        return;
    }
    s->fem_level = !!level;
    sx1262_rpc(s, req, sizeof(req), NULL, 0);
}

/* Ask the chip whether DIO1 is asserted, and drive the pin to match.
 *
 * Edge-driven, so a quiet chip costs one request per tick and no interrupt
 * traffic at all. The line is left alone when the model cannot be reached:
 * an unattached radio should look like a chip with nothing to say, not like
 * one holding an interrupt high for ever. */
static void sx1262_poll_dio1(void *opaque)
{
    SX1262State *s = SX1262(opaque);
    uint8_t req = RADIO_READ_IRQ;
    uint8_t rsp = 0;

    if (sx1262_rpc(s, &req, 1, &rsp, 1)) {
        bool asserted = rsp != 0;

        if (asserted != s->dio1_level) {
            s->dio1_level = asserted;
            qemu_set_irq(s->dio1_out, asserted);
        }
    }
    timer_mod(s->dio1_timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + DIO1_POLL_MS);
}

static uint32_t sx1262_transfer(SSIPeripheral *dev, uint32_t val)
{
    SX1262State *s = SX1262(dev);
    uint8_t req[2] = { RADIO_XFER, (uint8_t)val };
    uint8_t rsp = 0;

    /* Bytes on this bus are not necessarily ours. A board can hang a display
     * and a card reader off the same controller as the radio, told apart only
     * by which chip select is low, and a peripheral that answered while
     * another was addressed would corrupt both. */
    if (!s->cs_active) {
        return 0x00;
    }
    if (!sx1262_rpc(s, req, sizeof(req), &rsp, 1)) {
        /* 0x00 rather than 0xFF: an all-ones read looks like a chip reporting
         * every status bit set, which sends RadioLib down error paths that
         * obscure the real problem. Zero reads as "not ready" and the driver
         * simply keeps waiting, which is what is actually true. */
        return 0x00;
    }
    return rsp;
}

static void sx1262_realize(SSIPeripheral *dev, Error **errp)
{
    SX1262State *s = SX1262(dev);
    SocketAddress *addr;

    if (!s->path) {
        error_setg(errp, "sx1262: needs path= pointing at the radio model socket");
        return;
    }

    /* socket_parse() rather than a hard-coded Unix address: it takes
     * "host:port" as TCP, which is what lets the same device work on Windows,
     * where there is no Unix socket to hand.
     *
     * A bare filesystem path is spelled for it. socket_parse() does not take
     * one - it wants the "unix:" in front - and every caller here has always
     * passed the path plain, so without this the device stopped being able to
     * find a radio it had been finding for months:
     *
     *     sx1262: cannot make sense of path=/run/.../radio.sock
     */
    if (s->path[0] == '/') {
        g_autofree char *spelled = g_strdup_printf("unix:%s", s->path);
        addr = socket_parse(spelled, errp);
    } else {
        addr = socket_parse(s->path, errp);
    }
    if (!addr) {
        error_prepend(errp, "sx1262: cannot make sense of path=%s: ", s->path);
        return;
    }

    s->sock = qio_channel_socket_new();
    if (qio_channel_socket_connect_sync(s->sock, addr, errp) < 0) {
        error_prepend(errp, "sx1262: cannot reach the radio model at %s: ",
                      s->path);
        qapi_free_SocketAddress(addr);
        return;
    }
    qapi_free_SocketAddress(addr);
    /* Blocking on purpose. The guest is stopped while a transaction is in
     * flight, exactly as it would be waiting on real SPI, and it keeps the
     * emulated node in step with the engine rather than racing it. */
    qio_channel_set_blocking(QIO_CHANNEL(s->sock), true, NULL);
    s->connected = true;

    qdev_init_gpio_in_named(DEVICE(dev), sx1262_nss, "sx1262-nss", 1);
    qdev_init_gpio_in_named(DEVICE(dev), sx1262_fem, "sx1262-fem", 1);
    qdev_init_gpio_out_named(DEVICE(dev), &s->busy_out, "sx1262-busy", 1);
    qdev_init_gpio_out_named(DEVICE(dev), &s->dio1_out, "sx1262-dio1", 1);
    /* Not busy, and nothing to report, until the model says otherwise. */
    qemu_set_irq(s->busy_out, 0);
    qemu_set_irq(s->dio1_out, 0);

    s->dio1_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, sx1262_poll_dio1, s);
    timer_mod(s->dio1_timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + DIO1_POLL_MS);
}

static const VMStateDescription vmstate_sx1262 = {
    .name = "sx1262",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_SSI_PERIPHERAL(parent_obj, SX1262State),
        VMSTATE_BOOL(cs_active, SX1262State),
        VMSTATE_END_OF_LIST()
    }
};

static Property sx1262_properties[] = {
    DEFINE_PROP_STRING("path", SX1262State, path),
    DEFINE_PROP_END_OF_LIST(),
};

static void sx1262_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);

    k->realize = sx1262_realize;
    /* transfer_raw, not transfer: the chip select arrives on a GPIO line
     * rather than through the SSI bus, so qdev would never select this
     * device. Framing comes from the NSS handler above instead. */
    k->transfer_raw = sx1262_transfer;

    dc->desc = "SX1262 LoRa transceiver, backed by MeshBench's radio model";
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
