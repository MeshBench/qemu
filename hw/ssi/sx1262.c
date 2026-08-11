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
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/ssi/ssi.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "io/channel-socket.h"

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
};

struct SX1262State {
    SSIPeripheral parent_obj;

    char *path;                 /* unix socket of the radio model */
    QIOChannelSocket *sock;
    bool connected;
    bool cs_active;

    /* Reported once. A radio that is not there is a configuration mistake and
     * should say so on the console rather than by hanging the guest. */
    bool warned;
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

static int sx1262_set_cs(SSIPeripheral *dev, bool select)
{
    SX1262State *s = SX1262(dev);
    /* QEMU calls this with select=true meaning *asserted*; the SX1262's NSS is
     * active low, and the controller model has already applied that. */
    uint8_t req = select ? RADIO_CS_ASSERT : RADIO_CS_RELEASE;

    s->cs_active = select;
    sx1262_rpc(s, &req, 1, NULL, 0);
    return 0;
}

/* Every byte on the bus, selected or not.
 *
 * RadioLib drives NSS as an ordinary GPIO (Module(P_LORA_NSS=18, ...)), so the
 * ESP32 SPI controller never asserts its own chip select and this device would
 * otherwise never be handed a byte. Taking transfer_raw sidesteps qdev CS
 * entirely, which is correct while the radio is the only peripheral on this
 * controller.
 *
 * The cost is transaction framing: without a CS edge there is nothing to say
 * where one command ends and the next begins. Real framing needs the GPIO
 * controller to expose its output lines so NSS can drive set_cs properly, which
 * is the same change the board LEDs need.
 */
static uint32_t sx1262_transfer(SSIPeripheral *dev, uint32_t val)
{
    SX1262State *s = SX1262(dev);
    uint8_t req[2] = { RADIO_XFER, (uint8_t)val };
    uint8_t rsp = 0;

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
    SocketAddress addr = {
        .type = SOCKET_ADDRESS_TYPE_UNIX,
        .u.q_unix.path = s->path,
    };

    if (!s->path) {
        error_setg(errp, "sx1262: needs path= pointing at the radio model socket");
        return;
    }

    s->sock = qio_channel_socket_new();
    if (qio_channel_socket_connect_sync(s->sock, &addr, errp) < 0) {
        error_prepend(errp, "sx1262: cannot reach the radio model at %s: ",
                      s->path);
        return;
    }
    /* Blocking on purpose. The guest is stopped while a transaction is in
     * flight, exactly as it would be waiting on real SPI, and it keeps the
     * emulated node in step with the engine rather than racing it. */
    qio_channel_set_blocking(QIO_CHANNEL(s->sock), true, NULL);
    s->connected = true;
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
};

static void sx1262_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);

    k->realize = sx1262_realize;
    k->transfer_raw = sx1262_transfer;
    k->set_cs = sx1262_set_cs;
    k->cs_polarity = SSI_CS_LOW;

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
