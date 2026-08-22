/*
 * ESP32-S3 SAR ADC, as much of it as a battery divider needs.
 *
 * Modelled because an unmodelled one is not quiet. Measured on the published
 * companion build: with nothing here, the firmware started a conversion and
 * read the done bit 28,930,251 times in a single run, because the bit it was
 * waiting for could never arrive. An unmodelled output is ignored; an
 * unmodelled input is a firmware that never gets its answer.
 *
 * What a conversion is, on this part: the driver writes the channel it wants
 * into SAR1_EN_PAD, drops MEAS1_START_SAR and raises it again, then polls
 * MEAS1_DONE_SAR until it sets and reads the result out of the low half of the
 * same register. So the whole peripheral, from the guest's side, is one
 * register - and the rest of the block is kept as written so that reading back
 * a configuration returns the configuration.
 *
 * The reading itself comes from the simulation rather than from here. A cell
 * voltage invented by the emulator would be a number nobody could trace, and
 * the node already has a battery the engine is tracking; it arrives on the
 * same channel the board's buttons and keys do, tagged so each device keeps
 * its own.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/cutils.h"
#include "qemu/error-report.h"
#include "qemu/main-loop.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "hw/misc/esp32s3_saradc.h"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

/* SENS_SAR_MEAS1_CTRL2_REG, the one register that is not just storage. */
#define A_MEAS1_CTRL2      0x0C
#define R_MEAS1_DATA_MASK  0xFFFF
#define R_MEAS1_DONE       (1u << 16)
#define R_MEAS1_START      (1u << 17)
#define R_SAR1_EN_PAD_SHIFT 19
#define R_SAR1_EN_PAD_MASK  0xFFF

#define ADC_MSG 8
#define ADC_TAG 'A'

/* Which channel the driver asked for. The field is a mask of pads rather than
 * an index, and a driver that has enabled none is measuring nothing, so the
 * lowest set bit is the answer and no bits set is channel zero. */
static int adc_channel(uint32_t ctrl2)
{
    uint32_t pads = (ctrl2 >> R_SAR1_EN_PAD_SHIFT) & R_SAR1_EN_PAD_MASK;

    for (int i = 0; i < ESP32S3_SARADC_CHANNELS; i++) {
        if (pads & (1u << i)) {
            return i;
        }
    }
    return 0;
}

static void esp32s3_saradc_recv(void *opaque)
{
    Esp32s3SarAdcState *s = ESP32S3_SARADC(opaque);

    for (;;) {
        ssize_t n = recv(s->fd, s->buf + s->have, ADC_MSG - s->have, 0);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return;
            }
            n = 0;
        }
        if (n == 0) {
            qemu_set_fd_handler(s->fd, NULL, NULL, NULL);
            close(s->fd);
            s->fd = -1;
            return;
        }
        s->have += (int)n;
        if (s->have < ADC_MSG) {
            return;
        }
        s->have = 0;
        if (s->buf[0] != ADC_TAG) {
            continue;
        }
        uint8_t ch = s->buf[1];
        if (ch < ESP32S3_SARADC_CHANNELS) {
            s->raw[ch] = (uint16_t)(s->buf[2] | s->buf[3] << 8);
        }
    }
}

static void esp32s3_saradc_connect(Esp32s3SarAdcState *s)
{
    struct sockaddr_un addr;

    if (s->fd >= 0 || !s->path || !*s->path) {
        return;
    }
    if (strlen(s->path) >= sizeof(addr.sun_path)) {
        warn_report("esp32s3.saradc: socket path is too long");
        s->path = NULL;
        return;
    }
    int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK, 0);
    if (fd < 0) {
        return;
    }
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    pstrcpy(addr.sun_path, sizeof(addr.sun_path), s->path);
    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0 &&
        errno != EINPROGRESS) {
        close(fd);
        return;
    }
    s->fd = fd;
    qemu_set_fd_handler(fd, esp32s3_saradc_recv, NULL, s);
}

static uint64_t esp32s3_saradc_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32s3SarAdcState *s = ESP32S3_SARADC(opaque);
    uint32_t v = s->reg[addr / 4];

    if (addr == A_MEAS1_CTRL2) {
        /* A conversion on this part takes tens of microseconds and the driver
         * spins on the bit rather than waiting on an interrupt. Answering the
         * first poll costs the guest nothing it would notice and saves the
         * host a timer for a peripheral nobody is timing. */
        if (v & R_MEAS1_START) {
            v |= R_MEAS1_DONE;
            v = (v & ~R_MEAS1_DATA_MASK) | s->raw[adc_channel(v)];
        }
    }
    return v;
}

static void esp32s3_saradc_write(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned int size)
{
    Esp32s3SarAdcState *s = ESP32S3_SARADC(opaque);

    if (addr == A_MEAS1_CTRL2) {
        /* Done and data are the peripheral's to say. Keeping them out of what
         * is stored is what makes dropping start and raising it again start a
         * new conversion rather than return the last one. */
        value &= ~(uint64_t)(R_MEAS1_DONE | R_MEAS1_DATA_MASK);
    }
    s->reg[addr / 4] = (uint32_t)value;
}

static const MemoryRegionOps esp32s3_saradc_ops = {
    .read = esp32s3_saradc_read,
    .write = esp32s3_saradc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void esp32s3_saradc_realize(DeviceState *dev, Error **errp)
{
    Esp32s3SarAdcState *s = ESP32S3_SARADC(dev);

    s->fd = -1;
    s->have = 0;
    esp32s3_saradc_connect(s);
}

static void esp32s3_saradc_init(Object *obj)
{
    Esp32s3SarAdcState *s = ESP32S3_SARADC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_saradc_ops, s,
                          TYPE_ESP32S3_SARADC, ESP32S3_SARADC_REGS);
    sysbus_init_mmio(sbd, &s->iomem);
}

static Property esp32s3_saradc_props[] = {
    DEFINE_PROP_STRING("path", Esp32s3SarAdcState, path),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32s3_saradc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = esp32s3_saradc_realize;
    device_class_set_props(dc, esp32s3_saradc_props);
}

static const TypeInfo esp32s3_saradc_info = {
    .name = TYPE_ESP32S3_SARADC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32s3SarAdcState),
    .instance_init = esp32s3_saradc_init,
    .class_init = esp32s3_saradc_class_init,
};

static void esp32s3_saradc_register_types(void)
{
    type_register_static(&esp32s3_saradc_info);
}

type_init(esp32s3_saradc_register_types)
