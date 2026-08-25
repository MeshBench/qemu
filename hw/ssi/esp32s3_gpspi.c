/*
 * ESP32-S3 general-purpose SPI master (GPSPI2, GPSPI3).
 *
 * What a LoRa radio on one of these boards is actually wired to. Only the
 * master half is modelled, and only the parts a driver uses to move bytes: a
 * transfer is set up by writing the data words, saying how many bits are in
 * them, and then setting SPI_USR. Chip select is left alone deliberately -
 * every board here drives the radio's NSS as an ordinary GPIO, because
 * RadioLib holds it low across a whole command while the controller clocks the
 * bytes out a transfer at a time.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/ssi/esp32s3_gpspi.h"

static uint32_t esp32s3_gpspi_word(const ESP32S3GpspiState *s, unsigned i)
{
    return i < ESP32S3_GPSPI_BUF_WORDS ? s->data_reg[i] : 0;
}

/* Bit length as the register holds it: one less than the real count, and only
 * meaningful when the phase it belongs to is enabled. */
static unsigned bitlen_to_bytes(uint32_t bitlen)
{
    return (bitlen + 7) / 8;
}

/* Clock one byte out and take one back, on the bus this controller owns. */
static void esp32s3_gpspi_run(ESP32S3GpspiState *s)
{
    uint8_t buf[ESP32S3_GPSPI_BUF_WORDS * 4];
    unsigned bytes = 0;

    /* The command and address phases go out ahead of the data, most
     * significant byte first, when the driver has asked for them. Neither is
     * used by a radio driver, which puts its opcode in the data words like
     * everything else, but a controller that dropped them would be wrong for
     * the next device. */
    if (FIELD_EX32(s->user, GPSPI_USER, COMMAND)) {
        unsigned n = bitlen_to_bytes(
            FIELD_EX32(s->user2, GPSPI_USER2, COMMAND_BITLEN) + 1);
        uint32_t v = FIELD_EX32(s->user2, GPSPI_USER2, COMMAND_VALUE);
        for (unsigned i = 0; i < n && bytes < sizeof(buf); i++) {
            buf[bytes++] = (v >> (8 * i)) & 0xff;
        }
    }
    if (FIELD_EX32(s->user, GPSPI_USER, ADDR)) {
        unsigned n = bitlen_to_bytes(
            FIELD_EX32(s->user1, GPSPI_USER1, ADDR_BITLEN) + 1);
        for (unsigned i = 0; i < n && bytes < sizeof(buf); i++) {
            buf[bytes++] = (s->addr >> (8 * (n - 1 - i))) & 0xff;
        }
    }
    for (unsigned i = 0; i < bytes; i++) {
        ssi_transfer(s->spi, buf[i]);
    }

    unsigned data = 0;
    if (FIELD_EX32(s->user, GPSPI_USER, MOSI) ||
        FIELD_EX32(s->user, GPSPI_USER, MISO)) {
        data = bitlen_to_bytes(
            FIELD_EX32(s->ms_dlen, GPSPI_MS_DLEN, MS_DATA_BITLEN) + 1);
        if (data > sizeof(buf)) {
            data = sizeof(buf);
        }
    }
    if (data == 0) {
        return;
    }

    /* One buffer for both directions, which is what the hardware has: the
     * byte read back lands in the word the byte sent came from, and a
     * half-duplex read sends zeros to clock the answer out. */
    for (unsigned i = 0; i < data; i++) {
        buf[i] = FIELD_EX32(s->user, GPSPI_USER, MOSI)
            ? (esp32s3_gpspi_word(s, i / 4) >> (8 * (i % 4))) & 0xff
            : 0;
    }
    for (unsigned i = 0; i < data; i++) {
        buf[i] = ssi_transfer(s->spi, buf[i]) & 0xff;
    }
    if (FIELD_EX32(s->user, GPSPI_USER, MISO)) {
        for (unsigned i = 0; i < ESP32S3_GPSPI_BUF_WORDS; i++) {
            uint32_t w = 0;
            for (unsigned b = 0; b < 4; b++) {
                unsigned k = i * 4 + b;
                w |= (uint32_t)(k < data ? buf[k] : 0) << (8 * b);
            }
            s->data_reg[i] = w;
        }
    }
}

static uint64_t esp32s3_gpspi_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32S3GpspiState *s = ESP32S3_GPSPI(opaque);

    if (addr >= A_GPSPI_W0 &&
        addr < A_GPSPI_W0 + ESP32S3_GPSPI_BUF_WORDS * 4) {
        return s->data_reg[(addr - A_GPSPI_W0) / 4];
    }

    switch (addr) {
    /* USR reads back clear: a driver polls this to learn the transfer has
     * finished, and one that never clears it waits for ever. Ours finishes
     * inside the write that started it. */
    case A_GPSPI_CMD:      return 0;
    case A_GPSPI_ADDR:     return s->addr;
    case A_GPSPI_CTRL:     return s->ctrl;
    case A_GPSPI_CLOCK:    return s->clock;
    case A_GPSPI_USER:     return s->user;
    case A_GPSPI_USER1:    return s->user1;
    case A_GPSPI_USER2:    return s->user2;
    case A_GPSPI_MS_DLEN:  return s->ms_dlen;
    case A_GPSPI_MISC:     return s->misc;
    case A_GPSPI_DMA_CONF: return s->dma_conf;
    /* Transfer-done, always set for the same reason USR reads back clear. */
    case A_GPSPI_DMA_INT_RAW:
    case A_GPSPI_DMA_INT_ST:
        return 1 << 4;
    case A_GPSPI_DATE:     return 0x2101040;
    default:
        qemu_log_mask(LOG_UNIMP, "esp32s3.gpspi: unimplemented read at 0x%02x\n",
                      (unsigned)addr);
        return 0;
    }
}

static void esp32s3_gpspi_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned int size)
{
    ESP32S3GpspiState *s = ESP32S3_GPSPI(opaque);
    uint32_t v = (uint32_t)value;

    if (addr >= A_GPSPI_W0 &&
        addr < A_GPSPI_W0 + ESP32S3_GPSPI_BUF_WORDS * 4) {
        s->data_reg[(addr - A_GPSPI_W0) / 4] = v;
        return;
    }

    switch (addr) {
    case A_GPSPI_CMD:
        /* UPDATE latches the configuration and is not a transfer; only USR
         * moves bytes. Writing both at once is allowed and does one transfer. */
        if (FIELD_EX32(v, GPSPI_CMD, USR)) {
            esp32s3_gpspi_run(s);
        }
        break;
    case A_GPSPI_ADDR:     s->addr = v; break;
    case A_GPSPI_CTRL:     s->ctrl = v; break;
    case A_GPSPI_CLOCK:    s->clock = v; break;
    case A_GPSPI_USER:     s->user = v; break;
    case A_GPSPI_USER1:    s->user1 = v; break;
    case A_GPSPI_USER2:    s->user2 = v; break;
    case A_GPSPI_MS_DLEN:  s->ms_dlen = v; break;
    case A_GPSPI_MISC:     s->misc = v; break;
    case A_GPSPI_DMA_CONF: s->dma_conf = v; break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "esp32s3.gpspi: unimplemented write at 0x%02x = 0x%08x\n",
                      (unsigned)addr, v);
        break;
    }
}

static const MemoryRegionOps esp32s3_gpspi_ops = {
    .read = esp32s3_gpspi_read,
    .write = esp32s3_gpspi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32s3_gpspi_reset_hold(Object *obj, ResetType type)
{
    ESP32S3GpspiState *s = ESP32S3_GPSPI(obj);

    s->cmd = 0;
    s->addr = 0;
    s->ctrl = 0;
    s->clock = 0;
    s->user = 0;
    s->user1 = 0;
    s->user2 = 0;
    s->ms_dlen = 0;
    s->misc = 0;
    s->dma_conf = 0;
    memset(s->data_reg, 0, sizeof(s->data_reg));
}

/* esp32s3_gpspi_share_bus points one controller at another's peripherals.
 *
 * For a board whose radio, display and card slot are one bus in copper: the
 * GPIO matrix routes whichever controller the firmware picked onto those pins,
 * so the devices must answer either one. The bus stays owned by the controller
 * that created it.
 */
void esp32s3_gpspi_share_bus(ESP32S3GpspiState *from, ESP32S3GpspiState *onto)
{
    from->spi = onto->spi;
}

static void esp32s3_gpspi_init(Object *obj)
{
    ESP32S3GpspiState *s = ESP32S3_GPSPI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_gpspi_ops, s,
                          TYPE_ESP32S3_GPSPI, ESP32S3_GPSPI_IO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);

    esp32s3_gpspi_reset_hold(obj, RESET_TYPE_COLD);

    s->spi = ssi_create_bus(DEVICE(s), "spi");
    qdev_init_gpio_out_named(DEVICE(s), &s->cs_gpio[0], SSI_GPIO_CS,
                             ESP32S3_GPSPI_CS_COUNT);
}

static void esp32s3_gpspi_class_init(ObjectClass *klass, void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32s3_gpspi_reset_hold;
}

static const TypeInfo esp32s3_gpspi_info = {
    .name = TYPE_ESP32S3_GPSPI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3GpspiState),
    .instance_init = esp32s3_gpspi_init,
    .class_init = esp32s3_gpspi_class_init,
};

static void esp32s3_gpspi_register_types(void)
{
    type_register_static(&esp32s3_gpspi_info);
}

type_init(esp32s3_gpspi_register_types)
