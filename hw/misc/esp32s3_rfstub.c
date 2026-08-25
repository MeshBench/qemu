/*
 * The radio front end and its analog bus, answered rather than modelled.
 *
 * This machine has one radio and it is the LoRa transceiver on the board's SPI
 * bus. The ESP32's own Wi-Fi and Bluetooth front end is not simulated, and it
 * would not need to be if an absent peripheral were simply absent. It is not:
 * esp_wifi_init runs Espressif's PHY blob, which calibrates against this
 * hardware and waits for each step to report itself finished. With nothing
 * answering, it waits for ever - and the application never reaches the point
 * where it would draw anything. A handheld frozen on its boot logo, with
 * nothing on the console to say why.
 *
 * So the waits are answered. Every one below was read out of the blob and the
 * boot ROM rather than guessed, because guessing does not work here: the two
 * conventions are mixed. Some registers are polled until a busy bit CLEARS and
 * some until a done bit SETS, so a stub that returns all ones fixes half of
 * them and freezes the other half - which is exactly what happened on the way
 * to this table.
 *
 * What this buys is a PHY that finishes, not one that works. Nothing is ever
 * on the air. Recorded in docs/shortcomings.md, and said on the Hardware tab
 * of every ESP32 board.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"

#define TYPE_ESP32S3_RFSTUB "misc.esp32s3.rfstub"
OBJECT_DECLARE_SIMPLE_TYPE(Esp32s3RfStubState, ESP32S3_RFSTUB)

#define RFSTUB_REGS 0x1000

struct Esp32s3RfStubState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint32_t reg[RFSTUB_REGS / 4];

    /* Which block this instance is, because the answers are per register and
     * the blocks have registers at the same offsets, and how much of it to
     * claim - the analog bus sits directly below a peripheral that is
     * modelled properly, so it must not be covered over. */
    uint32_t base;
    uint32_t size;
};

/* One wait, and what has to be true for it to end.
 *
 * set is forced high on read and clear is forced low. Both are named with
 * where they came from: these are undocumented parts, and a bit with no
 * provenance is a bit nobody can check. */
typedef struct {
    uint32_t base;
    uint32_t off;
    uint32_t set;
    uint32_t clear;
} RfWait;

static const RfWait rf_waits[] = {
    /* The analog bus the PHY reaches every analog block through. The boot
     * ROM's writer composes a command, stores it, and spins on bit 31 until
     * the transfer lands: l32i / bltz at 0x40035b11, register from the
     * literal at 0x40035ac8. */
    { 0x60006000, 0x110, 0, 1u << 31 },

    /* A calibration step that reports itself finished in bit 16: bnone at
     * 0x40036606 against the mask at 0x40036588, register from the literal at
     * 0x40036584. This is the one that read eighteen million times. */
    { 0x60006000, 0x174, 1u << 16, 0 },

    /* The two analog master controllers. libphy's ram_i2c_master_reset writes
     * the reset bit and waits with bany on 0x02000000 for bit 25 to clear,
     * and ram_chip_i2c_writeReg waits the same way. */
    { 0x6000E000, 0x000, 0, 1u << 25 },
    { 0x6000E000, 0x004, 0, 1u << 25 },

    /* The Bluetooth controller's version register, which its link layer reads
     * before it will start: lld.c asserts at line 318 unless it sees exactly
     * this, and said so - "param 00000000 09001b00", what it got against what
     * it wanted. Read-only on the part, so it is pinned rather than merely
     * set: nothing the guest writes should change what the silicon says it
     * is. The number is the RW-BLE core's own version, type 9 build 0x1b. */
    { 0x60031000, 0x004, 0x09001B00, ~0x09001B00u },

    /* The Bluetooth controller's own command register, waited on the same way
     * the analog bus is: written, then polled until bit 31 clears. Its
     * control register is waited on the same way by r_rwip_driver_init, which
     * sets bit 31 and spins until the core takes it. The ROM loop is at
     * 0x4002bd75 with its register from the literal at 0x4002bd48; the other
     * is at 0x42135a55. */
    { 0x60031000, 0x000, 0, 1u << 31 },
    { 0x60031000, 0x01C, 0, 1u << 31 },

    /* The MAC, which is what the PHY hands over to. hal_init sets bit 1 and
     * waits with bbci for bit 0 on 0x60033D14 - the register from the literal
     * at 0x42178764. Nothing past here is the radio: the MAC is not modelled
     * either, and what it is being told is that it started. */
    { 0x60033000, 0xD14, 1u << 0, 0 },

    /* The analog bus's own command register. libphy's txdc_cal_v70 composes a
     * command in the low bits, stores it, and waits with bnone on 0x01000000
     * for bit 24 - the register from the literal at 0x4217fd8c and the mask
     * from 0x4217aad4, both read out of a running board rather than off the
     * disk, because the blob is relocated. The byte the analog block replies
     * with comes back in the top eight bits of the same register, which is
     * why forcing bit 24 also sets the low bit of every reading: the two
     * meanings overlap in the hardware and there is no separating them here. */
    { 0x6000E000, 0x04C, 1u << 24, 0 },

    /* And the same command register one slot down, which a newer PHY uses
     * instead. Measured on mesh-rs, an ESP-IDF v5.5 build: it clears the low
     * bits of 0x6000E040, ORs in a command, stores it and then spins with the
     * same 0x01000000 mask - the loop at 0x42254d3f with its register from
     * the literal at 0x42230170 and its mask from 0x4222fefc. Identical
     * convention, one register along, and the board sat in that loop for as
     * long as it was left running. */
    { 0x6000E000, 0x040, 1u << 24, 0 },

    /* Where rc_cal waits for its analog read or write to land - the register
     * that read fifteen million times before anything answered it. Which bit
     * means finished is not established; every high bit is set, and the low
     * byte is left as written because the same field carries the data the
     * analog block replied with. */
    { 0x6000E000, 0x050, 0xFFFFFF00, 0 },
};

static uint64_t esp32s3_rfstub_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32s3RfStubState *s = ESP32S3_RFSTUB(opaque);
    uint32_t v = s->reg[addr / 4];

    for (size_t i = 0; i < ARRAY_SIZE(rf_waits); i++) {
        if (rf_waits[i].base == s->base && rf_waits[i].off == addr) {
            return (v | rf_waits[i].set) & ~rf_waits[i].clear;
        }
    }
    return v;
}

static void esp32s3_rfstub_write(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned int size)
{
    Esp32s3RfStubState *s = ESP32S3_RFSTUB(opaque);

    s->reg[addr / 4] = (uint32_t)value;
}

static const MemoryRegionOps esp32s3_rfstub_ops = {
    .read = esp32s3_rfstub_read,
    .write = esp32s3_rfstub_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void esp32s3_rfstub_init(Object *obj)
{
    Esp32s3RfStubState *s = ESP32S3_RFSTUB(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_rfstub_ops, s,
                          TYPE_ESP32S3_RFSTUB, RFSTUB_REGS);
    sysbus_init_mmio(sbd, &s->iomem);
}

static Property esp32s3_rfstub_props[] = {
    DEFINE_PROP_UINT32("base", Esp32s3RfStubState, base, 0),
    DEFINE_PROP_UINT32("size", Esp32s3RfStubState, size, RFSTUB_REGS),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32s3_rfstub_realize(DeviceState *dev, Error **errp)
{
    Esp32s3RfStubState *s = ESP32S3_RFSTUB(dev);

    if (s->size == 0 || s->size > RFSTUB_REGS) {
        s->size = RFSTUB_REGS;
    }
    memory_region_set_size(&s->iomem, s->size);
}

static void esp32s3_rfstub_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = esp32s3_rfstub_realize;
    device_class_set_props(dc, esp32s3_rfstub_props);
}

static const TypeInfo esp32s3_rfstub_info = {
    .name = TYPE_ESP32S3_RFSTUB,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32s3RfStubState),
    .instance_init = esp32s3_rfstub_init,
    .class_init = esp32s3_rfstub_class_init,
};

static void esp32s3_rfstub_register_types(void)
{
    type_register_static(&esp32s3_rfstub_info);
}

type_init(esp32s3_rfstub_register_types)
