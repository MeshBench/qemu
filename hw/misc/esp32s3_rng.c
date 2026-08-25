/*
 * ESP32S3 Random Number Generator peripheral
 *
 * Copyright (c) 2019-2024 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/guest-random.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/misc/esp32s3_rng.h"


static uint64_t esp32s3_rng_read(void *opaque, hwaddr addr, unsigned int size)
{
    uint32_t r = 0;
    qemu_guest_getrandom_nofail(&r, sizeof(r));
    return r;
}

/*
 * The register is read-only on the part, and a write to it is discarded there.
 * It was discarded here by having no handler at all, which is not the same
 * thing: QEMU calls straight through the null pointer and the emulator dies
 * inside a guest store, with a backtrace that names memory dispatch and no
 * peripheral. Measured on mesh-rs, which writes it while it is starting.
 */
static void esp32s3_rng_write(void *opaque, hwaddr addr, uint64_t value,
                              unsigned int size)
{
}

static const MemoryRegionOps esp32s3_rng_ops = {
    .read =  esp32s3_rng_read,
    .write = esp32s3_rng_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32s3_rng_init(Object *obj)
{
    Esp32s3RngState *s = ESP32S3_RNG(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_rng_ops, s,
                          TYPE_ESP32S3_RNG, sizeof(uint32_t));
    sysbus_init_mmio(sbd, &s->iomem);
}


static const TypeInfo esp32s3_rng_info = {
    .name = TYPE_ESP32S3_RNG,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32s3RngState),
    .instance_init = esp32s3_rng_init,
};

static void esp32s3_rng_register_types(void)
{
    type_register_static(&esp32s3_rng_info);
}

type_init(esp32s3_rng_register_types)
