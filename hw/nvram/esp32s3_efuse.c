/*
 * ESP32-S3 eFuse emulation
 *
 * Copyright (c) 2024 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "hw/nvram/esp32s3_efuse.h"


static void esp32s3_efuse_realize(DeviceState *dev, Error **errp)
{
    ESP32S3EfuseClass* esp32s3_class = ESP32S3_EFUSE_GET_CLASS(dev);

    esp32s3_class->parent_realize(dev, errp);

    /* A production ESP32-S3 ships with ADC calibration burnt in eFuse BLK2;
     * a blank block makes esp_efuse_rtc_calib_get_ver() return 0, and a
     * firmware that reads the battery in its loop then logs "calibration
     * efuse version does not match" / "No calibration efuse burnt" on every
     * reading. Seed BLK_VERSION_MAJOR = 1 (BLK2 bit 128, the low bit of the
     * fifth word) so the firmware takes the calibration path and stays quiet.
     *
     * The ADC calibration diffs are left zero, which the firmware reads as the
     * baseline V1 curve (ADC1 init codes 1850/1940/1940/2010, cal points
     * 3200/2400/1700/900 at 850 mV). That curve is not the linear default the
     * uncalibrated fallback used, so batteryMeter in the engine inverts this
     * exact curve when it encodes the cell - the reported voltage is
     * unchanged, only the warning is gone. See docs/shortcomings.md.
     *
     * Only the in-RAM default is seeded; a caller that supplies an efuse file
     * gets exactly what the file holds. */
    ESPEfuseState *s = ESP_EFUSE(dev);
    if (s->blk == NULL && s->mirror != NULL) {
        ESPEfuseBlocks *blocks = (ESPEfuseBlocks *) s->mirror;
        blocks->rd_sys_part1_data4 |= 0x1u;
    }
}


static void esp32s3_efuse_init(Object *obj)
{
}

static void esp32s3_efuse_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ESP32S3EfuseClass* esp32s3_efuse = ESP32S3_EFUSE_CLASS(klass);

    device_class_set_parent_realize(dc, esp32s3_efuse_realize, &esp32s3_efuse->parent_realize);
}

static const TypeInfo esp32s3_efuse_info = {
    .name = TYPE_ESP32S3_EFUSE,
    .parent = TYPE_ESP_EFUSE,
    .instance_size = sizeof(ESP32S3EfuseState),
    .instance_init = esp32s3_efuse_init,
    .class_init = esp32s3_efuse_class_init,
    .class_size = sizeof(ESP32S3EfuseClass)
};

static void esp32s3_efuse_register_types(void)
{
    type_register_static(&esp32s3_efuse_info);
}

type_init(esp32s3_efuse_register_types)
