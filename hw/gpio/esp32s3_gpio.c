/*
 * ESP32-S3 GPIO emulation
 *
 * Copyright (c) 2023 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/gpio/esp32s3_gpio.h"


static void esp32s3_gpio_init(Object *obj)
{
    /* Set the default value for the property */
    object_property_set_int(obj, "strap_mode", ESP32S3_STRAP_MODE_FLASH_BOOT, &error_fatal);
}

/* If we need to override any function from the parent (reset, realize, ...), it shall be done
 * in this class_init function */
static void esp32s3_gpio_class_init(ObjectClass *klass, void *data)
{
    /* 49 GPIOs, against the ESP32's 40. The two banks and the propagation to
     * wired peripherals are the parent's, but the per-pin config block is not:
     * the S3 has extra per-core interrupt registers before it, so GPIO_PIN0_REG
     * sits at 0x74 rather than the ESP32's 0x88. Without this the firmware's
     * per-pin interrupt config lands on the wrong pin - the SX1262's DIO1 (pin
     * 33) config was arriving on pin 28, so the packet-received interrupt never
     * fired and no ESP32-S3 board ever relayed. */
    ESP32_GPIO_CLASS(klass)->pin_count = ESP32S3_GPIO_PIN_COUNT;
    ESP32_GPIO_CLASS(klass)->pin0_offset = 0x0074;
    /* GPIO_PCPU_INT1_REG - the bank-1 per-CPU interrupt status the ISR reads to
     * find a pin above 32. On the S3 it is at 0x68 (0x7C on the ESP32). Without
     * this the SX1262's DIO1 on pin 33 is never found and no board relays. */
    ESP32_GPIO_CLASS(klass)->pcpu_int1_offset = 0x0068;
}

static const TypeInfo esp32s3_gpio_info = {
    .name = TYPE_ESP32S3_GPIO,
    .parent = TYPE_ESP32_GPIO,
    .instance_size = sizeof(ESP32S3GPIOState),
    .instance_init = esp32s3_gpio_init,
    .class_init = esp32s3_gpio_class_init,
    .class_size = sizeof(ESP32S3GPIOClass),
};

static void esp32s3_gpio_register_types(void)
{
    type_register_static(&esp32s3_gpio_info);
}

type_init(esp32s3_gpio_register_types)
