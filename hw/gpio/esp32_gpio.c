/*
 * ESP32 GPIO, enough of it to drive and observe pins.
 *
 * The model used to answer only GPIO_STRAP, and its write handler was empty, so
 * every pin the firmware drove went nowhere and every pin it read came back
 * low. That is invisible for a board doing nothing with GPIO, and fatal for one
 * where a peripheral is wired to a pin: a driver that toggles a chip select by
 * hand has no way to tell the peripheral anything, and a driver waiting on a
 * busy line waits on a constant.
 *
 * Copyright (c) 2019-2026 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/qdev-properties.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "hw/gpio/esp32_gpio.h"

/* Bank 1 covers pins 32..39. */
#define ESP32_GPIO_BANK1_FIRST 32

static uint64_t esp32_gpio_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);
    uint64_t r = 0;

    switch (addr) {
    case A_GPIO_OUT:
        r = s->out;
        break;
    case A_GPIO_OUT1:
        r = s->out1;
        break;
    case A_GPIO_ENABLE:
        r = s->enable;
        break;
    case A_GPIO_ENABLE1:
        r = s->enable1;
        break;
    case A_GPIO_STRAP:
        r = s->strap_mode;
        break;
    case A_GPIO_IN:
        r = s->in;
        break;
    case A_GPIO_IN1:
        r = s->in1;
        break;
    default:
        break;
    }
    return r;
}

/* Push the pins whose level changed out to whatever is wired to them.
 *
 * Only pins configured as outputs are driven. An input keeps whatever a
 * peripheral set, which is what makes a busy line readable at all.
 */
static void esp32_gpio_update(Esp32GpioState *s, uint32_t old_out,
                              uint32_t old_out1)
{
    uint32_t changed = (s->out ^ old_out) & s->enable;
    uint32_t changed1 = (s->out1 ^ old_out1) & s->enable1;

    for (int i = 0; i < 32; i++) {
        if (changed & (1u << i)) {
            int level = !!(s->out & (1u << i));
            /* The CPU reads back the level it is driving. */
            s->in = (s->in & ~(1u << i)) | ((uint32_t)level << i);
            qemu_set_irq(s->out_irq[i], level);
        }
    }
    for (int i = 0; i < (int)s->pin_count - ESP32_GPIO_BANK1_FIRST; i++) {
        if (changed1 & (1u << i)) {
            int level = !!(s->out1 & (1u << i));
            s->in1 = (s->in1 & ~(1u << i)) | ((uint32_t)level << i);
            qemu_set_irq(s->out_irq[ESP32_GPIO_BANK1_FIRST + i], level);
        }
    }
}

static void esp32_gpio_write(void *opaque, hwaddr addr,
                             uint64_t value, unsigned int size)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);
    uint32_t old_out = s->out;
    uint32_t old_out1 = s->out1;

    switch (addr) {
    case A_GPIO_OUT:
        s->out = value;
        break;
    case A_GPIO_OUT_W1TS:
        s->out |= value;
        break;
    case A_GPIO_OUT_W1TC:
        s->out &= ~(uint32_t)value;
        break;
    case A_GPIO_OUT1:
        s->out1 = value;
        break;
    case A_GPIO_OUT1_W1TS:
        s->out1 |= value;
        break;
    case A_GPIO_OUT1_W1TC:
        s->out1 &= ~(uint32_t)value;
        break;

    /* Enabling an output makes the pin start driving, so the update below has
     * to run for these too, not only for writes that change a level. */
    case A_GPIO_ENABLE:
        s->enable = value;
        old_out = ~s->out;
        break;
    case A_GPIO_ENABLE_W1TS:
        s->enable |= value;
        old_out = ~s->out;
        break;
    case A_GPIO_ENABLE_W1TC:
        s->enable &= ~(uint32_t)value;
        break;
    case A_GPIO_ENABLE1:
        s->enable1 = value;
        old_out1 = ~s->out1;
        break;
    case A_GPIO_ENABLE1_W1TS:
        s->enable1 |= value;
        old_out1 = ~s->out1;
        break;
    case A_GPIO_ENABLE1_W1TC:
        s->enable1 &= ~(uint32_t)value;
        break;

    default:
        return;
    }

    esp32_gpio_update(s, old_out, old_out1);
}

/* A peripheral driving one of the SoC's pins. */
static void esp32_gpio_set_input(void *opaque, int n, int level)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);

    if (n < 0 || n >= ESP32_GPIO_PIN_COUNT) {
        return;
    }
    if (n < 32) {
        s->in = (s->in & ~(1u << n)) | ((uint32_t)!!level << n);
    } else {
        int b = n - ESP32_GPIO_BANK1_FIRST;
        s->in1 = (s->in1 & ~(1u << b)) | ((uint32_t)!!level << b);
    }
}

static const MemoryRegionOps esp32_gpio_ops = {
    .read =  esp32_gpio_read,
    .write = esp32_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_gpio_reset_hold(Object *obj, ResetType type)
{
    Esp32GpioState *s = ESP32_GPIO(obj);

    s->out = 0;
    s->out1 = 0;
    s->enable = 0;
    s->enable1 = 0;
    s->in = 0;
    s->in1 = 0;
}

static void esp32_gpio_realize(DeviceState *dev, Error **errp)
{
}

static void esp32_gpio_init(Object *obj)
{
    Esp32GpioState *s = ESP32_GPIO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    /* Set the default value for the strap_mode property */
    object_property_set_int(obj, "strap_mode", ESP32_STRAP_MODE_FLASH_BOOT,
                            &error_fatal);

    memory_region_init_io(&s->iomem, obj, &esp32_gpio_ops, s,
                          TYPE_ESP32_GPIO, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    s->pin_count = ESP32_GPIO_GET_CLASS(obj)->pin_count;
    qdev_init_gpio_out_named(DEVICE(obj), s->out_irq, ESP32_GPIO_OUT,
                             s->pin_count);
    qdev_init_gpio_in_named(DEVICE(obj), esp32_gpio_set_input, ESP32_GPIO_IN,
                            s->pin_count);
}

static const VMStateDescription vmstate_esp32_gpio = {
    .name = TYPE_ESP32_GPIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(out, Esp32GpioState),
        VMSTATE_UINT32(out1, Esp32GpioState),
        VMSTATE_UINT32(enable, Esp32GpioState),
        VMSTATE_UINT32(enable1, Esp32GpioState),
        VMSTATE_UINT32(in, Esp32GpioState),
        VMSTATE_UINT32(in1, Esp32GpioState),
        VMSTATE_END_OF_LIST()
    }
};

static Property esp32_gpio_properties[] = {
    /* The strap_mode needs to be explicitly set in the instance init, thus, set
     * the default value to 0. */
    DEFINE_PROP_UINT32("strap_mode", Esp32GpioState, strap_mode, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32_gpio_class_init(ObjectClass *klass, void *data)
{
    ESP32_GPIO_CLASS(klass)->pin_count = ESP32_GPIO_PIN_COUNT;
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = esp32_gpio_realize;
    dc->vmsd = &vmstate_esp32_gpio;
    device_class_set_props(dc, esp32_gpio_properties);
    rc->phases.hold = esp32_gpio_reset_hold;
}

static const TypeInfo esp32_gpio_info = {
    .name = TYPE_ESP32_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32GpioState),
    .instance_init = esp32_gpio_init,
    .class_init = esp32_gpio_class_init,
    .class_size = sizeof(Esp32GpioClass),
};

static void esp32_gpio_register_types(void)
{
    type_register_static(&esp32_gpio_info);
}

type_init(esp32_gpio_register_types)
