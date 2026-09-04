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

    /* The per-pin config block, taken before the switch for the same reason as
     * the write path: on the ESP32-S3 it overlaps the ESP32's INT1 registers. */
    Esp32GpioClass *cls = ESP32_GPIO_GET_CLASS(s);
    uint32_t pin0 = cls->pin0_offset;
    if (addr >= pin0 && addr < pin0 + 4 * ESP32_GPIO_PIN_MAX) {
        return s->pin[(addr - pin0) / 4];
    }
    /* The bank-1 per-CPU interrupt status, whose offset differs by part (see
     * pcpu_int1_offset). Taken before the switch because on the ESP32-S3 its
     * 0x68 has no fixed case, and the ISR needs the real bank-1 status here to
     * find the pin that fired. Mirrors GPIO_STATUS1. */
    if (addr == cls->pcpu_int1_offset) {
        return s->status1;
    }

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
    case A_GPIO_STATUS:
        r = s->status;
        break;
    case A_GPIO_STATUS1:
        r = s->status1;
        break;
    /* The per-core views of the same latched status.
     *
     * A handler reads the register for the core it is running on and
     * dispatches the pins it finds set there. Both cores get the whole status
     * rather than the slice their own enable bit asks for: the driver routes
     * its interrupt to whichever core allocated it, which need not be the core
     * the pin's enable bit names, and a handler that finds zero in its own
     * register returns without calling anything. Reporting only one core's
     * view is indistinguishable, from the guest, from an interrupt that never
     * happened. */
    case A_GPIO_PCPU_INT:
    case A_GPIO_ACPU_INT:
        r = s->status;
        break;
    case A_GPIO_PCPU_INT1:
    case A_GPIO_ACPU_INT1:
        r = s->status1;
        break;
    default:
        /* Per-pin config is handled above the switch. */
        break;
    }
    return r;
}

/* The controller's line into the interrupt matrix.
 *
 * Level-driven from the latched status rather than pulsed on each edge: a
 * handler that clears one pin while another is still pending must be entered
 * again, and only a level can do that.
 */
static void esp32_gpio_update_irq(Esp32GpioState *s)
{
    qemu_set_irq(s->irq, (s->status | s->status1) != 0);
}

/* Latch an input change if the pin was configured to care about it.
 *
 * Silence is the default: a pin whose INT_ENA is clear, or whose INT_TYPE is
 * disabled, records its new level and nothing more. That is what every pin in
 * this machine did before anything needed an interrupt, and it stays true for
 * all of them but the one that asked.
 */
static void esp32_gpio_latch_int(Esp32GpioState *s, int n, bool old_level,
                                 bool level)
{
    uint32_t cfg = s->pin[n];
    unsigned type = (cfg >> ESP32_GPIO_PIN_INT_TYPE_SHIFT) &
                    ESP32_GPIO_PIN_INT_TYPE_MASK;
    unsigned ena = (cfg >> ESP32_GPIO_PIN_INT_ENA_SHIFT) &
                   ESP32_GPIO_PIN_INT_ENA_MASK;
    bool fire = false;

    if (ena == 0) {
        return;
    }
    switch (type) {
    case ESP32_GPIO_INT_RISING:
        fire = !old_level && level;
        break;
    case ESP32_GPIO_INT_FALLING:
        fire = old_level && !level;
        break;
    case ESP32_GPIO_INT_ANY_EDGE:
        fire = old_level != level;
        break;
    case ESP32_GPIO_INT_LOW:
        /* Level-triggered: pending while the pin holds the level, not on an
         * edge. A level source re-asserts the instant a handler clears it and
         * only drops when the peripheral releases the line - so the status is
         * re-evaluated on every clear (see the STATUS writes) and the guest is
         * responsible for de-asserting the cause, exactly as on the part. The
         * SX1262's BUSY and DIO1 are wired this way, so a firmware that arms
         * them as level - mesh-rs does - gets nothing without this. */
        fire = !level;
        break;
    case ESP32_GPIO_INT_HIGH:
        fire = level;
        break;
    default:
        return;
    }
    if (!fire) {
        return;
    }
    if (n < ESP32_GPIO_BANK1_FIRST) {
        s->status |= 1u << n;
    } else {
        s->status1 |= 1u << (n - ESP32_GPIO_BANK1_FIRST);
    }
    esp32_gpio_update_irq(s);
}

/* Re-evaluate every pin's level interrupt against the level it is holding. A
 * level source a handler cleared but that the peripheral has not released
 * becomes pending again, which is what the hardware does and what a driver
 * that waits on it needs. Passing old==new means edge-typed pins see no edge
 * and stay quiet, so this is safe to run over the whole bank. */
static void esp32_gpio_reassert_levels(Esp32GpioState *s)
{
    for (int n = 0; n < (int)s->pin_count; n++) {
        bool level = n < ESP32_GPIO_BANK1_FIRST
            ? !!(s->in & (1u << n))
            : !!(s->in1 & (1u << (n - ESP32_GPIO_BANK1_FIRST)));
        esp32_gpio_latch_int(s, n, level, level);
    }
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

    /* GPIO_PINn: which edge this pin interrupts on, and whether it does. Taken
     * before the register switch because on the ESP32-S3 this block starts at
     * 0x74, overlapping the ESP32's ACPU/PCPU_INT1 registers, and the per-pin
     * config must win there or every pin interrupt is armed on the wrong pin. A
     * level-triggered pin can already be asserted when the driver enables it,
     * so the configuration is re-evaluated against the level the pin holds
     * rather than waiting for a change that has already happened. */
    uint32_t pin0 = ESP32_GPIO_GET_CLASS(s)->pin0_offset;
    if (addr >= pin0 && addr < pin0 + 4 * ESP32_GPIO_PIN_MAX) {
        int n = (addr - pin0) / 4;
        bool level = n < ESP32_GPIO_BANK1_FIRST
            ? !!(s->in & (1u << n))
            : !!(s->in1 & (1u << (n - ESP32_GPIO_BANK1_FIRST)));
        s->pin[n] = (uint32_t)value;
        esp32_gpio_latch_int(s, n, level, level);
        return;
    }

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

    /* Write-one-to-clear, which is how a handler acknowledges. Dropping the
     * line when the last bit goes is what stops the handler being re-entered
     * for ever. */
    case A_GPIO_STATUS:
    case A_GPIO_STATUS_W1TC:
        s->status &= ~(uint32_t)value;
        esp32_gpio_update_irq(s);
        esp32_gpio_reassert_levels(s);
        break;
    case A_GPIO_STATUS_W1TS:
        s->status |= (uint32_t)value;
        esp32_gpio_update_irq(s);
        break;
    case A_GPIO_STATUS1:
    case A_GPIO_STATUS1_W1TC:
        s->status1 &= ~(uint32_t)value;
        esp32_gpio_update_irq(s);
        esp32_gpio_reassert_levels(s);
        break;
    case A_GPIO_STATUS1_W1TS:
        s->status1 |= (uint32_t)value;
        esp32_gpio_update_irq(s);
        break;

    default:
        /* The per-pin config block is handled above the switch. Anything else
         * unmodelled is ignored. */
        return;
    }

    esp32_gpio_update(s, old_out, old_out1);
}

/* A peripheral driving one of the SoC's pins. */
static void esp32_gpio_set_input(void *opaque, int n, int level)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);

    /* s->pin_count, not ESP32_GPIO_PIN_COUNT. The constant is the ESP32's 40
     * and is right only as this class's default; the S3 subclass raises the
     * instance's count to 49, which is why every other loop in this file reads
     * it from the instance.
     *
     * Bounding a peripheral's input by the constant silently dropped every
     * level on pins 40 to 48 - it returns, so nothing is logged and no guest
     * register moves. The LilyGo T-Deck wires the SX1262's DIO1 to GPIO 45,
     * and MeshCore collects a packet only from the ISR that line fires: the
     * chip raised RxDone, routed it to DIO1 as it should, the firmware read
     * the status and cleared it, and the packet was never collected. That
     * board heard every advert and forwarded none, while boards on pins 14,
     * 33 and 39 forwarded normally - the whole difference being which side of
     * 40 the pin sits on.
     */
    if (n < 0 || n >= (int)s->pin_count) {
        return;
    }
    bool old_level;

    if (n < 32) {
        old_level = !!(s->in & (1u << n));
        s->in = (s->in & ~(1u << n)) | ((uint32_t)!!level << n);
    } else {
        int b = n - ESP32_GPIO_BANK1_FIRST;
        old_level = !!(s->in1 & (1u << b));
        s->in1 = (s->in1 & ~(1u << b)) | ((uint32_t)!!level << b);
    }
    esp32_gpio_latch_int(s, n, old_level, !!level);
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
    /*
     * GPIO0 comes up high. It is a strapping pin with an internal pull-up
     * enabled out of reset on both these parts - which is the only reason the
     * chip boots from flash instead of into download mode - and a board's
     * program button pulls it down when somebody presses it.
     *
     * Reading it low is not a neutral default. MeshCore's repeater watches
     * this pin for a long press and powers the board off when it sees one, so
     * a Heltec V3 that came up with the button apparently held down shut
     * itself down after two minutes, every time, before it had adverted once.
     */
    s->in = 1u << 0;
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
        VMSTATE_UINT32(status, Esp32GpioState),
        VMSTATE_UINT32(status1, Esp32GpioState),
        VMSTATE_UINT32_ARRAY(pin, Esp32GpioState, ESP32_GPIO_PIN_MAX),
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
    ESP32_GPIO_CLASS(klass)->pin0_offset = A_GPIO_PIN0;
    ESP32_GPIO_CLASS(klass)->pcpu_int1_offset = A_GPIO_PCPU_INT1;
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
