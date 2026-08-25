/*
 * ESP32-S3 Clocks definition
 *
 * Copyright (c) 2024 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/xtensa/esp32s3_clk.h"
#include "hw/xtensa/esp32s3_clk_defs.h"

#define CLOCK_DEBUG      0
#define CLOCK_WARNING    0

static uint32_t esp32s3_read_cpu_intr(ESP32S3ClockState *s, uint32_t index)
{
    return (s->levels >> index) & 1;
}


static void esp32s3_write_cpu_intr(ESP32S3ClockState *s, uint32_t index, uint32_t value)
{
    const uint32_t field = FIELD_EX32(value, SYSTEM_CPU_INTR_FROM_CPU_0, CPU_INTR_FROM_CPU_0);
    if (field) {
        s->levels |= BIT(index);
        qemu_set_irq(s->irqs[index], 1);
    } else {
        s->levels &= ~BIT(index);
        qemu_set_irq(s->irqs[index], 0);
    }
}

static uint32_t esp32s3_clock_get_ext_dev_enc_dec_ctrl(ESP32S3ClockState *s)
{
    return s->sys_ext_dev_enc_dec_ctrl;
}

/*
 * Whether the second core may run, and telling the machine so.
 *
 * It comes up in reset with its clock gated, exactly as the part does - the
 * RESETING bit's reset value is 1 - and the firmware clears those when it has
 * an entry point to give it. Modelling that is not tidiness: the core was
 * running from the moment the machine started, so it sat in the ROM's own
 * "wait for somewhere to jump" loop and took the very first thing written to
 * the message register, whatever that was and whoever wrote it. On a firmware
 * that uses the register for a message before it starts anything, the second
 * core jumped into a peripheral and the whole system panicked - measured on
 * mesh-rs, which writes 0x600C0110 there long before it starts a core.
 */
static void esp32s3_clock_set_core1_control(ESP32S3ClockState *s, uint32_t value)
{
    s->core1_control0 = value;
    const bool held = (value & R_SYSTEM_CORE_1_CONTROL_0_REG_CONTROL_CORE_1_RESETING_MASK) ||
                      !(value & R_SYSTEM_CORE_1_CONTROL_0_REG_CONTROL_CORE_1_CLKGATE_EN_MASK) ||
                      (value & R_SYSTEM_CORE_1_CONTROL_0_REG_CONTROL_CORE_1_RUNSTALL_MASK);
    qemu_set_irq(s->core1_stall, held);
}

static uint64_t esp32s3_clock_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32S3ClockState *s = ESP32S3_CLOCK(opaque);
    uint64_t r = 0;

    switch(addr) {
        case A_SYSTEM_CORE_1_CONTROL_0_REG:
            r = s->core1_control0;
            break;
        case A_SYSTEM_CORE_1_CONTROL_1_REG:
                r = s->app_cpu_addr;
            break;
        case A_SYSTEM_CPU_PER_CONF:
            r = s->cpuperconf;
            break;
        case A_SYSTEM_SYSCLK_CONF:
            r = s->sysclk;
            break;
        case A_SYSTEM_CPU_INTR_FROM_CPU_0:
        case A_SYSTEM_CPU_INTR_FROM_CPU_1:
        case A_SYSTEM_CPU_INTR_FROM_CPU_2:
        case A_SYSTEM_CPU_INTR_FROM_CPU_3:
            r = esp32s3_read_cpu_intr(s, (addr - A_SYSTEM_CPU_INTR_FROM_CPU_0) / sizeof(uint32_t));
            break;
        case A_SYSTEM_EXTERNAL_DEVICE_ENCRYPT_DECRYPT_CONTROL:
            r = s->sys_ext_dev_enc_dec_ctrl;
            break;
        case A_SYSTEM_BT_LPCK_DIV_INT:
            r = s->bt_lpck_div_int;
            break;
        case A_SYSTEM_BT_LPCK_DIV_FRAC:
            r = s->bt_lpck_div_frac;
            break;
        default:
            /* Whatever the guest last put there. Not modelled, but remembered:
             * see ESP32S3ClockState.other. */
            r = s->other[addr / sizeof(uint32_t)];
            break;
    }
    return r;
}

static void esp32s3_clock_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned int size)
{
    ESP32S3ClockState *s = ESP32S3_CLOCK(opaque);

    switch(addr) {
        case A_SYSTEM_CORE_1_CONTROL_0_REG:
                esp32s3_clock_set_core1_control(s, (uint32_t)value);
            break;
        case A_SYSTEM_CORE_1_CONTROL_1_REG:
                s->app_cpu_addr = (uint32_t)value;
            break;
        case A_SYSTEM_CPU_INTR_FROM_CPU_0:
        case A_SYSTEM_CPU_INTR_FROM_CPU_1:
        case A_SYSTEM_CPU_INTR_FROM_CPU_2:
        case A_SYSTEM_CPU_INTR_FROM_CPU_3:
            esp32s3_write_cpu_intr(s, (addr - A_SYSTEM_CPU_INTR_FROM_CPU_0) / sizeof(uint32_t), value);
            break;
        case A_SYSTEM_EXTERNAL_DEVICE_ENCRYPT_DECRYPT_CONTROL:
            s->sys_ext_dev_enc_dec_ctrl = value;
            break;
        case A_SYSTEM_BT_LPCK_DIV_INT:
            s->bt_lpck_div_int = (uint32_t)value;
            break;
        case A_SYSTEM_BT_LPCK_DIV_FRAC:
            s->bt_lpck_div_frac = (uint32_t)value;
            break;
        default:
            /* Kept rather than dropped: the guest reads these back and builds
             * on what it finds. See ESP32S3ClockState.other. */
            s->other[addr / sizeof(uint32_t)] = (uint32_t)value;
            break;
    }
}

static const MemoryRegionOps esp32s3_clock_ops = {
    .read =  esp32s3_clock_read,
    .write = esp32s3_clock_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32s3_clock_reset_hold(Object *obj, ResetType type)
{
    ESP32S3ClockState *s = ESP32S3_CLOCK(obj);
    /* On board reset, set the proper clocks and dividers */
    s->sysclk = ( 1 << R_SYSTEM_SYSCLK_CONF_PRE_DIV_CNT_SHIFT) |
                (ESP32S3_CLK_SEL_PLL << R_SYSTEM_SYSCLK_CONF_SOC_CLK_SEL_SHIFT) |
                (40 << R_SYSTEM_SYSCLK_CONF_CLK_XTAL_FREQ_SHIFT) |
                ( 1 << R_SYSTEM_SYSCLK_CONF_CLK_DIV_EN_SHIFT);

    /* Divider for PLL clock and APB  frequency */
    s->cpuperconf = (ESP32S3_PERIOD_SEL_80 << R_SYSTEM_CPU_PER_CONF_CPUPERIOD_SEL_SHIFT) |
                    (ESP32S3_FREQ_SEL_PLL_480 << R_SYSTEM_CPU_PER_CONF_PLL_FREQ_SEL_SHIFT);

    /* And the second core held, which is the RESETING bit's own reset value. */
    esp32s3_clock_set_core1_control(s, R_SYSTEM_CORE_1_CONTROL_0_REG_CONTROL_CORE_1_RESETING_MASK);

    /* Initialize the IRQs */
    s->levels = 0;
    for (int i = 0 ; i < ESP32S3_SYSTEM_CPU_INTR_COUNT; i++) {
        qemu_irq_lower(s->irqs[i]);
    }
}

static void esp32s3_clock_realize(DeviceState *dev, Error **errp)
{
    /* Initialize the registers */
    esp32s3_clock_reset_hold(OBJECT(dev), RESET_TYPE_COLD);
}

static void esp32s3_clock_init(Object *obj)
{
    ESP32S3ClockState *s = ESP32S3_CLOCK(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_clock_ops, s,
                          TYPE_ESP32S3_CLOCK, ESP32S3_CLOCK_REGS_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);

    /* Whether the second core may run. Its reset value holds it, which is
     * what the part does. */
    qdev_init_gpio_out_named(DEVICE(obj), &s->core1_stall,
                             ESP32S3_CLOCK_CORE1_STALL_GPIO, 1);

    /* Initialize the output IRQ lines used to manually trigger interrupts */
    for (uint64_t i = 0; i < ESP32S3_SYSTEM_CPU_INTR_COUNT; i++) {
        sysbus_init_irq(sbd, &s->irqs[i]);
    }
}

static void esp32s3_clock_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ESP32S3ClockClass* esp32s3_clock = ESP32S3_CLOCK_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32s3_clock_reset_hold;
    dc->realize = esp32s3_clock_realize;

    esp32s3_clock->get_ext_dev_enc_dec_ctrl = esp32s3_clock_get_ext_dev_enc_dec_ctrl;
}

static const TypeInfo esp32s3_cache_info = {
    .name = TYPE_ESP32S3_CLOCK,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3ClockState),
    .instance_init = esp32s3_clock_init,
    .class_init = esp32s3_clock_class_init,
    .class_size = sizeof(ESP32S3ClockClass)
};

static void esp32s3_cache_register_types(void)
{
    type_register_static(&esp32s3_cache_info);
}

type_init(esp32s3_cache_register_types)
