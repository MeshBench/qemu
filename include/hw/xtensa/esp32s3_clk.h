/*
 * ESP32-S3 Clocks definition
 *
 * Copyright (c) 2023 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */
#pragma once

#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"

#define TYPE_ESP32S3_CLOCK "esp32s3.soc.clk"
#define ESP32S3_CLOCK(obj) OBJECT_CHECK(ESP32S3ClockState, (obj), TYPE_ESP32S3_CLOCK)
#define ESP32S3_CLOCK_GET_CLASS(obj) OBJECT_GET_CLASS(ESP32S3ClockClass, obj, TYPE_ESP32S3_CLOCK)
#define ESP32S3_CLOCK_CLASS(klass) OBJECT_CLASS_CHECK(ESP32S3ClockClass, klass, TYPE_ESP32S3_CLOCK)


#define ESP32S3_SYSTEM_CPU_INTR_COUNT   4

/**
 * Value for SYSTEM_SOC_CLK_SEL
 */
#define ESP32S3_CLK_SEL_XTAL    0
#define ESP32S3_CLK_SEL_PLL     1
#define ESP32S3_CLK_SEL_RCFAST  2

/**
 * Values for SYSTEM_PLL_FREQ_SEL
 */
#define ESP32S3_FREQ_SEL_PLL_480    0
#define ESP32S3_FREQ_SEL_PLL_320    1

/**
 * Values for SYSTEM_CPUPERIOD_SEL
*/
#define ESP32S3_PERIOD_SEL_80       0
#define ESP32S3_PERIOD_SEL_160      1


/* The whole SYSTEM register block: 0x0A0 is the last register the defs name,
 * and this covers it and everything below. Kept as a literal because this
 * header is included before the generated definitions are. */
#define ESP32S3_CLOCK_REGS_SIZE 0x0A4

/* The line the SYSTEM block holds the second core with. */
#define ESP32S3_CLOCK_CORE1_STALL_GPIO "core1-stall"

typedef struct ESP32S3ClockState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    /* Registers for clocks configuration and frequency dividers */
    uint32_t cpuperconf;
    uint32_t sysclk;


    /* IRQs for crosscore interrupts */
    qemu_irq irqs[ESP32S3_SYSTEM_CPU_INTR_COUNT];

    /* Bitmap that keeps the level of the IRQs */
    uint32_t levels;
    
    uint32_t app_cpu_addr;
    /*
     * SYSTEM_CORE_1_CONTROL_0: whether the second core is in reset, whether
     * its clock is gated, and whether it is stalled. All three come up holding
     * it, as the part does, and the firmware releases it when it is ready.
     */
    uint32_t core1_control0;
    qemu_irq core1_stall;
    
    uint32_t sys_ext_dev_enc_dec_ctrl;

    /* The Bluetooth controller's low-power clock divider and its source.
     *
     * Ordinary storage, and that is the whole of what they need to be: the
     * controller writes the source it wants and reads the register back to
     * find out whether the hardware took it. Dropping the write made that
     * read-back say no, and esp_bt_controller_init asserts rather than
     * returning an error - so a board with no Bluetooth rebooted in a loop
     * instead of coming up without it. */
    uint32_t bt_lpck_div_int;
    uint32_t bt_lpck_div_frac;
    /*
     * Everything else in the block, as plain storage.
     *
     * These are the peripheral clock-enable and reset registers, and this
     * machine does not gate a clock or hold a peripheral in reset - but the
     * guest sets bits in them one at a time, reading, or-ing and writing back.
     * A register that reads zero and discards writes makes every one of those
     * read-modify-writes a no-op, and the guest's picture of the chip diverges
     * from the chip on the very first one.
     */
    uint32_t other[ESP32S3_CLOCK_REGS_SIZE / sizeof(uint32_t)];
} ESP32S3ClockState;

typedef struct ESP32S3ClockClass {
    SysBusDeviceClass parent_class;
    /* Virtual methods */
    uint32_t (*get_ext_dev_enc_dec_ctrl)(ESP32S3ClockState *s);
} ESP32S3ClockClass;

