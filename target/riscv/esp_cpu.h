/*
 * Espressif RISC-V CPU
 *
 * Copyright (c) 2023 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#pragma once

#include "hw/sysbus.h"
#include "hw/hw.h"
#include "hw/registerfields.h"
#include "cpu.h"

/* Make sure we are not in CONFIG_USER_ONLY */
#if defined(CONFIG_USER_ONLY) || !defined(CONFIG_SOFTMMU)
#error "ESP RISC-V Core only works in system emulation and in SOFTMMU configuration"
#endif

#include "hw/core/tcg-cpu-ops.h"

#define ESP_CPU_IRQ_LINES_NAME "espressif-cpu-irq-lines"

#define TYPE_ESP_RISCV_CPU      "espressif-riscv-cpu"
#define ESP_CPU(obj)            OBJECT_CHECK(EspRISCVCPU, (obj), TYPE_ESP_RISCV_CPU)
#define ESP_CPU_GET_CLASS(obj)  OBJECT_GET_CLASS(EspRISCVCPUClass, obj, TYPE_ESP_RISCV_CPU)
#define ESP_CPU_CLASS(klass)    OBJECT_CLASS_CHECK(EspRISCVCPUClass, klass, TYPE_ESP_RISCV_CPU)

#define ESP_CPU_INT_LINES   31


/* Define a type that will be used to generate a cycle counter */
typedef struct {
    uint64_t former_time;
    uint64_t former_rem_cycles;
    uint64_t cycles;
    /* The number of picoseconds an instruction takes to execute */
    uint64_t divider;
} ESPCPUCycleCounter;

/**
 * Espressif's RISC-V core is different from standard RISC-V because of the way interrupts are handled.
 * Extend the standard RISC-V core implementation.
 */
typedef struct EspRISCVCPU {
    /*< private >*/
    RISCVCPU parent_obj;

    /* Cycle counts */
    ESPCPUCycleCounter cc_user;
    ESPCPUCycleCounter cc_machine;

    /*< public >*/
    /* The parent object already has a reset vector property */
    uint32_t hartid_base;
    /* Parent IRQ_M line */
    qemu_irq parent_irq;
    /* Bitmap of interrupt matrix output lines currently asserted on CPU inputs. */
    uint32_t irq_lines;
} EspRISCVCPU;


typedef struct EspRISCVCPUClass {
    /*< private >*/
    RISCVCPUClass parent_class;
    DeviceRealize parent_realize;
    DeviceReset parent_reset;
    bool (*parent_exec_interrupt)(CPUState *cpu, int interrupt_request);

} EspRISCVCPUClass;
