/*
 * ESP32-S3 SAR ADC, as much of it as a battery divider needs.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#pragma once

#include "hw/hw.h"
#include "hw/sysbus.h"

#define TYPE_ESP32S3_SARADC "misc.esp32s3.saradc"
OBJECT_DECLARE_SIMPLE_TYPE(Esp32s3SarAdcState, ESP32S3_SARADC)

#define ESP32S3_SARADC_REGS 0x200
#define ESP32S3_SARADC_CHANNELS 12

struct Esp32s3SarAdcState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    /* The register file, so config written comes back as written. Only the
     * conversion registers mean anything below. */
    uint32_t reg[ESP32S3_SARADC_REGS / 4];

    /* What each channel measures, as the twelve bit number the firmware
     * reads. Set at bring-up and updated over the board's input channel. */
    uint16_t raw[ESP32S3_SARADC_CHANNELS];

    /* Where this part keeps the two registers that mean something, and which
     * bit of the temperature one says a reading is ready. The original ESP32
     * has no such bit at all, which is what zero means here. */
    uint32_t meas_off;
    uint32_t tsens_off;
    uint32_t tsens_ready_bit;

    char *path;
    int fd;
    uint8_t buf[8];
    int have;
};
