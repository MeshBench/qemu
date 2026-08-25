/*
 * ESP32-S3 USB Serial/JTAG, as a character device.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#pragma once

#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "chardev/char-fe.h"
#include "qemu/fifo8.h"
#include "qemu/timer.h"

#define TYPE_ESP32S3_USB_SERIAL_JTAG "misc.esp32s3.usb_serial_jtag"
#define ESP32S3_USB_SERIAL_JTAG(obj) \
    OBJECT_CHECK(ESP32S3UsbSerialJtagState, (obj), TYPE_ESP32S3_USB_SERIAL_JTAG)

#define ESP32S3_USB_SERIAL_JTAG_REGS_SIZE (0x84)

/* The endpoint FIFOs are 64 bytes on this part: one full-speed packet. */
#define ESP32S3_USB_SERIAL_JTAG_FIFO_SIZE 64

typedef struct ESP32S3UsbSerialJtagState {
    SysBusDevice parent_object;
    MemoryRegion iomem;
    CharBackend chr;
    qemu_irq irq;

    /* in is guest to host, out is host to guest, named as the registers are. */
    Fifo8 in_fifo;
    Fifo8 out_fifo;

    uint32_t int_raw;
    uint32_t int_ena;

    /*
     * A host sends a start-of-frame every millisecond, and the Arduino core
     * decides whether anything is plugged in by watching for one. Without it
     * every write is discarded as going nowhere.
     */
    QEMUTimer *sof_timer;
} ESP32S3UsbSerialJtagState;
