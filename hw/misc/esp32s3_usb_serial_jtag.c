/*
 * ESP32-S3 USB Serial/JTAG, as a character device.
 *
 * The part this machine had was a register stub: reads answered zero, writes
 * were discarded, and there was no character backend to attach. That is fine
 * for firmware that only prints to a UART, and wrong for a great deal of the
 * firmware people actually flash. A board built with ARDUINO_USB_CDC_ON_BOOT
 * has Serial on this peripheral rather than on UART0 - the LilyGo T-Deck, the
 * RAK3112 and the Heltec Wireless Tracker among them - so on those boards the
 * whole application console went into the stub and came out nowhere, and the
 * companion protocol that rides the same port had no far end.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/error-report.h"
#include "qemu/fifo8.h"
#include "qemu/timer.h"
#include "hw/hw.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "chardev/char-fe.h"
#include "hw/misc/esp32s3_usb_serial_jtag.h"

/* Registers, at the offsets the technical reference manual gives them. */
#define A_EP1        0x00
#define A_EP1_CONF   0x04
#define A_INT_RAW    0x08
#define A_INT_ST     0x0C
#define A_INT_ENA    0x10
#define A_INT_CLR    0x14
#define A_CONF0      0x18
#define A_TEST       0x1C
#define A_JFIFO_ST   0x20
#define A_FRAM_NUM   0x24
#define A_MEM_CONF   0x60
#define A_DATE       0x80

/* EP1_CONF */
#define EP1_CONF_WR_DONE       (1 << 0)
#define EP1_CONF_IN_DATA_FREE  (1 << 1)
#define EP1_CONF_OUT_DATA_AVAIL (1 << 2)

/* Interrupts */
#define INT_JTAG_IN_FLUSH      (1 << 0)
#define INT_SOF                (1 << 1)
#define INT_SERIAL_OUT_RECV_PKT (1 << 2)
#define INT_SERIAL_IN_EMPTY    (1 << 3)
#define INT_USB_BUS_RESET      (1 << 9)
#define INT_MASK               0x7FFFF

/*
 * A full-speed host frames every millisecond. Arduino's HWCDC clears this bit
 * on every FreeRTOS tick and reads it back on the next one, and treats an
 * unset bit as the cable having been pulled - at which point it discards
 * everything written to Serial without blocking and without erroring. So the
 * frame is not decoration: it is what makes the port real.
 */
#define SOF_PERIOD_NS 1000000

static void esp32s3_usj_update_irq(ESP32S3UsbSerialJtagState *s)
{
    qemu_set_irq(s->irq, (s->int_raw & s->int_ena & INT_MASK) != 0);
}

/* Push what the guest has written towards whatever is on the other end. */
static void esp32s3_usj_flush(ESP32S3UsbSerialJtagState *s)
{
    while (!fifo8_is_empty(&s->in_fifo)) {
        uint8_t b = fifo8_pop(&s->in_fifo);
        /*
         * Best effort and never blocking. A backend nobody has connected to
         * yet - a socket with server=on,wait=off, which is how every node's
         * console is started - must not stall the guest, or a board would
         * hang at its first printf until somebody opened a window.
         */
        qemu_chr_fe_write(&s->chr, &b, 1);
    }
    /* The FIFO has been taken, which is what this interrupt means. */
    s->int_raw |= INT_SERIAL_IN_EMPTY;
    esp32s3_usj_update_irq(s);
}

static uint64_t esp32s3_usj_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32S3UsbSerialJtagState *s = ESP32S3_USB_SERIAL_JTAG(opaque);
    uint64_t r = 0;

    switch (addr) {
    case A_EP1:
        if (!fifo8_is_empty(&s->out_fifo)) {
            r = fifo8_pop(&s->out_fifo);
            if (fifo8_is_empty(&s->out_fifo)) {
                /* Room again, so the backend may hand over more. */
                qemu_chr_fe_accept_input(&s->chr);
            }
        }
        break;
    case A_EP1_CONF:
        /*
         * Always free to write: the model drains the FIFO the moment the guest
         * flushes it, so there is never a full endpoint to wait on. A driver
         * polling this is polling for a host that has already taken the data.
         */
        r = EP1_CONF_IN_DATA_FREE;
        if (!fifo8_is_empty(&s->out_fifo)) {
            r |= EP1_CONF_OUT_DATA_AVAIL;
        }
        break;
    case A_INT_RAW:
        r = s->int_raw;
        break;
    case A_INT_ST:
        r = s->int_raw & s->int_ena;
        break;
    case A_INT_ENA:
        r = s->int_ena;
        break;
    case A_JFIFO_ST:
    case A_FRAM_NUM:
    case A_CONF0:
    case A_TEST:
    case A_MEM_CONF:
        break;
    case A_DATE:
        r = 0x02012600;
        break;
    default:
        break;
    }
    return r;
}

static void esp32s3_usj_write(void *opaque, hwaddr addr, uint64_t value,
                              unsigned int size)
{
    ESP32S3UsbSerialJtagState *s = ESP32S3_USB_SERIAL_JTAG(opaque);

    switch (addr) {
    case A_EP1:
        if (fifo8_is_full(&s->in_fifo)) {
            /*
             * A driver that overruns the endpoint on real hardware loses the
             * byte, so losing it here is the honest answer rather than growing
             * the buffer and pretending the part is bigger than it is.
             */
            break;
        }
        fifo8_push(&s->in_fifo, (uint8_t)value);
        break;
    case A_EP1_CONF:
        if (value & EP1_CONF_WR_DONE) {
            esp32s3_usj_flush(s);
        }
        break;
    case A_INT_ENA:
        s->int_ena = value & INT_MASK;
        esp32s3_usj_update_irq(s);
        break;
    case A_INT_CLR:
        s->int_raw &= ~((uint32_t)value & INT_MASK);
        esp32s3_usj_update_irq(s);
        break;
    case A_INT_RAW:
        /* Write-to-clear on this part, same as INT_CLR for our purposes. */
        s->int_raw &= ~((uint32_t)value & INT_MASK);
        esp32s3_usj_update_irq(s);
        break;
    default:
        break;
    }
}

static const MemoryRegionOps esp32s3_usj_ops = {
    .read = esp32s3_usj_read,
    .write = esp32s3_usj_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static int esp32s3_usj_can_receive(void *opaque)
{
    ESP32S3UsbSerialJtagState *s = ESP32S3_USB_SERIAL_JTAG(opaque);
    return fifo8_num_free(&s->out_fifo);
}

static void esp32s3_usj_receive(void *opaque, const uint8_t *buf, int size)
{
    ESP32S3UsbSerialJtagState *s = ESP32S3_USB_SERIAL_JTAG(opaque);

    for (int i = 0; i < size && !fifo8_is_full(&s->out_fifo); i++) {
        fifo8_push(&s->out_fifo, buf[i]);
    }
    /* A packet arrived, which is the one the driver waits on. */
    s->int_raw |= INT_SERIAL_OUT_RECV_PKT;
    esp32s3_usj_update_irq(s);
}

static void esp32s3_usj_chr_event(void *opaque, QEMUChrEvent event)
{
    /*
     * Nothing to do. Deliberately no bus reset on open or close: the guest
     * treats one as the cable being pulled and drops its connected state, and
     * a console somebody attaches to a running board is not a replug.
     */
}

/* The host's frame, once a millisecond, for as long as the machine runs. */
static void esp32s3_usj_sof(void *opaque)
{
    ESP32S3UsbSerialJtagState *s = ESP32S3_USB_SERIAL_JTAG(opaque);

    s->int_raw |= INT_SOF;
    esp32s3_usj_update_irq(s);
    timer_mod_ns(s->sof_timer,
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + SOF_PERIOD_NS);
}

static void esp32s3_usj_reset_hold(Object *obj, ResetType type)
{
    ESP32S3UsbSerialJtagState *s = ESP32S3_USB_SERIAL_JTAG(obj);

    fifo8_reset(&s->in_fifo);
    fifo8_reset(&s->out_fifo);
    /*
     * SERIAL_IN_EMPTY comes up set on this part - the endpoint starts empty -
     * and a driver that waits for it before its first write would wait for
     * ever without it.
     */
    s->int_raw = INT_SERIAL_IN_EMPTY;
    s->int_ena = 0;
    esp32s3_usj_update_irq(s);
}

static void esp32s3_usj_realize(DeviceState *dev, Error **errp)
{
    ESP32S3UsbSerialJtagState *s = ESP32S3_USB_SERIAL_JTAG(dev);

    qemu_chr_fe_set_handlers(&s->chr, esp32s3_usj_can_receive,
                             esp32s3_usj_receive, esp32s3_usj_chr_event,
                             NULL, s, NULL, true);
    s->sof_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, esp32s3_usj_sof, s);
    timer_mod_ns(s->sof_timer,
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + SOF_PERIOD_NS);
}

static void esp32s3_usj_init(Object *obj)
{
    ESP32S3UsbSerialJtagState *s = ESP32S3_USB_SERIAL_JTAG(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_usj_ops, s,
                          TYPE_ESP32S3_USB_SERIAL_JTAG,
                          ESP32S3_USB_SERIAL_JTAG_REGS_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    fifo8_create(&s->in_fifo, ESP32S3_USB_SERIAL_JTAG_FIFO_SIZE);
    fifo8_create(&s->out_fifo, ESP32S3_USB_SERIAL_JTAG_FIFO_SIZE);
}

static Property esp32s3_usj_properties[] = {
    DEFINE_PROP_CHR("chardev", ESP32S3UsbSerialJtagState, chr),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32s3_usj_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32s3_usj_reset_hold;
    dc->realize = esp32s3_usj_realize;
    device_class_set_props(dc, esp32s3_usj_properties);
}

static const TypeInfo esp32s3_usj_info = {
    .name = TYPE_ESP32S3_USB_SERIAL_JTAG,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3UsbSerialJtagState),
    .instance_init = esp32s3_usj_init,
    .class_init = esp32s3_usj_class_init,
};

static void esp32s3_usj_types(void)
{
    type_register_static(&esp32s3_usj_info);
}

type_init(esp32s3_usj_types)
