#pragma once

#include "hw/sysbus.h"
#include "hw/hw.h"
#include "hw/registerfields.h"

#define TYPE_ESP32_GPIO "esp32.gpio"
#define ESP32_GPIO(obj)             OBJECT_CHECK(Esp32GpioState, (obj), TYPE_ESP32_GPIO)
#define ESP32_GPIO_GET_CLASS(obj)   OBJECT_GET_CLASS(Esp32GpioClass, obj, TYPE_ESP32_GPIO)
#define ESP32_GPIO_CLASS(klass)     OBJECT_CLASS_CHECK(Esp32GpioClass, klass, TYPE_ESP32_GPIO)

REG32(GPIO_OUT,           0x0004)
REG32(GPIO_OUT_W1TS,      0x0008)
REG32(GPIO_OUT_W1TC,      0x000C)
REG32(GPIO_OUT1,          0x0010)
REG32(GPIO_OUT1_W1TS,     0x0014)
REG32(GPIO_OUT1_W1TC,     0x0018)
REG32(GPIO_ENABLE,        0x0020)
REG32(GPIO_ENABLE_W1TS,   0x0024)
REG32(GPIO_ENABLE_W1TC,   0x0028)
REG32(GPIO_ENABLE1,       0x002C)
REG32(GPIO_ENABLE1_W1TS,  0x0030)
REG32(GPIO_ENABLE1_W1TC,  0x0034)
REG32(GPIO_STRAP,         0x0038)
REG32(GPIO_IN,            0x003C)
REG32(GPIO_IN1,           0x0040)

/* Interrupt status, and the per-pin configuration that decides what lands in
 * it. Needed because an input edge is the only way a peripheral can tell the
 * firmware something happened: a driver that attaches an interrupt handler is
 * waiting on GPIO_STATUS, and a controller that only records levels leaves it
 * waiting for ever. */
REG32(GPIO_STATUS,        0x0044)
REG32(GPIO_STATUS_W1TS,   0x0048)
REG32(GPIO_STATUS_W1TC,   0x004C)
REG32(GPIO_STATUS1,       0x0050)
REG32(GPIO_STATUS1_W1TS,  0x0054)
REG32(GPIO_STATUS1_W1TC,  0x0058)
REG32(GPIO_ACPU_INT,      0x005C)
REG32(GPIO_ACPU_INT1,     0x0074)
REG32(GPIO_PCPU_INT,      0x0064)
REG32(GPIO_PCPU_INT1,     0x007C)
REG32(GPIO_PIN0,          0x0088)

/* GPIO_PINn_REG: how this pin interrupts, and who it interrupts. */
#define ESP32_GPIO_PIN_INT_TYPE_SHIFT 7
#define ESP32_GPIO_PIN_INT_TYPE_MASK  0x7
#define ESP32_GPIO_PIN_INT_ENA_SHIFT  13
#define ESP32_GPIO_PIN_INT_ENA_MASK   0x1F

enum {
    ESP32_GPIO_INT_DISABLED = 0,
    ESP32_GPIO_INT_RISING   = 1,
    ESP32_GPIO_INT_FALLING  = 2,
    ESP32_GPIO_INT_ANY_EDGE = 3,
    ESP32_GPIO_INT_LOW      = 4,
    ESP32_GPIO_INT_HIGH     = 5,
};

#define ESP32_STRAP_MODE_FLASH_BOOT 0x12
#define ESP32_STRAP_MODE_UART_BOOT  0x0f

/* The ESP32 has 40 GPIOs, in two banks of 32 and 8. The ESP32-S3 has 49, in
 * banks of 32 and 17, and subclasses this model to say so - the register layout
 * and the propagation are the same, only the count differs. The array is sized
 * for the largest part in the family; how many lines are actually offered comes
 * from the class, so an ESP32 does not advertise pins it does not have. */
#define ESP32_GPIO_PIN_COUNT 40
#define ESP32_GPIO_PIN_MAX   49

/* Names for the qdev GPIO arrays, so a board can wire a peripheral to a pin:
 *
 *   qdev_connect_gpio_out_named(gpio, ESP32_GPIO_OUT, 18, sink)   pin -> device
 *   qdev_get_gpio_in_named(gpio, ESP32_GPIO_IN, 35)               device -> pin
 */
#define ESP32_GPIO_OUT "esp32-gpio-out"
#define ESP32_GPIO_IN  "esp32-gpio-in"

typedef struct Esp32GpioState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t strap_mode;

    /* Pin state, in the two banks the hardware registers use. `in` holds the
     * level the CPU reads: for a pin driven by the SoC that is the value it is
     * driving, and for an input it is whatever a peripheral last set. */
    uint32_t out;
    uint32_t out1;
    uint32_t enable;
    uint32_t enable1;
    uint32_t in;
    uint32_t in1;

    /* Latched interrupt status, and the per-pin configuration written by the
     * driver. Cleared by the W1TC registers, exactly as the handler does. */
    uint32_t status;
    uint32_t status1;
    uint32_t pin[ESP32_GPIO_PIN_MAX];

    qemu_irq out_irq[ESP32_GPIO_PIN_MAX];

    /* Taken from the class at instance_init; see Esp32GpioClass. */
    uint32_t pin_count;
} Esp32GpioState;

typedef struct Esp32GpioClass {
    SysBusDeviceClass parent_class;

    /* How many GPIOs this part actually has. */
    uint32_t pin_count;

    /* Byte offset of GPIO_PIN0_REG, the first per-pin config register. The
     * ESP32-S3 has extra per-core interrupt registers ahead of it, so its
     * block starts at 0x74 where the ESP32's starts at 0x88. The per-pin
     * interrupt config (and so every pin interrupt, the SX1262's DIO1 among
     * them) lands on the wrong pin if this is read from the wrong base. */
    uint32_t pin0_offset;

    /* Byte offset of the bank-1 per-CPU interrupt status register
     * (GPIO_PCPU_INT1 / GPIO_STATUS_NEXT1), which a level-1 interrupt handler
     * reads to find which pin above 32 fired. The ESP32 has it at 0x7C, the
     * ESP32-S3 at 0x68. Reading it from the wrong offset returns zero, so the
     * handler never finds the SX1262's DIO1 on pin 33 and the board never
     * relays. Mirrors GPIO_STATUS1. */
    uint32_t pcpu_int1_offset;
} Esp32GpioClass;
