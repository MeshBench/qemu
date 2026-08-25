/*
 * ESP32S3 SoC and Machine
 *
 * Copyright (c) 2023-2024 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "qemu/memalign.h"
#include "hw/hw.h"
#include "hw/boards.h"
#include "hw/loader.h"
#include "hw/sysbus.h"
#include "hw/xtensa/xtensa_memory.h"
#include "hw/misc/unimp.h"
#include "hw/irq.h"
#include "hw/i2c/i2c.h"
#include "hw/qdev-properties.h"

#include "qemu/osdep.h"
#include "hw/hw.h"
#include "target/xtensa/cpu.h"

#include "hw/misc/esp32s3_rtc_cntl.h"
#include "hw/xtensa/esp32s3_intc.h"

#include "hw/sd/dwc_sdmmc.h"
#include "hw/misc/ssi_psram.h"
#include "core-esp32s3/core-isa.h"
#include "qemu/datadir.h"
#include "sysemu/sysemu.h"
#include "sysemu/reset.h"
#include "sysemu/cpus.h"
#include "sysemu/runstate.h"
#include "sysemu/blockdev.h"
#include "sysemu/block-backend.h"
#include "exec/exec-all.h"
#include "net/net.h"
#include "elf.h"

#include "hw/ssi/esp32s3_spi.h"
#include "hw/ssi/esp32s3_gpspi.h"
#include "hw/i2c/esp32_i2c.h"
#include "hw/misc/esp32s3_cache.h"
#include "hw/char/esp32s3_uart.h"
#include "hw/misc/esp32s3_rng.h"
#include "hw/misc/esp32s3_saradc.h"
#include "hw/ssi/ssi.h"

#include "hw/nvram/esp32s3_efuse.h"
#include "hw/xtensa/esp32s3_clk.h"
#include "hw/dma/esp32s3_gdma.h"
#include "hw/misc/esp32s3_sha.h"
#include "hw/misc/esp32s3_aes.h"
#include "hw/misc/esp32s3_rsa.h"
#include "hw/misc/esp32s3_hmac.h"
#include "hw/misc/esp32s3_ds.h"
#include "hw/timer/esp32s3_timg.h"
#include "hw/timer/esp32s3_systimer.h"
#include "hw/gpio/esp32s3_gpio.h"
#include "hw/misc/esp32s3_xts_aes.h"
#include "hw/misc/esp32s3_pms.h"
#include "hw/net/can/esp32s3_twai.h"

#include "cpu_esp32s3.h"

#include "hw/misc/esp32s3_usb_serial_jtag.h"
#include "hw/display/esp_rgb.h"

#define TYPE_ESP32S3_SOC "xtensa.esp32s3"
#define ESP32S3_SOC(obj) OBJECT_CHECK(Esp32s3SocState, (obj), TYPE_ESP32S3_SOC)

#define TYPE_ESP32S3_CPU XTENSA_CPU_TYPE_NAME("esp32s3")


enum {
    ESP32S3_MEMREGION_IROM,
    ESP32S3_MEMREGION_DROM,
    ESP32S3_MEMREGION_DRAM,
    ESP32S3_MEMREGION_IRAM,
    ESP32S3_MEMREGION_ICACHE,
    ESP32S3_MEMREGION_DCACHE,
    ESP32S3_MEMREGION_RTCSLOW,
    ESP32S3_MEMREGION_RTCFAST,
    ESP32S3_MEMREGION_FRAMEBUF,
};

static const struct MemmapEntry {
    hwaddr base;
    hwaddr size;
} esp32s3_memmap[] = {
    [ESP32S3_MEMREGION_DROM] = { 0x3ff00000, 0x20000 },
    [ESP32S3_MEMREGION_IROM] = { 0x40000000, 0x60000 },
    [ESP32S3_MEMREGION_DRAM] = { 0x3FC80000, 0x170000 },
    [ESP32S3_MEMREGION_IRAM] = { 0x40370000, 0x80000 },
    [ESP32S3_MEMREGION_DCACHE] = { 0x3c000000, ESP32S3_EXTMEM_REGION_SIZE },
    [ESP32S3_MEMREGION_ICACHE] = { 0x42000000, ESP32S3_EXTMEM_REGION_SIZE },
    [ESP32S3_MEMREGION_RTCSLOW] = { 0x50000000, 0x2000 },
    [ESP32S3_MEMREGION_RTCFAST] = { 0x600fe000, 0x2000 },
    /* Virtual Framebuffer, used for the graphical interface */
    [ESP32S3_MEMREGION_FRAMEBUF] = { 0x20000000, ESP_RGB_MAX_VRAM_SIZE },
};


#define ESP32S3_SOC_RESET_PROCPU    0x1
#define ESP32S3_SOC_RESET_APPCPU    0x2
#define ESP32S3_SOC_RESET_PERIPH    0x4
#define ESP32S3_SOC_RESET_DIG       (ESP32S3_SOC_RESET_PROCPU | ESP32S3_SOC_RESET_APPCPU | ESP32S3_SOC_RESET_PERIPH)
#define ESP32S3_SOC_RESET_RTC       0x8
#define ESP32S3_SOC_RESET_ALL       (ESP32S3_SOC_RESET_RTC | ESP32S3_SOC_RESET_DIG)

#define ESP32S3_IO_WARNING  0

typedef struct Esp32s3SocState {
    /*< private >*/
    DeviceState parent_obj;

    /*< public >*/
    XtensaCPU cpu[ESP32S3_CPU_COUNT];
    Esp32s3IntMatrixState intmatrix;
    ESP32S3UARTState uart[ESP32S3_UART_COUNT];
    ESP32S3GPIOState gpio;
    Esp32s3RngState rng;
    Esp32S3TWAIState twai;

    Esp32s3RtcCntlState rtc_cntl;

    BusState rtc_bus;
    BusState periph_bus;

    MemoryRegion cpu_specific_mem[ESP32S3_CPU_COUNT];
    ESP32S3SpiState spi1;

    /* The two general-purpose SPI controllers. The flash hangs off spi1 and
     * uses a different register set entirely; these are what a board puts a
     * radio on. Arduino calls GPSPI2 "FSPI" and GPSPI3 "HSPI", and a
     * default-constructed SPIClass on this part is HSPI - so the published
     * board images drive their radio through gpspi3, not gpspi2. */
    ESP32S3GpspiState gpspi2;
    ESP32S3GpspiState gpspi3;
    /* The two I2C controllers. Every board here that carries a display puts it
     * on one of these, and the T-Deck's keyboard and touch panel as well - the
     * machine answered none of it until now, so 1,356 register accesses a boot
     * went into the peripheral window and came back zero. */
    Esp32I2CState i2c[ESP32S3_I2C_COUNT];
    ESP32S3CacheState cache;
    ESP32S3EfuseState efuse;
    ESP32S3ClockState clock;
    ESP32S3GdmaState gdma;
    ESP32S3ShaState sha;
    ESP32S3AesState aes;
    ESP32S3RsaState rsa;
    ESP32S3HmacState hmac;
    ESP32S3DsState ds;
    ESP32S3PmsState pms;

    ESP32S3XtsAesState xts_aes;
    ESP32S3TimgState timg[2];
    ESP32S3SysTimerState systimer;

    ESP32S3UsbSerialJtagState jtag;
    ESPRgbState rgb;

    MemoryRegion iomem;
    DWCSDMMCState sdmmc;
    DeviceState *eth;
    SsiPsramState *psram;

    uint32_t requested_reset;
    /* The SYSTEM block's hold on core 1: in reset, clock-gated or stalled. */
    bool core1_held;
    /* Whether the coprocessors come up enabled; see esp32s3_apply_cp_at_reset. */
    bool cp_at_reset;
} Esp32s3SocState;


/* Temporary macro to mark the CPU as in non-debugging mode */
#define A_ASSIST_DEBUG_CORE_0_DEBUG_MODE_REG    0x098

/* "QEMU" as a 32-bit value, can be used by the application to to check whether it is running in
 * QEMU or on real hardware */
#define RGB_QEMU_ORIGIN     0x51454d55
#define RGB_QEMU_ORIGIN_REG 0x3F8

static void remove_cpu_watchpoints(XtensaCPU* xcs)
{
    for (int i = 0; i < MAX_NDBREAK; ++i) {
        if (xcs->env.cpu_watchpoint[i]) {
            cpu_watchpoint_remove_by_ref(CPU(xcs), xcs->env.cpu_watchpoint[i]);
            xcs->env.cpu_watchpoint[i] = NULL;
        }
    }
}

static void esp32s3_dig_reset(void *opaque, int n, int level)
{
    Esp32s3SocState *s = ESP32S3_SOC(opaque);
    if (level) {
        s->requested_reset = ESP32S3_SOC_RESET_DIG;
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
    }
}

static void esp32s3_cpu_reset(void* opaque, int n, int level)
{
    Esp32s3SocState *s = ESP32S3_SOC(opaque);
    if (level) {
        s->requested_reset = (n == 0) ? ESP32S3_SOC_RESET_PROCPU : ESP32S3_SOC_RESET_APPCPU;
        /* Use different cause for APP CPU so that its reset doesn't cause QEMU to exit,
         * when -no-reboot option is given.
         */
        ShutdownCause cause = (n == 0) ? SHUTDOWN_CAUSE_GUEST_RESET : SHUTDOWN_CAUSE_SUBSYSTEM_RESET;
        qemu_system_reset_request(cause);
    }
}

/*
 * Bring the coprocessors up enabled, when asked, which the part does not.
 *
 * CPENABLE resets to zero on this architecture and the firmware decides which
 * tasks may use the floating point unit. That is faithful, and it is also a
 * dead end for one firmware here: its exception handler saves floating point
 * state, so it reads FCR with the unit still disabled, takes a
 * CoprocessorDisabled trap inside an exception vector - which is fatal - and
 * loops there for ever. Nothing behind that point can be seen at all.
 *
 * Off by default and named for what it is. Turning it on makes the machine
 * lie about a register in a way silicon does not, and the only reason it
 * exists is that with it on, that firmware stops looping and reaches its own
 * panic handler, which is a fault worth being able to look at.
 */
static void esp32s3_apply_cp_at_reset(Esp32s3SocState *s, int core)
{
    if (s->cp_at_reset && core < ESP32S3_CPU_COUNT) {
        s->cpu[core].env.sregs[CPENABLE] = 0xff;
    }
}

static void esp32s3_soc_reset(DeviceState *dev)
{
    Esp32s3SocState *s = ESP32S3_SOC(dev);
    if (s->requested_reset == 0) {
        s->requested_reset = ESP32S3_SOC_RESET_ALL;
    }
    if (s->requested_reset & ESP32S3_SOC_RESET_PERIPH) {
        device_cold_reset(DEVICE(&s->intmatrix));
        for (int i = 0; i < ESP32S3_UART_COUNT; ++i) {
            device_cold_reset(DEVICE(&s->uart[i]));
        }
    }
    if (s->requested_reset & ESP32S3_SOC_RESET_PROCPU) {
        xtensa_select_static_vectors(&s->cpu[0].env, s->rtc_cntl.stat_vector_sel[0]);
        remove_cpu_watchpoints(&s->cpu[0]);
        cpu_reset(CPU(&s->cpu[0]));
        esp32s3_apply_cp_at_reset(s, 0);
    }
    if (s->requested_reset & ESP32S3_SOC_RESET_APPCPU && (ESP32S3_CPU_COUNT > 1)) {
        xtensa_select_static_vectors(&s->cpu[1].env, s->rtc_cntl.stat_vector_sel[1]);
        remove_cpu_watchpoints(&s->cpu[1]);
        cpu_reset(CPU(&s->cpu[1]));
        esp32s3_apply_cp_at_reset(s, 1);
    }
    s->requested_reset = 0;
}

/*
 * Whether a core may run.
 *
 * Core 1 comes up held - in reset, with its clock gated - exactly as the part
 * does, and runs when the firmware says so. It used to run from the moment the
 * machine started, which put it in the ROM's "wait for somewhere to jump" loop
 * before anything had a jump to give it, so it took the first word written to
 * the message register whatever that word was. A firmware that uses that
 * register for a message before it starts a core then sent its second core
 * into a peripheral, and the panic that followed named the firmware.
 */
static void esp32s3_cpu_stall(void* opaque, int n, int level)
{
    /* The RTC's own stall lines are still not acted on. Left alone rather than
     * wired up while passing: nothing here has ever depended on them, and a
     * core that stops for a reason nobody was expecting is a worse fault than
     * one that never stops. */
}

/* The SYSTEM block's hold on core 1: in reset, clock-gated, or stalled. */
static void esp32s3_core1_hold(void* opaque, int n, int level)
{
    Esp32s3SocState *s = ESP32S3_SOC(opaque);

    if (ESP32S3_CPU_COUNT < 2) {
        return;
    }
    s->core1_held = level != 0;
    if (s->core1_held != s->cpu[1].env.runstall) {
        xtensa_runstall(&s->cpu[1].env, s->core1_held);
    }
}

static void esp32s3_clk_update(void* opaque, int n, int level)
{
    if (!level) {
        return;
    }
}

static void esp32s3_soc_add_periph_device(MemoryRegion *dest, void* dev, hwaddr dport_base_addr)
{
    MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 0);
    memory_region_add_subregion_overlap(dest, dport_base_addr, mr, 0);
    MemoryRegion *mr_apb = g_new(MemoryRegion, 1);
    char *name = g_strdup_printf("mr-apb-0x%08x", (uint32_t) dport_base_addr);
    memory_region_init_alias(mr_apb, OBJECT(dev), name, mr, 0, memory_region_size(mr));
    g_free(name);
}

#define MB (1024*1024)

static void esp32s3_init_spi_flash(Esp32s3SocState *ms, BlockBackend* blk)
{
    DeviceState *spi_master = DEVICE(&ms->spi1);
    BusState* spi_bus = qdev_get_child_bus(spi_master, "spi");
    const char* flash_model = NULL;
    int64_t image_size = blk_getlength(blk);

    switch (image_size) {
        case 2 * MB:
            flash_model = "w25x16";
            break;
        case 4 * MB:
            flash_model = "gd25q32";
            break;
        case 8 * MB:
            flash_model = "gd25q64";
            break;
        case 16 * MB:
            flash_model = "is25lp128";
            break;
        default:
            error_report("Drive size error: only 2, 4, 8, and 16MB images are supported");
            return;
    }

    /* Create the SPI flash model */
    DeviceState *flash_dev = qdev_new(flash_model);
    qdev_prop_set_drive(flash_dev, "drive", blk);

    /* Realize the SPI flash, its "drive" (blk) property must already be set! */
    qdev_realize(flash_dev, spi_bus, &error_fatal);
    qdev_connect_gpio_out_named(spi_master, SSI_GPIO_CS, 0,
                                qdev_get_gpio_in_named(flash_dev, SSI_GPIO_CS, 0));
}

static void esp32s3_machine_init_psram(Esp32s3SocState *ms, uint32_t size_mbytes,
                                       bool psram_octal)
{
    /* PSRAM attached to SPI1, CS1 */
    DeviceState *spi_master = DEVICE(&ms->spi1);
    BusState* spi_bus = qdev_get_child_bus(spi_master, "spi");
    DeviceState *psram = qdev_new(TYPE_SSI_PSRAM);
    qdev_prop_set_uint32(psram, "size_mbytes", size_mbytes);
    /* Octal or quad is a property of the part the board carries, and the
     * firmware probes for the one it was built against: an OPI board reads the
     * ID with the octal command and gets zeros from a quad chip, which
     * ESP-IDF reports as "PSRAM chip not found" and then asserts on. */
    qdev_prop_set_bit(psram, "is_octal", psram_octal);
    qdev_prop_set_uint8(psram, "cs", 1);
    qdev_realize(psram, spi_bus, &error_fatal);
    ms->psram = SSI_PSRAM(psram);
    qdev_connect_gpio_out_named(spi_master, SSI_GPIO_CS, 1,
                                qdev_get_gpio_in_named(psram, SSI_GPIO_CS, 0));
}

static void esp32s3_machine_init_sd(Esp32s3SocState* ss)
{
    DriveInfo *dinfo = drive_get(IF_SD, 0, 0);
    if (dinfo) {
        DeviceState *card;

        card = qdev_new(TYPE_SD_CARD);
        qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(dinfo),
                                &error_fatal);
        /* See the comment on not using sysbus-default in esp32_machine_init_i2c */
        DeviceState *sdmmc = DEVICE(&ss->sdmmc);
        SDBus* sd_bus = SD_BUS(qdev_get_child_bus(sdmmc, "sd-bus"));
        qdev_realize_and_unref(card, BUS(sd_bus), &error_fatal);
    }
}

struct Esp32s3MachineState {
    MachineState parent;

    Esp32s3SocState esp32s3;
    DeviceState *flash_dev;

    bool psram_octal;
    /* Whether the coprocessors come up enabled. Off, as the part is. */
    bool cp_at_reset;
    char *radio_path;
    uint32_t radio_spi;
    /* The board's display, where it has one: which controller, on which I2C
     * controller at which address, and the socket its picture leaves by.
     * Absent by default, because a board whose panel nobody has established
     * must show nothing rather than something invented. */
    char *panel_path;
    uint32_t panel_i2c;
    uint32_t panel_addr;
    uint32_t panel_offset;
    /* A colour panel instead, on the SPI controller the radio is on: its own
     * chip select and the command/data line that says what a byte means. */
    uint32_t panel_cs;
    uint32_t panel_dc;
    /* Where button presses arrive from, and which pins they can move. */
    char *input_path;
    char *input_pins;
    /* Which pin selects the board's card slot, where it has one on the same
     * bus as its radio and its display. The card itself arrives as an ordinary
     * drive, so a node's storage is a file the operator can look inside -
     * which a real handheld only offers by taking the card out. */
    uint32_t card_cs;

    /* Which ADC channel the board's battery divider sits on, and what it
     * reads at bring-up. Nothing here invents a voltage: the engine knows the
     * node's cell and says so, and keeps saying so as it drains. */
    uint32_t bat_adc_channel;
    uint32_t bat_adc_raw;

    /* The I2C addresses a keyboard and a touch panel answer on, or zero
     * where the board has neither. */
    uint32_t kbd_addr;
    uint32_t touch_addr;
    uint32_t panel_w;
    uint32_t panel_h;
    uint32_t radio_cs;
    uint32_t radio_nss;
    uint32_t radio_busy;
    uint32_t radio_dio1;
    uint32_t radio_fem;
};

/* No GPIO here, rather than pin 0, which is a real pin and would silently wire
 * the module to whatever drives it. */
#define ESP32S3_RADIO_PIN_NONE UINT32_MAX
#define TYPE_ESP32S3_MACHINE MACHINE_TYPE_NAME("esp32s3")

/* The LoRa radio, when the machine was given a model to talk to.
 *
 * The same shape as the ESP32's, and for the same reason: MeshCore's
 * radio_init() drives an SX1262 over SPI and RadioLib spins waiting for it, so
 * a board with nothing on the bus watchdogs itself instead of booting. The chip
 * is modelled in MeshBench; this only has to put a peripheral on the right bus
 * and chip select and hand it the socket.
 *
 * Default index 2 is GPSPI2 - FSPI on this part, and what Arduino's SPI object
 * drives. Boards that route the radio elsewhere override it.
 */
static void esp32s3_machine_init_radio(Esp32s3SocState *ss, const char *path,
                                       unsigned spi_index, unsigned cs,
                                       unsigned nss_pin, unsigned busy_pin,
                                       unsigned dio1_pin, unsigned fem_pin)
{
    if (spi_index != 2 && spi_index != 3) {
        error_report("radio-spi=%u: this machine has GPSPI2 and GPSPI3",
                     spi_index);
        return;
    }

    DeviceState *spi_master = spi_index == 3
        ? DEVICE(&ss->gpspi3) : DEVICE(&ss->gpspi2);
    BusState *spi_bus = qdev_get_child_bus(spi_master, "spi");
    DeviceState *radio = qdev_new("sx1262");

    qdev_prop_set_string(radio, "path", path);
    qdev_prop_set_uint8(radio, "cs", cs);
    qdev_realize_and_unref(radio, spi_bus, &error_fatal);

    /* NSS and BUSY are ordinary GPIOs on these boards, not the controller's own
     * chip select. NSS is what gives the device its transaction boundaries:
     * RadioLib holds it low across a multi-byte command while the controller
     * clocks the bytes out one transfer at a time. Without it the device sees an
     * unframed byte stream and cannot answer a register read. */
    DeviceState *gpio = DEVICE(&ss->gpio);

    qdev_connect_gpio_out_named(gpio, ESP32_GPIO_OUT, nss_pin,
                                qdev_get_gpio_in_named(radio, "sx1262-nss", 0));
    qdev_connect_gpio_out_named(radio, "sx1262-busy", 0,
                                qdev_get_gpio_in_named(gpio, ESP32_GPIO_IN,
                                                       busy_pin));

    /* DIO1, the packet-received interrupt. MeshCore reads a packet only from
     * the ISR this line fires, so a board left without it receives correctly
     * and never collects anything. Left unwired when no pin is given, which
     * is what a board that genuinely has no such line should look like. */
    if (dio1_pin != ESP32S3_RADIO_PIN_NONE) {
        qdev_connect_gpio_out_named(radio, "sx1262-dio1", 0,
                                    qdev_get_gpio_in_named(gpio, ESP32_GPIO_IN,
                                                           dio1_pin));
    }

    /* The front-end module's transmit enable, on boards that have one. Left
     * unwired by default because most do not, and a board with no module must
     * not be modelled as one whose module is permanently off. */
    if (fem_pin != ESP32S3_RADIO_PIN_NONE) {
        qdev_connect_gpio_out_named(gpio, ESP32_GPIO_OUT, fem_pin,
                                    qdev_get_gpio_in_named(radio,
                                                           "sx1262-fem", 0));
    }
}

static void esp32s3_init_openeth(Esp32s3SocState *ms)
{
    MemoryRegion* mr = NULL;
    SysBusDevice* sbd = NULL;

    MemoryRegion* sys_mem = get_system_memory();

    /* Create a new OpenCores Ethernet component */
    DeviceState* open_eth_dev = qemu_create_nic_device("open_eth", true, NULL);
    if (!open_eth_dev) {
        return;
    }
    ms->eth = open_eth_dev;
    sbd = SYS_BUS_DEVICE(open_eth_dev);
    sysbus_realize(sbd, &error_fatal);

    /* OpenCores Ethernet has two memory regions: one for registers and one for descriptors,
        * we need to provide one I/O range for each of them */
    mr = sysbus_mmio_get_region(sbd, 0);
    memory_region_add_subregion_overlap(sys_mem, DR_REG_EMAC_BASE, mr, 0);
    mr = sysbus_mmio_get_region(sbd, 1);
    memory_region_add_subregion_overlap(sys_mem, DR_REG_EMAC_BASE + 0x400, mr, 0);

    sysbus_connect_irq(sbd, 0,
                        qdev_get_gpio_in(DEVICE(&ms->intmatrix), ETS_ETH_MAC_INTR_SOURCE));
}


static void esp32s3_soc_realize(DeviceState *dev, Error **errp)
{
    Esp32s3SocState *s = ESP32S3_SOC(dev);
    MachineState *ms = MACHINE(qdev_get_machine());
    DeviceState* intmatrix_dev = DEVICE(&s->intmatrix);
    MemoryRegion *sys_mem = get_system_memory();

    const struct MemmapEntry *memmap = esp32s3_memmap;

    MemoryRegion *iram = g_new(MemoryRegion, 1);
    MemoryRegion *rtcslow = g_new(MemoryRegion, 1);
    MemoryRegion *rtcfast = g_new(MemoryRegion, 1);

    for (int i = 0; i < ms->smp.cpus; ++i) {
        MemoryRegion *drom = g_new(MemoryRegion, 1);
        MemoryRegion *irom = g_new(MemoryRegion, 1);

        char name[20];
        snprintf(name, sizeof(name), "esp32s3.irom.cpu%d", i);
        memory_region_init_rom(irom, NULL, name, memmap[ESP32S3_MEMREGION_IROM].size, &error_fatal);
        memory_region_add_subregion(&s->cpu_specific_mem[i], memmap[ESP32S3_MEMREGION_IROM].base, irom);

        const hwaddr offset_in_orig = 0x40000;
        snprintf(name, sizeof(name), "esp32s3.drom.cpu%d", i);
        memory_region_init_alias(drom, NULL, name, irom, offset_in_orig, memmap[ESP32S3_MEMREGION_DROM].size);
        memory_region_add_subregion(sys_mem, memmap[ESP32S3_MEMREGION_DROM].base, drom);
    }


    memory_region_init_ram(iram, NULL, "esp32s3.iram",
                           memmap[ESP32S3_MEMREGION_IRAM].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32S3_MEMREGION_IRAM].base, iram);

    memory_region_init_ram(rtcslow, NULL, "esp32s3.rtcslow",
                           memmap[ESP32S3_MEMREGION_RTCSLOW].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32S3_MEMREGION_RTCSLOW].base, rtcslow);

    memory_region_init_ram(rtcfast, NULL, "esp32s3.rtcfast",
                           memmap[ESP32S3_MEMREGION_RTCFAST].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32S3_MEMREGION_RTCFAST].base, rtcfast);

    for (int i = 0; i < ms->smp.cpus; ++i) {
        qdev_realize(DEVICE(&s->cpu[i]), NULL, &error_fatal);
    }


    for (int i = 0; i < ESP32S3_CPU_COUNT; ++i) {
        char name[16];
        snprintf(name, sizeof(name), "cpu%d", i);
        object_property_set_link(OBJECT(&s->intmatrix), name, OBJECT(qemu_get_cpu(i)), &error_abort);
    }
    qdev_realize(DEVICE(&s->intmatrix), &s->periph_bus, &error_fatal);


    qdev_realize(DEVICE(&s->rtc_cntl), &s->rtc_bus, &error_fatal);
    esp32s3_soc_add_periph_device(sys_mem, &s->rtc_cntl, DR_REG_RTCCNTL_BASE);

    qdev_connect_gpio_out_named(DEVICE(&s->rtc_cntl), ESP32S3_RTC_DIG_RESET_GPIO, 0,
                                qdev_get_gpio_in_named(dev, ESP32S3_RTC_DIG_RESET_GPIO, 0));
    qdev_connect_gpio_out_named(DEVICE(&s->rtc_cntl), ESP32S3_RTC_CLK_UPDATE_GPIO, 0,
                                qdev_get_gpio_in_named(dev, ESP32S3_RTC_CLK_UPDATE_GPIO, 0));
    for (int i = 0; i < ms->smp.cpus; ++i) {
        qdev_connect_gpio_out_named(DEVICE(&s->rtc_cntl), ESP32S3_RTC_CPU_RESET_GPIO, i,
                                    qdev_get_gpio_in_named(dev, ESP32S3_RTC_CPU_RESET_GPIO, i));
        qdev_connect_gpio_out_named(DEVICE(&s->rtc_cntl), ESP32S3_RTC_CPU_STALL_GPIO, i,
                                    qdev_get_gpio_in_named(dev, ESP32S3_RTC_CPU_STALL_GPIO, i));
    }

    for (int i = 0; i < ESP32S3_UART_COUNT; ++i) {
        const hwaddr uart_base[] = {DR_REG_UART_BASE, DR_REG_UART1_BASE, DR_REG_UART2_BASE};
        qdev_realize(DEVICE(&s->uart[i]), &s->periph_bus, &error_fatal);
        esp32s3_soc_add_periph_device(sys_mem, &s->uart[i], uart_base[i]);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->uart[i]), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_UART0_INTR_SOURCE + i));
    }

    qdev_realize(DEVICE(&s->sdmmc), &s->periph_bus, &error_fatal);
    esp32s3_soc_add_periph_device(sys_mem, &s->sdmmc, DR_REG_SDMMC_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->sdmmc), 0,
                       qdev_get_gpio_in(intmatrix_dev, ETS_SDIO_HOST_INTR_SOURCE));

    /* Emulation of APB_CTRL_DATE_REG, needed for ECO3 revision detection.
     * This is a small hack to avoid creating a whole new device just to emulate one
     * register.
     */
    const hwaddr apb_ctrl_regs = DR_REG_APB_CTRL_BASE;
    MemoryRegion *apbctrl_mem = g_new(MemoryRegion, 1);
    memory_region_init_ram(apbctrl_mem, NULL, "esp32s3.apbctrl", 0x400 /* bytes */, &error_fatal);
    memory_region_add_subregion(sys_mem, apb_ctrl_regs, apbctrl_mem);
    uint32_t apb_ctrl_date_reg_val = 0x16042000 | 0x80000000;  /* MSB indicates ECO3 silicon revision */
    uint32_t qemu_sig = RGB_QEMU_ORIGIN;
    cpu_physical_memory_write(apb_ctrl_regs + 0x7c, &apb_ctrl_date_reg_val, 4);
    cpu_physical_memory_write(apb_ctrl_regs + RGB_QEMU_ORIGIN_REG, &qemu_sig, 4);

    qemu_register_reset((QEMUResetHandler*) esp32s3_soc_reset, dev);

    /* TWAI realization */
    {
        /* Initialize and realize the TWAI device */
        sysbus_realize(SYS_BUS_DEVICE(&s->twai), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->twai), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_TWAI_BASE, mr, 0);
        /* Connect TWAI interrupt to the interrupt matrix */
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->twai), 0,
                          qdev_get_gpio_in(intmatrix_dev, ETS_TWAI_INTR_SOURCE));
    }
}


static uint64_t esp32s3_io_read(void *opaque, hwaddr addr, unsigned int size)
{
    /*
     * Everything in the peripheral window that no device claimed lands here.
     * Say so, rather than answering zero in silence: a register that reads as
     * zero is a peripheral reporting something, and firmware that acts on it
     * looks broken while the emulator looks fine. Six and a half thousand of
     * these went unremarked in fifteen seconds of one boot.
     */
    qemu_log_mask(LOG_UNIMP, "esp32s3: unimplemented read at 0x%08lx, size %u\n",
                  (unsigned long)(ESP32S3_IO_START_ADDR + addr), size);
    return 0;
}


static void esp32s3_io_write(void *opaque, hwaddr addr, uint64_t value, unsigned int size)
{
    qemu_log_mask(LOG_UNIMP,
                  "esp32s3: unimplemented write at 0x%08lx = 0x%08lx, size %u\n",
                  (unsigned long)(ESP32S3_IO_START_ADDR + addr),
                  (unsigned long)value, size);
}


/* Define operations for I/OS */
static const MemoryRegionOps esp32s3_io_ops = {
    .read =  esp32s3_io_read,
    .write = esp32s3_io_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};



static void esp32s3_soc_init(Object *obj)
{
    Esp32s3SocState *s = ESP32S3_SOC(obj);
    MachineState *ms = MACHINE(qdev_get_machine());
    char name[16];
    MemoryRegion *system_memory = get_system_memory();


    qbus_init(&s->periph_bus, sizeof(s->periph_bus),
                        TYPE_SYSTEM_BUS, DEVICE(s), "esp32-periph-bus");
    qbus_init(&s->rtc_bus, sizeof(s->rtc_bus),
                        TYPE_SYSTEM_BUS, DEVICE(s), "esp32-rtc-bus");

    for (int i = 0; i < ms->smp.cpus; ++i) {
        snprintf(name, sizeof(name), "cpu%d", i);

        object_initialize_child(obj, name, &s->cpu[i], TYPE_ESP32S3_CPU);
        // Allocate memory for TIE registers
        s->cpu[i].env.ext = qemu_memalign(16, sizeof(CPUXtensaEsp32s3State));

        if (i == 0)
        {
            s->cpu[i].env.sregs[PRID] = 0xcdcd;
        }
        if (i == 1)
        {
            s->cpu[i].env.sregs[PRID] = 0xabab;
        }

        snprintf(name, sizeof(name), "cpu%d-mem", i);
        memory_region_init(&s->cpu_specific_mem[i], NULL, name, UINT32_MAX);

        CPUState* cs = CPU(&s->cpu[i]);
        cs->num_ases = 1;
        cpu_address_space_init(cs, 0, "cpu-memory", &s->cpu_specific_mem[i]);

        MemoryRegion *cpu_view_sysmem = g_new(MemoryRegion, 1);
        snprintf(name, sizeof(name), "cpu%d-sysmem", i);
        memory_region_init_alias(cpu_view_sysmem, NULL, name, system_memory, 0, UINT32_MAX);
        memory_region_add_subregion_overlap(&s->cpu_specific_mem[i], 0, cpu_view_sysmem, 0);
        cs->memory = &s->cpu_specific_mem[i];
    }

    for (int i = 0; i < ESP32S3_UART_COUNT; ++i) {
        snprintf(name, sizeof(name), "uart%d", i);
        object_initialize_child(obj, name, &s->uart[i], TYPE_ESP32S3_UART);
    }

    object_property_add_alias(obj, "serial0", OBJECT(&s->uart[0]), "chardev");
    object_property_add_alias(obj, "serial1", OBJECT(&s->uart[1]), "chardev");
    // object_property_add_alias(obj, "serial2", OBJECT(&s->uart[2]), "chardev");
    qdev_prop_set_chr(DEVICE(&s->uart[0]), "chardev", serial_hd(0));
    qdev_prop_set_chr(DEVICE(&s->uart[1]), "chardev", serial_hd(1));
    // qdev_prop_set_chr(DEVICE(&s->uart[2]), "chardev", serial_hd(2));

    object_initialize_child(obj, "intmatrix", &s->intmatrix, TYPE_ESP32S3_INTMATRIX);

    object_initialize_child(obj, "rtc_cntl", &s->rtc_cntl, TYPE_ESP32S3_RTC_CNTL);

    qdev_init_gpio_in_named(DEVICE(s), esp32s3_dig_reset,  ESP32S3_RTC_DIG_RESET_GPIO, 1);
    qdev_init_gpio_in_named(DEVICE(s), esp32s3_cpu_reset,  ESP32S3_RTC_CPU_RESET_GPIO, ESP32S3_CPU_COUNT);
    qdev_init_gpio_in_named(DEVICE(s), esp32s3_cpu_stall,  ESP32S3_RTC_CPU_STALL_GPIO, ESP32S3_CPU_COUNT);
    qdev_init_gpio_in_named(DEVICE(s), esp32s3_core1_hold,
                            ESP32S3_CLOCK_CORE1_STALL_GPIO, 1);
    qdev_init_gpio_in_named(DEVICE(s), esp32s3_clk_update, ESP32S3_RTC_CLK_UPDATE_GPIO, 1);

    object_initialize_child(obj, "twai", &s->twai, TYPE_ESP32S3_TWAI);

    object_initialize_child(obj, "sdmmc", &s->sdmmc, TYPE_DWC_SDMMC);
}

static Property esp32s3_soc_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32s3_soc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = esp32s3_soc_realize;
    device_class_set_props(dc, esp32s3_soc_properties);
}

static const TypeInfo esp32s3_soc_info = {
    .name = TYPE_ESP32S3_SOC,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(Esp32s3SocState),
    .instance_init = esp32s3_soc_init,
    .class_init = esp32s3_soc_class_init
};

static void esp32s3_soc_register_types(void)
{
    type_register_static(&esp32s3_soc_info);
}

type_init(esp32s3_soc_register_types)


static uint64_t translate_phys_addr(void *opaque, uint64_t addr)
{
    XtensaCPU *cpu = opaque;

    return cpu_get_phys_page_debug(CPU(cpu), addr);
}

OBJECT_DECLARE_SIMPLE_TYPE(Esp32s3MachineState, ESP32S3_MACHINE)

// -----------------------------------------------

static void esp32s3_soc_add_unimp_device(MemoryRegion *dest, const char* name, hwaddr dport_base_addr, size_t size)
{
    create_unimplemented_device(name, dport_base_addr, size);
    char * name_apb = g_strdup_printf("%s-apb", name);
    create_unimplemented_device(name_apb, dport_base_addr + APB_REG_BASE, size);
    g_free(name_apb);
}

static void esp32s3_machine_init(MachineState *machine)
{
    DriveInfo *dinfo = drive_get(IF_MTD, 0, 0);
    BlockBackend* blk = NULL;
    if (dinfo) {
        /* MTD was given! We need to initialize and emulate SPI flash */
        qemu_log("Adding SPI flash device\n");
        blk = blk_by_legacy_dinfo(dinfo);
    } else {
        qemu_log("Not initializing SPI Flash\n");
    }

    MemoryRegion *sys_mem = get_system_memory();
    Esp32s3MachineState *ms = ESP32S3_MACHINE(machine);
    object_initialize_child(OBJECT(ms), "soc", &ms->esp32s3, TYPE_ESP32S3_SOC);
    Esp32s3SocState *ss = ESP32S3_SOC(&ms->esp32s3);

    MemoryRegion *dram = g_new(MemoryRegion, 1);
    const struct MemmapEntry *memmap = esp32s3_memmap;

    memory_region_init_ram(dram, NULL, "esp32s3.dram",
                           memmap[ESP32S3_MEMREGION_DRAM].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32S3_MEMREGION_DRAM].base, dram);


    memory_region_init_io(&ss->iomem, OBJECT(&ss->cpu[0]), &esp32s3_io_ops,
                          NULL, "esp32s3.iomem", 0xd1000);
    memory_region_add_subregion(sys_mem, ESP32S3_IO_START_ADDR, &ss->iomem);

    // qdev_prop_set_chr(DEVICE(ss), "serial0", serial_hd(0));
    // qdev_prop_set_chr(DEVICE(ss), "serial1", serial_hd(1));
    // qdev_prop_set_chr(DEVICE(ss), "serial2", serial_hd(2));

    qdev_realize(DEVICE(ss), NULL, &error_fatal);

    object_initialize_child(OBJECT(ss), "extmem", &ss->cache, TYPE_ESP32S3_CACHE);
    object_initialize_child(OBJECT(ss), "spi1", &ss->spi1, TYPE_ESP32S3_SPI);
    for (int i = 0; i < ESP32S3_I2C_COUNT; ++i) {
        char name[8];
        snprintf(name, sizeof(name), "i2c%d", i);
        object_initialize_child(OBJECT(ss), name, &ss->i2c[i], TYPE_ESP32_I2C);
        /* This part numbers three of the I2C commands differently from the
         * ESP32 the model was written for. Same registers, different
         * encoding - and reading a list with the wrong table does not fail,
         * it runs the wrong commands. */
        qdev_prop_set_bit(DEVICE(&ss->i2c[i]), "newer-opcodes", true);
    }
    object_initialize_child(OBJECT(ss), "gpspi2", &ss->gpspi2, TYPE_ESP32S3_GPSPI);
    object_initialize_child(OBJECT(ss), "gpspi3", &ss->gpspi3, TYPE_ESP32S3_GPSPI);
    object_initialize_child(OBJECT(ss), "efuse", &ss->efuse, TYPE_ESP32S3_EFUSE);
    object_initialize_child(OBJECT(ss), "jtag", &ss->jtag,
                            TYPE_ESP32S3_USB_SERIAL_JTAG);
    object_initialize_child(OBJECT(ss), "gpio", &ss->gpio, TYPE_ESP32S3_GPIO);
    object_initialize_child(OBJECT(ss), "rng", &ss->rng, TYPE_ESP32S3_RNG);

    object_initialize_child(OBJECT(ss), "clock", &ss->clock, TYPE_ESP32S3_CLOCK);

    object_initialize_child(OBJECT(ss), "gdma", &ss->gdma, TYPE_ESP32S3_GDMA);
    object_initialize_child(OBJECT(ss), "sha", &ss->sha, TYPE_ESP32S3_SHA);
    object_initialize_child(OBJECT(ss), "aes", &ss->aes, TYPE_ESP32S3_AES);
    object_initialize_child(OBJECT(ss), "rsa", &ss->rsa, TYPE_ESP32S3_RSA);
    object_initialize_child(OBJECT(ss), "hmac", &ss->hmac, TYPE_ESP32S3_HMAC);
    object_initialize_child(OBJECT(ss), "ds", &ss->ds, TYPE_ESP32S3_DS);
    object_initialize_child(OBJECT(ss), "pms", &ss->pms, TYPE_ESP32S3_PMS);

    object_initialize_child(OBJECT(ss), "xts_aes", &ss->xts_aes, TYPE_ESP32S3_XTS_AES);
    object_initialize_child(OBJECT(ss), "timg0", &ss->timg[0], TYPE_ESP32S3_TIMG);
    object_initialize_child(OBJECT(ss), "timg1", &ss->timg[1], TYPE_ESP32S3_TIMG);
    object_initialize_child(OBJECT(ss), "systimer", &ss->systimer, TYPE_ESP32S3_SYSTIMER);
    object_initialize_child(OBJECT(ss), "rgb", &ss->rgb, TYPE_ESP_RGB);

    DeviceState* intmatrix_dev = DEVICE(&ss->intmatrix);
    {
        /* Store the current Machine CPU in the interrupt matrix */
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->intmatrix), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_INTERRUPT_BASE, mr, 0);
    }

    /* Initialize OpenCores Ethernet controller now sicne it requires the interrupt matrix */
    esp32s3_init_openeth(ss);

    /*
     * USB Serial/JTAG. A character device rather than the register stub it
     * was: a board built with ARDUINO_USB_CDC_ON_BOOT has Serial here and not
     * on UART0, so on those boards the application's whole console went into
     * the stub and came out nowhere.
     *
     * Its backend is the third serial port, so a machine started with
     * -serial chardev:con -serial chardev:gps -serial chardev:usb reaches it
     * without a new machine property.
     */
    {
        qdev_prop_set_chr(DEVICE(&ss->jtag), "chardev", serial_hd(2));
        sysbus_realize(SYS_BUS_DEVICE(&ss->jtag), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->jtag), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_USB_SERIAL_JTAG_BASE, mr, 0);
        sysbus_connect_irq(SYS_BUS_DEVICE(&ss->jtag), 0,
                           qdev_get_gpio_in(intmatrix_dev,
                                            ETS_USB_SERIAL_JTAG_INTR_SOURCE));
    }

    /* SPI1 controller (SPI Flash) */
    {
        ss->spi1.xts_aes = &ss->xts_aes;
        sysbus_realize(SYS_BUS_DEVICE(&ss->spi1), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->spi1), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_SPI1_BASE, mr, 0);
        if (blk) {
            esp32s3_init_spi_flash(ss, blk);
        }
        if (machine->ram_size > 0) {
            esp32s3_machine_init_psram(ss, (uint32_t) (machine->ram_size / MiB),
                                       ESP32S3_MACHINE(OBJECT(machine))->psram_octal);
        }
        ss->cp_at_reset = ESP32S3_MACHINE(OBJECT(machine))->cp_at_reset;
    }

    /* GPSPI2, the general-purpose controller a board puts a radio on. */
    {
        for (int i = 0; i < ESP32S3_I2C_COUNT; ++i) {
            const hwaddr i2c_base[] = {DR_REG_I2C_EXT_BASE, DR_REG_I2C_EXT_BASE + 0x11000};
            qdev_realize(DEVICE(&ss->i2c[i]), &ss->periph_bus, &error_fatal);
            esp32s3_soc_add_periph_device(sys_mem, &ss->i2c[i], i2c_base[i]);
            sysbus_connect_irq(SYS_BUS_DEVICE(&ss->i2c[i]), 0,
                               qdev_get_gpio_in(intmatrix_dev,
                                                ETS_I2C_EXT0_INTR_SOURCE + i));
        }

        sysbus_realize(SYS_BUS_DEVICE(&ss->gpspi2), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->gpspi2), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_SPI2_BASE, mr, 0);

        sysbus_connect_irq(SYS_BUS_DEVICE(&ss->gpspi2), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_SPI2_INTR_SOURCE));

        sysbus_realize(SYS_BUS_DEVICE(&ss->gpspi3), &error_fatal);
        MemoryRegion *mr3 = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->gpspi3), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_SPI3_BASE, mr3, 0);
        sysbus_connect_irq(SYS_BUS_DEVICE(&ss->gpspi3), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_SPI3_INTR_SOURCE));
    }

    /* (Extmem) Cache realization */
    {
        if (blk) {
            ss->cache.flash_blk = blk;
        }
        if (ss->psram) {
            ss->cache.psram = ss->psram;
        }
        ss->cache.xts_aes = &ss->xts_aes;
        sysbus_realize(SYS_BUS_DEVICE(&ss->cache), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->cache), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_EXTMEM_BASE, mr, 0);

        memory_region_add_subregion(sys_mem, memmap[ESP32S3_MEMREGION_DCACHE].base, &ss->cache.dcache);
        memory_region_add_subregion(sys_mem, memmap[ESP32S3_MEMREGION_ICACHE].base, &ss->cache.icache);
    }

    /* eFuses realization */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ss->efuse), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->efuse), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_EFUSE_BASE, mr, 0);
        sysbus_connect_irq(SYS_BUS_DEVICE(&ss->efuse), 0,
                       qdev_get_gpio_in(intmatrix_dev, ETS_EFUSE_INTR_SOURCE));
    }

    /* System clock realization */
    {
        qdev_connect_gpio_out_named(DEVICE(&ss->clock),
                                    ESP32S3_CLOCK_CORE1_STALL_GPIO, 0,
                                    qdev_get_gpio_in_named(DEVICE(ss),
                                        ESP32S3_CLOCK_CORE1_STALL_GPIO, 0));
        sysbus_realize(SYS_BUS_DEVICE(&ss->clock), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->clock), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_SYSTEM_BASE, mr, 0);
        /* Connect the IRQ lines to the interrupt matrix */
        for (int i = 0; i < ESP32S3_SYSTEM_CPU_INTR_COUNT; i++) {
            sysbus_connect_irq(SYS_BUS_DEVICE(&ss->clock), i,
                           qdev_get_gpio_in(intmatrix_dev, ETS_FROM_CPU_INTR0_SOURCE + i));
        }
    }
    /* Timer Groups realization */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ss->timg[0]), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->timg[0]), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_TIMERGROUP0_BASE, mr, 0);
        /* Connect the T0 interrupt line to the interrupt matrix */
        qdev_connect_gpio_out_named(DEVICE(&ss->timg[0]), ESP32S3_T0_IRQ_INTERRUPT, 0,
                                    qdev_get_gpio_in(intmatrix_dev, ETS_TG0_T0_LEVEL_INTR_SOURCE));
        /* Connect the T1 interrupt line to the interrupt matrix */
        qdev_connect_gpio_out_named(DEVICE(&ss->timg[0]), ESP32S3_T1_IRQ_INTERRUPT, 0,
                                    qdev_get_gpio_in(intmatrix_dev, ETS_TG0_T1_LEVEL_INTR_SOURCE));
        /* Connect the Watchdog interrupt line to the interrupt matrix */
        qdev_connect_gpio_out_named(DEVICE(&ss->timg[0]), ESP32S3_WDT_IRQ_INTERRUPT, 0,
                                    qdev_get_gpio_in(intmatrix_dev, ETS_TG0_WDT_LEVEL_INTR_SOURCE));
    }
    {
        sysbus_realize(SYS_BUS_DEVICE(&ss->timg[1]), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->timg[1]), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_TIMERGROUP1_BASE, mr, 0);
        /* Connect the T0 interrupt line to the interrupt matrix */
        qdev_connect_gpio_out_named(DEVICE(&ss->timg[1]), ESP32S3_T0_IRQ_INTERRUPT, 0,
                                    qdev_get_gpio_in(intmatrix_dev, ETS_TG1_T0_LEVEL_INTR_SOURCE));
        /* Connect the T1 interrupt line to the interrupt matrix */
        qdev_connect_gpio_out_named(DEVICE(&ss->timg[1]), ESP32S3_T1_IRQ_INTERRUPT, 0,
                                    qdev_get_gpio_in(intmatrix_dev, ETS_TG1_T1_LEVEL_INTR_SOURCE));
        /* Connect the Watchdog interrupt line to the interrupt matrix */
        qdev_connect_gpio_out_named(DEVICE(&ss->timg[1]), ESP32S3_WDT_IRQ_INTERRUPT, 0,
                                    qdev_get_gpio_in(intmatrix_dev, ETS_TG1_WDT_LEVEL_INTR_SOURCE));
    }

    /* System timer */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ss->systimer), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->systimer), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_SYSTIMER_BASE, mr, 0);
        for (int i = 0; i < ESP_SYSTIMER_IRQ_COUNT; i++) {
            sysbus_connect_irq(SYS_BUS_DEVICE(&ss->systimer), i,
                           qdev_get_gpio_in(intmatrix_dev, ETS_SYSTIMER_TARGET0_EDGE_INTR_SOURCE + i));
        }
    }


    /* GPIO realization */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ss->gpio), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->gpio), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_GPIO_BASE, mr, 0);
    }

    {
        qdev_realize(DEVICE(&ss->rng), &ss->periph_bus, &error_fatal);
        esp32s3_soc_add_periph_device(sys_mem, &ss->rng, ESP32S3_RNG_BASE);

    }


    /* GDMA Realization */
    {
        object_property_set_link(OBJECT(&ss->gdma), "soc_mr", OBJECT(dram), &error_abort);
        sysbus_realize(SYS_BUS_DEVICE(&ss->gdma), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->gdma), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_GDMA_BASE, mr, 0);
        /* Connect the IRQs to the Interrupt Matrix */
        for (int i = 0; i < ESP32S3_GDMA_CHANNEL_COUNT; i++) {
            qdev_connect_gpio_out_named(DEVICE(&ss->gdma), ESP_GDMA_IRQ_IN_NAME, i,
                                        qdev_get_gpio_in(intmatrix_dev, ETS_DMA_IN_CH0_INTR_SOURCE + i));
            qdev_connect_gpio_out_named(DEVICE(&ss->gdma), ESP_GDMA_IRQ_OUT_NAME, i,
                                        qdev_get_gpio_in(intmatrix_dev, ETS_DMA_OUT_CH0_INTR_SOURCE + i));
        }
   }

    /* SHA realization */
    {
        ss->sha.parent.gdma = ESP_GDMA(&ss->gdma);
        sysbus_realize(SYS_BUS_DEVICE(&ss->sha), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->sha), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_SHA_BASE, mr, 0);
        sysbus_connect_irq(SYS_BUS_DEVICE(&ss->sha), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_SHA_INTR_SOURCE));
    }

    /* AES realization */
    {
        ss->aes.parent.gdma = ESP_GDMA(&ss->gdma);
        sysbus_realize(SYS_BUS_DEVICE(&ss->aes), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->aes), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_AES_BASE, mr, 0);
        sysbus_connect_irq(SYS_BUS_DEVICE(&ss->aes), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_AES_INTR_SOURCE));
    }
    /* RSA realization */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ss->rsa), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->rsa), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_RSA_BASE, mr, 0);
        sysbus_connect_irq(SYS_BUS_DEVICE(&ss->rsa), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_RSA_INTR_SOURCE));
    }
    /* PMS realization */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ss->pms), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->pms), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_SENSITIVE_BASE, mr, 0);
    }

    /* HMAC realization */
    {
        ss->hmac.parent.efuse = ESP_EFUSE(&ss->efuse);
        qdev_realize(DEVICE(&ss->hmac), &ss->periph_bus, &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->hmac), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_HMAC_BASE, mr, 0);
    }


    /* Digital Signature realization */
    {
        ss->ds.parent.hmac = ESP_HMAC(&ss->hmac);
        ss->ds.parent.aes = ESP_AES(&ss->aes);
        ss->ds.parent.rsa = ESP_RSA(&ss->rsa);
        ss->ds.parent.sha = ESP_SHA(&ss->sha);
        qdev_realize(DEVICE(&ss->ds), &ss->periph_bus, &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->ds), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_DIGITAL_SIGNATURE_BASE, mr, 0);
    }
    /* XTS-AES realization */
    {
        ss->xts_aes.efuse = ESP_EFUSE(&ss->efuse);
        ss->xts_aes.clock = &ss->clock;
        qdev_realize(DEVICE(&ss->xts_aes), &ss->periph_bus, &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->xts_aes), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_AES_XTS_BASE, mr, 0);
    }

    /* RGB display realization */
    {
        /* Give the internal RAM memory region to the display */
        ss->rgb.intram = dram;
        sysbus_realize(SYS_BUS_DEVICE(&ss->rgb), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->rgb), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_FRAMEBUF_BASE, mr, 0);
        memory_region_add_subregion_overlap(sys_mem, esp32s3_memmap[ESP32S3_MEMREGION_FRAMEBUF].base, &ss->rgb.vram, 0);
    }

    /* The Bluetooth controller's exchange memory.
     *
     * A block of SRAM below the data RAM this machine already has, which the
     * controller's ROM copies its radio configuration into before it starts.
     * Absent, that copy is a load from nowhere: measured as a LoadStoreError
     * at 0x3FC00100 inside r_rw_rf_init, which panics and reboots - so a board
     * whose firmware merely offers Bluetooth could not finish booting. The
     * memory is real on the part; only its contents mean nothing here,
     * because nothing is on the air. */
    {
        MemoryRegion *btem = g_new(MemoryRegion, 1);
        memory_region_init_ram(btem, NULL, "esp32s3.btem", 0x80000, &error_fatal);
        memory_region_add_subregion(sys_mem, 0x3FC00000, btem);
    }

    esp32s3_soc_add_unimp_device(sys_mem, "esp32s3.rmt", DR_REG_RMT_BASE, 0x1000);
    esp32s3_soc_add_unimp_device(sys_mem, "esp32s3.iomux", DR_REG_IO_MUX_BASE, 0x2000);

    

    {
        Esp32s3MachineState *mach = ESP32S3_MACHINE(OBJECT(machine));
        /* The peripherals below hang off the controller the board names; this
         * points the other one at the same bus, because on these boards there
         * is one bus. The radio, the display and the card slot share the same
         * pins in copper and the GPIO matrix routes whichever controller the
         * firmware picked onto them - so a board answered only the firmware
         * that happened to pick the same controller as the image its wiring
         * was verified against. */
        if (mach->radio_spi == 3) {
            esp32s3_gpspi_share_bus(&ss->gpspi2, &ss->gpspi3);
        } else {
            esp32s3_gpspi_share_bus(&ss->gpspi3, &ss->gpspi2);
        }
        if (mach->radio_path) {
            esp32s3_machine_init_radio(ss, mach->radio_path, mach->radio_spi,
                                       mach->radio_cs, mach->radio_nss,
                                       mach->radio_busy, mach->radio_dio1,
                                       mach->radio_fem);
        }
        /* The card. One drive, put wherever the board's slot actually is:
         * on these handhelds it is a third device on the bus the radio and
         * the display already share, told apart by its own select, and only
         * the controller-attached kind where a board has that instead. */
        if (mach->card_cs != ESP32S3_RADIO_PIN_NONE) {
            DeviceState *master = mach->radio_spi == 3
                ? DEVICE(&ss->gpspi3) : DEVICE(&ss->gpspi2);
            BusState *bus = qdev_get_child_bus(master, "spi");
            DeviceState *slot = qdev_new("ssi-sd");
            /* An index of its own, as the display has: the bus wants them
             * distinct even where the select is a GPIO rather than the
             * controller's. */
            qdev_prop_set_uint8(slot, "cs", mach->radio_cs == 0 ? 2 : 3);
            qdev_realize_and_unref(slot, bus, &error_fatal);
            qemu_irq cs = qdev_get_gpio_in_named(slot, SSI_GPIO_CS, 0);
            qdev_connect_gpio_out_named(DEVICE(&ss->gpio), ESP32_GPIO_OUT,
                                        mach->card_cs, cs);
            /* Deselected until somebody selects it, because the pin has a
             * pull-up on the board and the controller here starts its outputs
             * low. A card left selected answers every byte on a bus it shares
             * with the radio and the display, and what came back was the two
             * replies merged - a firmware that reads the radio's version
             * string gets it with the card's answer laid over the top and
             * reports no chip fitted. Which is what happened, on a build that
             * never touches the card at all. */
            qemu_set_irq(cs, 1);
            DriveInfo *dinfo = drive_get(IF_SD, 0, 0);
            if (dinfo) {
                DeviceState *card = qdev_new(TYPE_SD_CARD_SPI);
                qdev_prop_set_drive_err(card, "drive",
                                        blk_by_legacy_dinfo(dinfo), &error_fatal);
                qdev_realize_and_unref(card,
                    qdev_get_child_bus(slot, "sd-bus"), &error_fatal);
            }
        } else {
            esp32s3_machine_init_sd(ss);
        }

        /* The radio front end and its analog bus. Stubbed rather than
         * modelled - this machine has one radio and it is the LoRa
         * transceiver - but answered rather than absent, because the PHY
         * waits on these and an unanswered wait never ends. See the device. */
        {
            static const struct { uint32_t base, size; } rf[] = {
                { 0x60006000, 0x1000 },  /* the front end */
                { 0x6000E000, 0x0200 },  /* the analog bus, below I2S */
                { 0x60033000, 0x1000 },  /* the MAC the PHY hands over to */
                { 0x60031000, 0x1000 },  /* the Bluetooth controller */
                { 0x60032000, 0x1000 },  /* and the rest of its block */
                { 0x60019000, 0x1000 },  /* LEDC, which drives the backlight */
            };
            for (size_t i = 0; i < ARRAY_SIZE(rf); i++) {
                DeviceState *dev = qdev_new("misc.esp32s3.rfstub");
                qdev_prop_set_uint32(dev, "base", rf[i].base);
                qdev_prop_set_uint32(dev, "size", rf[i].size);
                sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
                memory_region_add_subregion_overlap(sys_mem, rf[i].base,
                    sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 0), 0);
            }
        }

        /* The SAR ADC. Always fitted, because every one of these boards has
         * one and a firmware that reads an absent one waits for ever - which
         * is what the published companion build did here, polling the done
         * bit twenty-nine million times in a single run. */
        {
            DeviceState *adc = qdev_new(TYPE_ESP32S3_SARADC);
            if (mach->input_path && *mach->input_path) {
                qdev_prop_set_string(adc, "path", mach->input_path);
            }
            sysbus_realize_and_unref(SYS_BUS_DEVICE(adc), &error_fatal);
            Esp32s3SarAdcState *a = ESP32S3_SARADC(adc);
            if (mach->bat_adc_channel < ESP32S3_SARADC_CHANNELS) {
                a->raw[mach->bat_adc_channel] = (uint16_t)mach->bat_adc_raw;
            }
            memory_region_add_subregion_overlap(sys_mem, DR_REG_SENS_BASE,
                sysbus_mmio_get_region(SYS_BUS_DEVICE(adc), 0), 0);
        }
        /* The display, on whichever I2C controller the board puts it. Only
         * when the board declares one: a machine with no panel path has no
         * display, and a driver that probes for one is told nothing answers,
         * which is exactly what it is told today. */
        /* Buttons, where the board has any and something is listening. Each
         * declared pin gets a line into the GPIO controller, so a press is
         * the pin going where a finger would take it. */
        if (mach->input_path && *mach->input_path &&
            mach->input_pins && *mach->input_pins) {
            DeviceState *in = qdev_new("mb-input");
            qdev_prop_set_string(in, "path", mach->input_path);
            qdev_prop_set_string(in, "pins", mach->input_pins);
            sysbus_realize_and_unref(SYS_BUS_DEVICE(in), &error_fatal);
            const char *p = mach->input_pins;
            for (int i = 0; *p; i++) {
                char *end = NULL;
                long pin = strtol(p, &end, 10);
                if (end == p) {
                    break;
                }
                qdev_connect_gpio_out_named(in, "mb-input", i,
                                            qdev_get_gpio_in_named(DEVICE(&ss->gpio),
                                                                   ESP32_GPIO_IN,
                                                                   (int)pin));
                p = (*end == ',') ? end + 1 : end;
            }
        }

        /* A keyboard and a touch panel, where the board has them. Both are
         * I2C devices and both listen on the same socket the buttons do,
         * picking out the messages tagged for them. */
        if (mach->input_path && *mach->input_path && mach->kbd_addr != 0) {
            unsigned n = mach->panel_i2c < ESP32S3_I2C_COUNT ? mach->panel_i2c : 0;
            I2CBus *bus = I2C_BUS(qdev_get_child_bus(DEVICE(&ss->i2c[n]), "i2c"));
            DeviceState *kbd = qdev_new("tdeck-keyboard");
            qdev_prop_set_string(kbd, "path", mach->input_path);
            qdev_prop_set_uint8(kbd, "address", (uint8_t)mach->kbd_addr);
            qdev_realize_and_unref(kbd, BUS(bus), &error_fatal);
        }
        if (mach->input_path && *mach->input_path && mach->touch_addr != 0) {
            unsigned n = mach->panel_i2c < ESP32S3_I2C_COUNT ? mach->panel_i2c : 0;
            I2CBus *bus = I2C_BUS(qdev_get_child_bus(DEVICE(&ss->i2c[n]), "i2c"));
            DeviceState *tp = qdev_new("gt911-touch");
            qdev_prop_set_string(tp, "path", mach->input_path);
            qdev_prop_set_uint8(tp, "address", (uint8_t)mach->touch_addr);
            qdev_realize_and_unref(tp, BUS(bus), &error_fatal);
        }

        if (mach->panel_path && *mach->panel_path &&
            mach->panel_cs != ESP32S3_RADIO_PIN_NONE) {
            /* A colour panel shares the radio's controller, told apart by its
             * own select. Both peripherals ignore bytes that arrive while
             * their select is high, which is what makes sharing safe. */
            DeviceState *master = mach->radio_spi == 3
                ? DEVICE(&ss->gpspi3) : DEVICE(&ss->gpspi2);
            BusState *bus = qdev_get_child_bus(master, "spi");
            DeviceState *tft = qdev_new("st7789-panel");
            qdev_prop_set_string(tft, "path", mach->panel_path);
            qdev_prop_set_uint32(tft, "width", mach->panel_w);
            qdev_prop_set_uint32(tft, "height", mach->panel_h);
            /* The bus wants a distinct index per peripheral even though
             * neither device uses the controller's own select - both are
             * framed by a GPIO the firmware drives. Any index but the
             * radio's will do. */
            qdev_prop_set_uint8(tft, "cs", mach->radio_cs == 0 ? 1 : 0);
            qdev_realize_and_unref(tft, bus, &error_fatal);
            qdev_connect_gpio_out_named(DEVICE(&ss->gpio), ESP32_GPIO_OUT,
                                        mach->panel_cs,
                                        qdev_get_gpio_in_named(tft, "st7789-cs", 0));
            if (mach->panel_dc != ESP32S3_RADIO_PIN_NONE) {
                qdev_connect_gpio_out_named(DEVICE(&ss->gpio), ESP32_GPIO_OUT,
                                            mach->panel_dc,
                                            qdev_get_gpio_in_named(tft, "st7789-dc", 0));
            }
        } else if (mach->panel_path && *mach->panel_path) {
            unsigned n = mach->panel_i2c < ESP32S3_I2C_COUNT ? mach->panel_i2c : 0;
            I2CBus *bus = I2C_BUS(qdev_get_child_bus(DEVICE(&ss->i2c[n]), "i2c"));
            DeviceState *panel = qdev_new("ssd1306-panel");
            qdev_prop_set_string(panel, "path", mach->panel_path);
            qdev_prop_set_uint32(panel, "column-offset", mach->panel_offset);
            qdev_prop_set_uint8(panel, "address", (uint8_t)mach->panel_addr);
            qdev_realize_and_unref(panel, BUS(bus), &error_fatal);
        }
    }

    /* Need MMU initialized prior to ELF loading,
     * so that ELF gets loaded into virtual addresses
     */
    cpu_reset(CPU(&ss->cpu[0]));

    const char *load_elf_filename = NULL;
    if (machine->firmware) {
        load_elf_filename = machine->firmware;
    }
    if (machine->kernel_filename) {
        qemu_log("Warning: both -bios and -kernel arguments specified. Only loading the the -kernel file.\n");
        load_elf_filename = machine->kernel_filename;
    }

    if (load_elf_filename) {
        uint64_t elf_entry;
        uint64_t elf_lowaddr;
        int size = load_elf(load_elf_filename, NULL,
                               translate_phys_addr, &ss->cpu[0],
                               &elf_entry, &elf_lowaddr,
                               NULL, NULL, 0, EM_XTENSA, 0, 0);
        if (size < 0) {
            error_report("Error: could not load ELF file '%s'", load_elf_filename);
            exit(1);
        }

        if (elf_entry != XCHAL_RESET_VECTOR_PADDR) {
            // Since ROM is empty when loading elf file AND
            // PC value is 0x40000400 after reset
            // need to jump to elf entry point to run a programm
            uint8_t p[4];
            memcpy(p, &elf_entry, 4);
            uint8_t boot[] = {
                0x06, 0x01, 0x00,       /* j    1 */
                0x00,                   /* .literal_position */
                p[0], p[1], p[2], p[3], /* .literal elf_entry */
                                        /* 1: */
                0x01, 0xff, 0xff,       /* l32r a0, elf_entry */
                0xa0, 0x00, 0x00,       /* jx   a0 */
            };
            // Write boot function to reset-vector address (0x40000400) of the CPU 0
            rom_add_blob_fixed_as("boot", boot, sizeof(boot), XCHAL_RESET_VECTOR_PADDR, CPU(&ss->cpu[0])->as);
            ss->cpu[0].env.pc = XCHAL_RESET_VECTOR_PADDR;
        }
    } else {
        char *rom_binary = qemu_find_file(QEMU_FILE_TYPE_BIOS, "esp32s3_rev0_rom.bin");
        if (rom_binary == NULL) {
            error_report("Error: -bios argument not set, and ROM code binary not found (1)");
            exit(1);
        }

        int size = load_image_targphys_as(rom_binary, esp32s3_memmap[ESP32S3_MEMREGION_IROM].base, esp32s3_memmap[ESP32S3_MEMREGION_IROM].size, CPU(&ss->cpu[0])->as);
        if (size < 0) {
            error_report("Error: could not load ROM binary '%s'", rom_binary);
            exit(1);
        }
        g_free(rom_binary);

        if (ESP32S3_CPU_COUNT > 1)
        {
            rom_binary = qemu_find_file(QEMU_FILE_TYPE_BIOS, "esp32s3_rev0_rom.bin");
            if (rom_binary == NULL) {
                error_report("Error: -bios argument not set, and ROM code binary not found (2)");
                exit(1);
            }

            size = load_image_targphys_as(rom_binary, esp32s3_memmap[ESP32S3_MEMREGION_IROM].base, esp32s3_memmap[ESP32S3_MEMREGION_IROM].size, CPU(&ss->cpu[1])->as);
            if (size < 0) {
                error_report("Error: could not load ROM binary '%s'", rom_binary);
                exit(1);
            }
            g_free(rom_binary);
        }
    }
}


static ram_addr_t esp32s3_fixup_ram_size(ram_addr_t requested_size)
{
    ram_addr_t size;
    if (requested_size == 0) {
        size = 0;
    } else if (requested_size <= 2 * MiB) {
        size = 2 * MiB;
    } else if (requested_size <= 4 * MiB ) {
        size = 4 * MiB;
    } else if (requested_size <= 8 * MiB ) {
        size = 8 * MiB;
    } else if (requested_size <= 16 * MiB ) {
        size = 16 * MiB;
    } else if (requested_size <= 32 * MiB ) {
        size = 32 * MiB;
    } else {
        qemu_log("RAM size larger than 32 MB not supported\n");
        size = 32 * MiB;
    }
    return size;
}

/* Initialize machine type */
static void esp32s3_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    mc->desc = "Espressif ESP32S3 machine";
    mc->init = esp32s3_machine_init;
    mc->max_cpus = 2;
    mc->default_cpus = 2;
    mc->default_ram_size = 0;
    mc->fixup_ram_size = esp32s3_fixup_ram_size;
}

static bool esp32s3_get_cp_at_reset(Object *obj, Error **errp)
{
    return ESP32S3_MACHINE(obj)->cp_at_reset;
}

static void esp32s3_set_cp_at_reset(Object *obj, bool value, Error **errp)
{
    ESP32S3_MACHINE(obj)->cp_at_reset = value;
}

static bool esp32s3_get_psram_octal(Object *obj, Error **errp)
{
    return ESP32S3_MACHINE(obj)->psram_octal;
}

static void esp32s3_set_psram_octal(Object *obj, bool value, Error **errp)
{
    ESP32S3_MACHINE(obj)->psram_octal = value;
}

static char *esp32s3_get_radio_path(Object *obj, Error **errp)
{
    return g_strdup(ESP32S3_MACHINE(obj)->radio_path);
}

static void esp32s3_set_radio_path(Object *obj, const char *value, Error **errp)
{
    Esp32s3MachineState *ms = ESP32S3_MACHINE(obj);
    g_free(ms->radio_path);
    ms->radio_path = g_strdup(value);
}

static char *esp32s3_get_input_path(Object *obj, Error **errp)
{
    return g_strdup(ESP32S3_MACHINE(obj)->input_path);
}

static void esp32s3_set_input_path(Object *obj, const char *value, Error **errp)
{
    Esp32s3MachineState *ms = ESP32S3_MACHINE(obj);
    g_free(ms->input_path);
    ms->input_path = g_strdup(value);
}

static char *esp32s3_get_input_pins(Object *obj, Error **errp)
{
    return g_strdup(ESP32S3_MACHINE(obj)->input_pins);
}

static void esp32s3_set_input_pins(Object *obj, const char *value, Error **errp)
{
    Esp32s3MachineState *ms = ESP32S3_MACHINE(obj);
    g_free(ms->input_pins);
    ms->input_pins = g_strdup(value);
}

static char *esp32s3_get_panel_path(Object *obj, Error **errp)
{
    return g_strdup(ESP32S3_MACHINE(obj)->panel_path);
}

static void esp32s3_set_panel_path(Object *obj, const char *value, Error **errp)
{
    Esp32s3MachineState *ms = ESP32S3_MACHINE(obj);
    g_free(ms->panel_path);
    ms->panel_path = g_strdup(value);
}

static void esp32s3_machine_instance_init(Object *obj)
{
    Esp32s3MachineState *ms = ESP32S3_MACHINE(obj);

    ms->radio_spi = 2;   /* GPSPI2 - FSPI, Arduino's SPI object on this part */
    ms->radio_cs = 0;
    ms->radio_nss = 41;  /* Xiao S3 WIO; per board */
    ms->radio_busy = 40;
    /* No interrupt line unless the board says which pin. */
    ms->radio_dio1 = ESP32S3_RADIO_PIN_NONE;
    ms->panel_i2c = 0;
    ms->panel_addr = 0x3C;
    ms->panel_offset = 0;
    ms->panel_cs = ESP32S3_RADIO_PIN_NONE;
    ms->panel_dc = ESP32S3_RADIO_PIN_NONE;
    ms->kbd_addr = 0;
    ms->touch_addr = 0;
    ms->bat_adc_channel = 0;
    ms->bat_adc_raw = 0;
    ms->card_cs = ESP32S3_RADIO_PIN_NONE;
    ms->panel_w = 320;
    ms->panel_h = 240;
    ms->radio_fem = ESP32S3_RADIO_PIN_NONE;
    ms->psram_octal = false;
    ms->cp_at_reset = false;
    object_property_add_bool(obj, "cp-at-reset",
                             esp32s3_get_cp_at_reset, esp32s3_set_cp_at_reset);
    object_property_set_description(obj, "cp-at-reset",
        "bring the coprocessors up enabled, which the part does not: a "
        "debugging aid for firmware whose exception handler saves floating "
        "point state before anything has enabled the FPU, where the trap that "
        "causes is fatal and hides everything behind it");
    object_property_add_bool(obj, "psram-octal",
                             esp32s3_get_psram_octal, esp32s3_set_psram_octal);
    object_property_set_description(obj, "psram-octal",
        "the external RAM is an octal (OPI) part rather than a quad one");
    object_property_add_str(obj, "radio-path",
                            esp32s3_get_radio_path, esp32s3_set_radio_path);
    object_property_set_description(obj, "radio-path",
        "unix socket of the SX1262 model to attach to the SPI bus");
    object_property_add_uint32_ptr(obj, "radio-spi", &ms->radio_spi,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_set_description(obj, "radio-spi",
        "which SPI controller the radio is on (default 2, GPSPI2)");
    object_property_add_uint32_ptr(obj, "radio-cs", &ms->radio_cs,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_set_description(obj, "radio-cs",
        "chip select the radio answers on (default 0)");
    object_property_add_uint32_ptr(obj, "radio-nss", &ms->radio_nss,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_set_description(obj, "radio-nss",
        "GPIO the driver toggles as the radio's chip select (default 41)");
    object_property_add_uint32_ptr(obj, "radio-busy", &ms->radio_busy,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_set_description(obj, "radio-busy",
        "GPIO the radio drives as its BUSY line (default 40)");
    object_property_add_uint32_ptr(obj, "radio-dio1", &ms->radio_dio1,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_set_description(obj, "radio-dio1",
        "GPIO the radio drives as its DIO1 interrupt, which is the only way "
        "the firmware learns a packet arrived (default none)");
    object_property_add_str(obj, "panel-path",
                            esp32s3_get_panel_path, esp32s3_set_panel_path);
    object_property_set_description(obj, "panel-path",
        "unix socket the board's display sends its picture to; without it the "
        "board has no display, which is what a board with none looks like");
    object_property_add_uint32_ptr(obj, "panel-i2c", &ms->panel_i2c,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_set_description(obj, "panel-i2c",
        "which I2C controller the display is on (default 0)");
    object_property_add_uint32_ptr(obj, "panel-addr", &ms->panel_addr,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_set_description(obj, "panel-addr",
        "the display's I2C address (default 0x3C)");
    object_property_add_str(obj, "input-path",
                            esp32s3_get_input_path, esp32s3_set_input_path);
    object_property_set_description(obj, "input-path",
        "unix socket button presses arrive on; without it the board's buttons "
        "sit where their pull-ups leave them, which is nobody pressing them");
    object_property_add_str(obj, "input-pins",
                            esp32s3_get_input_pins, esp32s3_set_input_pins);
    object_property_set_description(obj, "input-pins",
        "comma separated GPIOs a press may move, which are the board's own "
        "button pins");
    object_property_add_uint32_ptr(obj, "card-cs", &ms->card_cs,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_set_description(obj, "card-cs",
        "GPIO that selects the card slot on the radio's own SPI bus");
    object_property_add_uint32_ptr(obj, "bat-adc-channel", &ms->bat_adc_channel,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_set_description(obj, "bat-adc-channel",
        "ADC1 channel the board's battery divider is wired to");
    object_property_add_uint32_ptr(obj, "bat-adc-raw", &ms->bat_adc_raw,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_set_description(obj, "bat-adc-raw",
        "what that channel reads at bring-up, as the twelve bit number the "
        "firmware sees");
    object_property_add_uint32_ptr(obj, "kbd-addr", &ms->kbd_addr,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_set_description(obj, "kbd-addr",
        "I2C address the board's keyboard answers on, or zero for a board "
        "with none");
    object_property_add_uint32_ptr(obj, "touch-addr", &ms->touch_addr,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_set_description(obj, "touch-addr",
        "I2C address the board's touch panel answers on, or zero for none");
    object_property_add_uint32_ptr(obj, "panel-cs", &ms->panel_cs,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_set_description(obj, "panel-cs",
        "GPIO the driver toggles as a colour panel's chip select; giving it "
        "puts the display on the radio's SPI controller rather than on I2C");
    object_property_add_uint32_ptr(obj, "panel-dc", &ms->panel_dc,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_set_description(obj, "panel-dc",
        "GPIO that says whether a byte to the panel is a command or data");
    object_property_add_uint32_ptr(obj, "panel-w", &ms->panel_w,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "panel-h", &ms->panel_h,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "panel-offset", &ms->panel_offset,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_set_description(obj, "panel-offset",
        "column offset of the display: 0 for an SSD1306, 2 for an SH1106, "
        "and getting it wrong slides the whole picture sideways");
    object_property_add_uint32_ptr(obj, "radio-fem", &ms->radio_fem,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_set_description(obj, "radio-fem",
        "GPIO the firmware drives as the front-end module's transmit enable "
        "(default none)");
}

static const TypeInfo esp32s3_info = {
    .name = TYPE_ESP32S3_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(Esp32s3MachineState),
    .instance_init = esp32s3_machine_instance_init,
    .class_init = esp32s3_machine_class_init,
};

static void esp32s3_machine_type_init(void)
{
    type_register_static(&esp32s3_info);
}

type_init(esp32s3_machine_type_init);

