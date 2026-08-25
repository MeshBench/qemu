/*
 * ESP32-S3 general-purpose SPI master (GPSPI2 and GPSPI3).
 *
 * Not the same peripheral as the one that drives the flash, and not the same
 * registers: the flash controller's are the SPI_MEM_* set, where a transfer is
 * started by bit 18 of its command register and data sits at offset 0x58. A
 * general-purpose controller starts on bit 24, latches its configuration on
 * bit 23, and keeps its data at 0x98. A board whose radio is driven through
 * one of these and modelled with the other looks exactly like a board with no
 * radio fitted.
 */
#pragma once

#include "hw/hw.h"
#include "hw/registerfields.h"
#include "hw/ssi/ssi.h"
#include "hw/dma/esp_gdma.h"

#define TYPE_ESP32S3_GPSPI "ssi.esp32s3.gpspi"
#define ESP32S3_GPSPI(obj) \
    OBJECT_CHECK(ESP32S3GpspiState, (obj), TYPE_ESP32S3_GPSPI)

/* Sixteen words of transfer buffer, as the part has. */
#define ESP32S3_GPSPI_BUF_WORDS 16
#define ESP32S3_GPSPI_CS_COUNT 3
#define ESP32S3_GPSPI_IO_SIZE 0x100

REG32(GPSPI_CMD, 0x000)
    FIELD(GPSPI_CMD, CONF_BITLEN, 0, 18)
    FIELD(GPSPI_CMD, UPDATE, 23, 1)
    FIELD(GPSPI_CMD, USR, 24, 1)
REG32(GPSPI_ADDR, 0x004)
REG32(GPSPI_CTRL, 0x008)
REG32(GPSPI_CLOCK, 0x00C)
REG32(GPSPI_USER, 0x010)
    FIELD(GPSPI_USER, DOUTDIN, 0, 1)
    FIELD(GPSPI_USER, MISO, 28, 1)
    FIELD(GPSPI_USER, MOSI, 27, 1)
    FIELD(GPSPI_USER, DUMMY, 29, 1)
    FIELD(GPSPI_USER, ADDR, 30, 1)
    FIELD(GPSPI_USER, COMMAND, 31, 1)
REG32(GPSPI_USER1, 0x014)
    FIELD(GPSPI_USER1, DUMMY_CYCLELEN, 0, 8)
    FIELD(GPSPI_USER1, ADDR_BITLEN, 27, 5)
REG32(GPSPI_USER2, 0x018)
    FIELD(GPSPI_USER2, COMMAND_VALUE, 0, 16)
    FIELD(GPSPI_USER2, COMMAND_BITLEN, 28, 4)
REG32(GPSPI_MS_DLEN, 0x01C)
    FIELD(GPSPI_MS_DLEN, MS_DATA_BITLEN, 0, 18)
REG32(GPSPI_MISC, 0x020)
REG32(GPSPI_DMA_CONF, 0x030)
    /* The two bits ESP-IDF sets to route a transfer through GDMA rather than
     * the CPU data registers. Present since forever on this part; unmodelled
     * here until a DMA-only display driver (esp-hal's) waited on a transfer
     * that never moved. */
    FIELD(GPSPI_DMA_CONF, DMA_RX_ENA, 27, 1)
    FIELD(GPSPI_DMA_CONF, DMA_TX_ENA, 28, 1)
REG32(GPSPI_DMA_INT_ENA, 0x034)
/*
 * 0x038 and 0x03C keep the meanings this model has always given them, and
 * both keep reading back transfer-done. On the part 0x038 is the clear
 * register and 0x03C the raw one, but a driver that polls here for completion
 * has been served a done bit for as long as this machine has existed -
 * MeshCore's own build is one, and correcting the offsets underneath it left
 * its radio waiting for ever, three runs out of three. Writes to 0x038 clear,
 * which is what the part does and what an interrupt-driven driver needs.
 */
REG32(GPSPI_DMA_INT_RAW, 0x038)
REG32(GPSPI_DMA_INT_ST, 0x03C)
/* Bit 12 of the interrupt registers: the transfer this controller was told to
 * do has finished. The one interrupt an SPI master driver waits on. */
#define GPSPI_INT_TRANS_DONE (1u << 12)
REG32(GPSPI_W0, 0x098)
REG32(GPSPI_DATE, 0x0F0)

typedef struct ESP32S3GpspiState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    SSIBus *spi;
    qemu_irq cs_gpio[ESP32S3_GPSPI_CS_COUNT];
    /* The general DMA controller and which peripheral id this SPI is on it
     * (GDMA_SPI2 or GDMA_SPI3). A transfer with DMA enabled takes its data
     * from, and returns it to, the GDMA channel bound to that id rather than
     * the CPU data registers. Null on a machine wired without GDMA. */
    ESPGdmaState *gdma;
    int dma_peri;
    /* The line into the interrupt matrix, and what is armed and pending on it.
     * A driver that arms the transfer-done interrupt and sleeps has nothing
     * else to wake it. */
    qemu_irq irq;
    uint32_t int_ena;
    uint32_t int_raw;

    uint32_t cmd;
    uint32_t addr;
    uint32_t ctrl;
    uint32_t clock;
    uint32_t user;
    uint32_t user1;
    uint32_t user2;
    uint32_t ms_dlen;
    uint32_t misc;
    uint32_t dma_conf;
    uint32_t data_reg[ESP32S3_GPSPI_BUF_WORDS];
} ESP32S3GpspiState;

void esp32s3_gpspi_share_bus(ESP32S3GpspiState *from, ESP32S3GpspiState *onto);
