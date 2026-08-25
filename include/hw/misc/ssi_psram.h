#pragma once

#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/ssi/ssi.h"
#include "qom/object.h"
#include "exec/memory.h"


typedef enum PsramState {
    ST_IDLE = 0,
    ST_CMD_LSB,     /* Received the command LSB */
    ST_CMD_READY,   /* Received the command */
    ST_CMD_ADDR0,   /* Received the 1st byte of the 32-bit address */
    ST_CMD_ADDR1,   /* Received the 2nd byte of the 32-bit address */
    ST_CMD_ADDR2,   /* Received the 3rd byte of the 32-bit address */
    ST_DUMMY_CYCLE, /* Dummy cycles between the address and the data */
    ST_PROCESSING,  /* 32-bit address received, sending/receiving data */
    ST_READ_ID,     /* Received ID command */
} PsramState;


typedef struct SsiPsramState {
    SSIPeripheral parent_obj;
    uint32_t size_mbytes;
    uint32_t dummy;
    uint32_t command;
    /*
     * The address as the part sees it: 32 bits, unsigned. It was signed, and
     * an address with the top bit set - which the tuning the ESP-IDF v5.5
     * bootloader runs against octal PSRAM produces - came out negative. The
     * bounds check only tested the upper end, a negative index passed it, and
     * the read landed before the buffer.
     */
    uint32_t addr;
    uint32_t byte_count;
    int dummy_cycles;
    bool is_octal;

    uint8_t mr0;
    uint8_t mr1;
    uint8_t mr2;
    uint8_t mr3;
    uint8_t mr4;
    uint8_t mr8;

    PsramState state;
    MemoryRegion data_mr;
} SsiPsramState;

#define TYPE_SSI_PSRAM "ssi_psram"
OBJECT_DECLARE_SIMPLE_TYPE(SsiPsramState, SSI_PSRAM)

