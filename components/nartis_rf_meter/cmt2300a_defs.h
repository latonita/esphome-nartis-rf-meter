/*
 * CMT2300A Register Definitions
 *
 * Based on CMOSTEK CMT2300A datasheet v1.8 and cmt2300a_defs.h v1.2
 * Original Copyright (C) CMOSTEK SZ.
 *
 * Extended with state machine command defines and Nartis-specific constants.
 */

#pragma once

#include <cstdint>

namespace esphome::nartis_rf_meter {

/* ================================================================
 * Register Bank Addresses and Sizes
 * ================================================================ */
static constexpr uint8_t CMT_BANK_ADDR = 0x00;
static constexpr uint8_t CMT_BANK_SIZE = 12;

static constexpr uint8_t SYSTEM_BANK_ADDR = 0x0C;
static constexpr uint8_t SYSTEM_BANK_SIZE = 12;

static constexpr uint8_t FREQUENCY_BANK_ADDR = 0x18;
static constexpr uint8_t FREQUENCY_BANK_SIZE = 8;

static constexpr uint8_t DATA_RATE_BANK_ADDR = 0x20;
static constexpr uint8_t DATA_RATE_BANK_SIZE = 24;

static constexpr uint8_t BASEBAND_BANK_ADDR = 0x38;
static constexpr uint8_t BASEBAND_BANK_SIZE = 29;

static constexpr uint8_t TX_BANK_ADDR = 0x55;
static constexpr uint8_t TX_BANK_SIZE = 11;

/* Total config: 12 + 12 + 8 + 24 + 29 + 11 = 96 bytes */
static constexpr uint8_t TOTAL_CONFIG_SIZE = 96;

/* ================================================================
 * Register Addresses — Configuration Banks (0x00–0x5F)
 * ================================================================ */

/* CMT Bank */
static constexpr uint8_t REG_CMT1 = 0x00;
static constexpr uint8_t REG_CMT2 = 0x01;  // Product ID
static constexpr uint8_t REG_CMT3 = 0x02;
static constexpr uint8_t REG_CMT4 = 0x03;
static constexpr uint8_t REG_CMT5 = 0x04;
static constexpr uint8_t REG_CMT6 = 0x05;
static constexpr uint8_t REG_CMT7 = 0x06;
static constexpr uint8_t REG_CMT8 = 0x07;
static constexpr uint8_t REG_CMT9 = 0x08;
static constexpr uint8_t REG_CMT10 = 0x09;
static constexpr uint8_t REG_CMT11 = 0x0A;
static constexpr uint8_t REG_RSSI = 0x0B;

/* System Bank */
static constexpr uint8_t REG_SYS1 = 0x0C;
static constexpr uint8_t REG_SYS2 = 0x0D;
static constexpr uint8_t REG_SYS3 = 0x0E;
static constexpr uint8_t REG_SYS4 = 0x0F;
static constexpr uint8_t REG_SYS5 = 0x10;
static constexpr uint8_t REG_SYS6 = 0x11;
static constexpr uint8_t REG_SYS7 = 0x12;
static constexpr uint8_t REG_SYS8 = 0x13;
static constexpr uint8_t REG_SYS9 = 0x14;
static constexpr uint8_t REG_SYS10 = 0x15;
static constexpr uint8_t REG_SYS11 = 0x16;
static constexpr uint8_t REG_SYS12 = 0x17;

/* Frequency Bank */
static constexpr uint8_t REG_RF1 = 0x18;
static constexpr uint8_t REG_RF2 = 0x19;
static constexpr uint8_t REG_RF3 = 0x1A;
static constexpr uint8_t REG_RF4 = 0x1B;
static constexpr uint8_t REG_RF5 = 0x1C;
static constexpr uint8_t REG_RF6 = 0x1D;
static constexpr uint8_t REG_RF7 = 0x1E;
static constexpr uint8_t REG_RF8 = 0x1F;

/* Data Rate Bank */
static constexpr uint8_t REG_RF9 = 0x20;
static constexpr uint8_t REG_RF10 = 0x21;
static constexpr uint8_t REG_RF11 = 0x22;
static constexpr uint8_t REG_RF12 = 0x23;
static constexpr uint8_t REG_FSK1 = 0x24;
static constexpr uint8_t REG_FSK2 = 0x25;
static constexpr uint8_t REG_FSK3 = 0x26;
static constexpr uint8_t REG_FSK4 = 0x27;
static constexpr uint8_t REG_FSK5 = 0x28;
static constexpr uint8_t REG_FSK6 = 0x29;
static constexpr uint8_t REG_FSK7 = 0x2A;
static constexpr uint8_t REG_CDR1 = 0x2B;
static constexpr uint8_t REG_CDR2 = 0x2C;
static constexpr uint8_t REG_CDR3 = 0x2D;
static constexpr uint8_t REG_CDR4 = 0x2E;
static constexpr uint8_t REG_AGC1 = 0x2F;
static constexpr uint8_t REG_AGC2 = 0x30;
static constexpr uint8_t REG_AGC3 = 0x31;
static constexpr uint8_t REG_AGC4 = 0x32;
static constexpr uint8_t REG_OOK1 = 0x33;
static constexpr uint8_t REG_OOK2 = 0x34;
static constexpr uint8_t REG_OOK3 = 0x35;
static constexpr uint8_t REG_OOK4 = 0x36;
static constexpr uint8_t REG_OOK5 = 0x37;

/* Baseband Bank */
static constexpr uint8_t REG_PKT1 = 0x38;
static constexpr uint8_t REG_PKT2 = 0x39;
static constexpr uint8_t REG_PKT3 = 0x3A;
static constexpr uint8_t REG_PKT4 = 0x3B;
static constexpr uint8_t REG_PKT5 = 0x3C;
static constexpr uint8_t REG_PKT6 = 0x3D;
static constexpr uint8_t REG_PKT7 = 0x3E;
static constexpr uint8_t REG_PKT8 = 0x3F;
static constexpr uint8_t REG_PKT9 = 0x40;
static constexpr uint8_t REG_PKT10 = 0x41;
static constexpr uint8_t REG_PKT11 = 0x42;
static constexpr uint8_t REG_PKT12 = 0x43;
static constexpr uint8_t REG_PKT13 = 0x44;
static constexpr uint8_t REG_PKT14 = 0x45;
static constexpr uint8_t REG_PKT15 = 0x46;
static constexpr uint8_t REG_PKT16 = 0x47;
static constexpr uint8_t REG_PKT17 = 0x48;
static constexpr uint8_t REG_PKT18 = 0x49;
static constexpr uint8_t REG_PKT19 = 0x4A;
static constexpr uint8_t REG_PKT20 = 0x4B;
static constexpr uint8_t REG_PKT21 = 0x4C;
static constexpr uint8_t REG_PKT22 = 0x4D;
static constexpr uint8_t REG_PKT23 = 0x4E;
static constexpr uint8_t REG_PKT24 = 0x4F;
static constexpr uint8_t REG_PKT25 = 0x50;
static constexpr uint8_t REG_PKT26 = 0x51;
static constexpr uint8_t REG_PKT27 = 0x52;
static constexpr uint8_t REG_PKT28 = 0x53;
static constexpr uint8_t REG_PKT29 = 0x54;

/* TX Bank */
static constexpr uint8_t REG_TX1 = 0x55;
static constexpr uint8_t REG_TX2 = 0x56;
static constexpr uint8_t REG_TX3 = 0x57;
static constexpr uint8_t REG_TX4 = 0x58;
static constexpr uint8_t REG_TX5 = 0x59;
static constexpr uint8_t REG_TX6 = 0x5A;
static constexpr uint8_t REG_TX7 = 0x5B;
static constexpr uint8_t REG_TX8 = 0x5C;
static constexpr uint8_t REG_TX9 = 0x5D;
static constexpr uint8_t REG_TX10 = 0x5E;
static constexpr uint8_t REG_LBD = 0x5F;

/* ================================================================
 * Register Addresses — Control Banks (0x60–0x71)
 * ================================================================ */

/* Control1 Bank */
static constexpr uint8_t REG_MODE_CTL = 0x60;
static constexpr uint8_t REG_MODE_STA = 0x61;
static constexpr uint8_t REG_EN_CTL = 0x62;
static constexpr uint8_t REG_FREQ_CHNL = 0x63;
static constexpr uint8_t REG_FREQ_OFS = 0x64;
static constexpr uint8_t REG_IO_SEL = 0x65;
static constexpr uint8_t REG_INT1_CTL = 0x66;
static constexpr uint8_t REG_INT2_CTL = 0x67;
static constexpr uint8_t REG_INT_EN = 0x68;
static constexpr uint8_t REG_FIFO_CTL = 0x69;
static constexpr uint8_t REG_INT_CLR1 = 0x6A;

/* Control2 Bank */
static constexpr uint8_t REG_INT_CLR2 = 0x6B;
static constexpr uint8_t REG_FIFO_CLR = 0x6C;
static constexpr uint8_t REG_INT_FLAG = 0x6D;
static constexpr uint8_t REG_FIFO_FLAG = 0x6E;
static constexpr uint8_t REG_RSSI_CODE = 0x6F;
static constexpr uint8_t REG_RSSI_DBM = 0x70;
static constexpr uint8_t REG_LBD_RESULT = 0x71;

/* Soft reset register */
static constexpr uint8_t REG_SOFT_RST = 0x7F;
static constexpr uint8_t SOFT_RST_VALUE = 0xFF;

/* ================================================================
 * Chip Mode — GO commands (write to REG_MODE_CTL)
 * ================================================================ */
static constexpr uint8_t GO_EEPROM = 0x01;
static constexpr uint8_t GO_STBY = 0x02;
static constexpr uint8_t GO_RFS = 0x04;
static constexpr uint8_t GO_RX = 0x08;
static constexpr uint8_t GO_SLEEP = 0x10;
static constexpr uint8_t GO_TFS = 0x20;
static constexpr uint8_t GO_TX = 0x40;
static constexpr uint8_t GO_SWITCH = 0x80;

/* ================================================================
 * Chip Status — read from REG_MODE_STA
 * ================================================================ */
static constexpr uint8_t MASK_CHIP_MODE_STA = 0x0F;
static constexpr uint8_t MASK_CFG_RETAIN = 0x10;

static constexpr uint8_t STA_IDLE = 0x00;
static constexpr uint8_t STA_SLEEP = 0x01;
static constexpr uint8_t STA_STBY = 0x02;
static constexpr uint8_t STA_RFS = 0x03;
static constexpr uint8_t STA_TFS = 0x04;
static constexpr uint8_t STA_RX = 0x05;
static constexpr uint8_t STA_TX = 0x06;
static constexpr uint8_t STA_EEPROM = 0x07;
static constexpr uint8_t STA_ERROR = 0x08;
static constexpr uint8_t STA_CAL = 0x09;

/* ================================================================
 * Interrupt Flag Register (REG_INT_FLAG = 0x6D)
 * ================================================================ */
static constexpr uint8_t FLAG_LBD = 0x80;
static constexpr uint8_t FLAG_COL_ERR = 0x40;
static constexpr uint8_t FLAG_PKT_ERR = 0x20;
static constexpr uint8_t FLAG_PREAM_OK = 0x10;
static constexpr uint8_t FLAG_SYNC_OK = 0x08;
static constexpr uint8_t FLAG_NODE_OK = 0x04;
static constexpr uint8_t FLAG_CRC_OK = 0x02;
static constexpr uint8_t FLAG_PKT_OK = 0x01;

/* ================================================================
 * INT_CLR1 Register (REG_INT_CLR1 = 0x6A)
 * ================================================================ */
static constexpr uint8_t CLR1_SL_TMO_FLG = 0x20;
static constexpr uint8_t CLR1_RX_TMO_FLG = 0x10;
static constexpr uint8_t CLR1_TX_DONE_FLG = 0x08;
static constexpr uint8_t CLR1_TX_DONE_CLR = 0x04;
static constexpr uint8_t CLR1_SL_TMO_CLR = 0x02;
static constexpr uint8_t CLR1_RX_TMO_CLR = 0x01;

/* ================================================================
 * INT_CLR2 Register (REG_INT_CLR2 = 0x6B)
 * ================================================================ */
static constexpr uint8_t CLR2_LBD_CLR = 0x20;
static constexpr uint8_t CLR2_PREAM_OK_CLR = 0x10;
static constexpr uint8_t CLR2_SYNC_OK_CLR = 0x08;
static constexpr uint8_t CLR2_NODE_OK_CLR = 0x04;
static constexpr uint8_t CLR2_CRC_OK_CLR = 0x02;
static constexpr uint8_t CLR2_PKT_DONE_CLR = 0x01;

/* ================================================================
 * FIFO_CLR Register (REG_FIFO_CLR = 0x6C)
 * ================================================================ */
static constexpr uint8_t FIFO_RESTORE = 0x04;
static constexpr uint8_t FIFO_CLR_RX = 0x02;
static constexpr uint8_t FIFO_CLR_TX = 0x01;

/* ================================================================
 * FIFO_FLAG Register (REG_FIFO_FLAG = 0x6E)
 * ================================================================ */
static constexpr uint8_t FIFO_RX_FULL = 0x40;
static constexpr uint8_t FIFO_RX_NMTY = 0x20;
static constexpr uint8_t FIFO_RX_TH = 0x10;
static constexpr uint8_t FIFO_RX_OVF = 0x08;
static constexpr uint8_t FIFO_TX_FULL = 0x04;
static constexpr uint8_t FIFO_TX_NMTY = 0x02;
static constexpr uint8_t FIFO_TX_TH = 0x01;

/* ================================================================
 * FIFO_CTL Register (REG_FIFO_CTL = 0x69)
 * ================================================================ */
static constexpr uint8_t MASK_FIFO_MERGE_EN = 0x02;
static constexpr uint8_t MASK_FIFO_RX_TX_SEL = 0x04;
static constexpr uint8_t MASK_SPI_FIFO_RD_WR_SEL = 0x01;

/* ================================================================
 * IO_SEL Register (REG_IO_SEL = 0x65) — GPIO function mapping
 * ================================================================ */
static constexpr uint8_t MASK_GPIO1_SEL = 0x03;
static constexpr uint8_t MASK_GPIO2_SEL = 0x0C;
static constexpr uint8_t MASK_GPIO3_SEL = 0x30;
static constexpr uint8_t MASK_GPIO4_SEL = 0xC0;

/* GPIO1 options */
static constexpr uint8_t GPIO1_SEL_DOUT = 0x00;
static constexpr uint8_t GPIO1_SEL_INT1 = 0x01;
static constexpr uint8_t GPIO1_SEL_INT2 = 0x02;
static constexpr uint8_t GPIO1_SEL_DCLK = 0x03;

/* GPIO2 options */
static constexpr uint8_t GPIO2_SEL_INT1 = 0x00;
static constexpr uint8_t GPIO2_SEL_INT2 = 0x04;
static constexpr uint8_t GPIO2_SEL_DOUT = 0x08;
static constexpr uint8_t GPIO2_SEL_DCLK = 0x0C;

/* ================================================================
 * INT1_CTL / INT2_CTL — Interrupt source selection
 * ================================================================ */
static constexpr uint8_t MASK_INT1_SEL = 0x1F;
static constexpr uint8_t MASK_INT_POLAR = 0x20;

static constexpr uint8_t INT_SEL_RX_ACTIVE = 0x00;
static constexpr uint8_t INT_SEL_TX_ACTIVE = 0x01;
static constexpr uint8_t INT_SEL_RSSI_VLD = 0x02;
static constexpr uint8_t INT_SEL_PREAM_OK = 0x03;
static constexpr uint8_t INT_SEL_SYNC_OK = 0x04;
static constexpr uint8_t INT_SEL_NODE_OK = 0x05;
static constexpr uint8_t INT_SEL_CRC_OK = 0x06;
static constexpr uint8_t INT_SEL_PKT_OK = 0x07;
static constexpr uint8_t INT_SEL_SL_TMO = 0x08;
static constexpr uint8_t INT_SEL_RX_TMO = 0x09;
static constexpr uint8_t INT_SEL_TX_DONE = 0x0A;
static constexpr uint8_t INT_SEL_RX_FIFO_NMTY = 0x0B;
static constexpr uint8_t INT_SEL_RX_FIFO_TH = 0x0C;
static constexpr uint8_t INT_SEL_RX_FIFO_FULL = 0x0D;
static constexpr uint8_t INT_SEL_RX_FIFO_WBYTE = 0x0E;
static constexpr uint8_t INT_SEL_RX_FIFO_OVF = 0x0F;
static constexpr uint8_t INT_SEL_TX_FIFO_NMTY = 0x10;
static constexpr uint8_t INT_SEL_TX_FIFO_TH = 0x11;
static constexpr uint8_t INT_SEL_TX_FIFO_FULL = 0x12;
static constexpr uint8_t INT_SEL_STATE_IS_STBY = 0x13;
static constexpr uint8_t INT_SEL_STATE_IS_FS = 0x14;
static constexpr uint8_t INT_SEL_STATE_IS_RX = 0x15;
static constexpr uint8_t INT_SEL_STATE_IS_TX = 0x16;
static constexpr uint8_t INT_SEL_LED = 0x17;
static constexpr uint8_t INT_SEL_TRX_ACTIVE = 0x18;
static constexpr uint8_t INT_SEL_PKT_DONE = 0x19;

/* ================================================================
 * INT_EN Register (REG_INT_EN = 0x68)
 * ================================================================ */
static constexpr uint8_t INT_EN_SL_TMO = 0x80;
static constexpr uint8_t INT_EN_RX_TMO = 0x40;
static constexpr uint8_t INT_EN_TX_DONE = 0x20;
static constexpr uint8_t INT_EN_PREAM_OK = 0x10;
static constexpr uint8_t INT_EN_SYNC_OK = 0x08;
static constexpr uint8_t INT_EN_NODE_OK = 0x04;
static constexpr uint8_t INT_EN_CRC_OK = 0x02;
static constexpr uint8_t INT_EN_PKT_DONE = 0x01;

/* ================================================================
 * SYS11 Register (REG_SYS11 = 0x16) — FIFO merge
 * ================================================================ */
static constexpr uint8_t MASK_FIFO_MERGE_CFG = 0xE0;  // top 3 bits preserved

/* PKT29 Register (REG_PKT29 = 0x54) — FIFO threshold */
static constexpr uint8_t MASK_FIFO_TH = 0x7F;
static constexpr uint8_t MASK_FIFO_AUTO_RES_EN = 0x80;

/* ================================================================
 * SPI Protocol Constants
 * ================================================================ */
static constexpr uint8_t SPI_WRITE = 0x00;  // Bit 7 = 0 for write
static constexpr uint8_t SPI_READ = 0x80;   // Bit 7 = 1 for read

/* ================================================================
 * Nartis Firmware Register Configuration
 * Verbatim from flash.bin at 0x1373C (96 bytes across 6 banks)
 * ================================================================ */

// clang-format off
static constexpr uint8_t NARTIS_CMT_BANK[12] = {
    0x00, 0x66, 0xEC, 0x1D, 0xF0, 0x80, 0x14, 0x08, 0x91, 0x02, 0x02, 0xA0
};

static constexpr uint8_t NARTIS_SYSTEM_BANK[12] = {
    0xAE, 0xE0, 0x35, 0x00, 0x00, 0xF4, 0x10, 0xE2, 0x42, 0x20, 0x00, 0x81
};

static constexpr uint8_t NARTIS_DATA_RATE_BANK[24] = {
    0x32, 0x18, 0x00, 0x99, 0xC1, 0x9B, 0x07, 0x0A,
    0x9F, 0x39, 0x29, 0x29, 0xC0, 0x51, 0x2A, 0x53,
    0x00, 0x00, 0xB4, 0x00, 0x00, 0x01, 0x00, 0x00
};

static constexpr uint8_t NARTIS_BASEBAND_BANK[29] = {
    0x2A, 0x0A, 0x00, 0x55, 0x06, 0x00, 0x00, 0x00,  // PKT1-PKT8
    0x00, 0xF6, 0x55, 0x55, 0x55, 0x10, 0xFF, 0x00,  // PKT9-PKT16
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60,  // PKT17-PKT24 (0x60=PN9 whiten ON, Manchester OFF)
    0xFF, 0x00, 0x00, 0x1F, 0x10                      // PKT25-PKT29 (0xFF=whiten seed)
};

static constexpr uint8_t NARTIS_TX_BANK[11] = {
    0x50, 0xC7, 0x03, 0x00, 0x42, 0xB0, 0x00, 0x8A, 0x18, 0x3F, 0x7F
};

/* 4-channel frequency presets (8 bytes each: 4 RX + 4 TX) */
static constexpr uint8_t NARTIS_FREQ_CHANNELS[4][8] = {
    {0x42, 0x6D, 0x8F, 0x1C, 0x42, 0x57, 0xDD, 0x1B},  // CH0: RX 434.10 / TX 433.82 MHz
    {0x42, 0xBF, 0x47, 0x1B, 0x42, 0xA9, 0x95, 0x1A},  // CH1: RX 433.58 / TX 433.30 MHz
    {0x42, 0xB2, 0xA4, 0x1D, 0x42, 0x9B, 0xF2, 0x1C},  // CH2: RX 434.54 / TX 434.26 MHz
    {0x42, 0xF6, 0xB9, 0x1E, 0x42, 0xE0, 0x07, 0x1E},  // CH3: RX 434.98 / TX 434.70 MHz
};
// clang-format on

/* Runtime override values (applied after bank writes) */
static constexpr uint8_t FIFO_MERGE_VALUE = 0x12;   // REG_SYS11: (reg & 0xE0) | 0x12 — merge TX+RX = 64B
static constexpr uint8_t FIFO_TH_VALUE = 0x0C;      // REG_PKT29: (reg & 0xE0) | 0x0C — threshold = 12 bytes
static constexpr uint8_t TX_REFILL_CHUNK = 15;       // Firmware refills 15 bytes per TX_FIFO_TH event (cmt_tx_chunked_write (0x131B8))

/* Number of frequency channels */
static constexpr uint8_t NUM_CHANNELS = 4;

/* FIFO size when merged */
static constexpr uint8_t FIFO_SIZE_MERGED = 64;

/* Timing constants */
static constexpr uint32_t RESET_DELAY_MS = 20;
static constexpr uint32_t STATE_POLL_TIMEOUT_MS = 10;
static constexpr uint32_t STATE_POLL_INTERVAL_US = 100;

/* Chunked RX queue capacity — enough for max 290-byte packet at 12 bytes/chunk */
static constexpr size_t RX_QUEUE_CAPACITY = 30;

}  // namespace esphome::nartis_rf_meter
