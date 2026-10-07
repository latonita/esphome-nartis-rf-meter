/*
 * TI CC1101 register definitions - RF433-1 (Nartis D101) profile.
 *
 * Names and numbering follow the CC1101 datasheet (SWRS061I). This header is
 * deliberately standalone: ESPHome ships a `cc1101` component with an equivalent
 * table, but that one is an OOK-oriented driver in its own namespace and is not a
 * dependency of this project, so the layout is mirrored here rather than imported.
 *
 * Everything that depends on a user-settable parameter (carrier, deviation, RX
 * bandwidth, output power, modulation shaping, the FOC carrier-sense gate) is
 * solved at runtime in cc1101_hal.cpp from the datasheet formulas - only the fixed
 * part of the profile lives in the table below, so there is exactly one place where
 * a magic number can be wrong.
 *
 * ----------------------------------------------------------------------------
 * RF433-1 IS NOT RF433-2. This file is the sibling of the cc1101_defs.h in the
 * nartis_rf_2_meter and uart_nartis_rf projects and keeps the same shape on
 * purpose, but every PHY number differs and one structural thing differs:
 *
 *   RF433-1 (here)        2400 bps, +-6 kHz (h ~= 5), MSB-first on air,
 *                         sync 55 55 F6 72 on air, 10 x 0x55 preamble,
 *                         +14 dBm CIU.
 *   RF433-2 (siblings)    1200 bps, +-25 kHz (h ~= 41), LSB-first on air,
 *                         sync 98 F3, 12 x 0x55 preamble, +20 dBm CIU.
 *
 * MSB-first is the CC1101's native shift order, so there is NO bit reversal
 * anywhere in this backend - not on the payload, not in the sync registers. The
 * sibling projects reverse8() every byte in both directions; copying that here
 * would put a radio on the air that nothing can decode. See the note at
 * CC1101_SYNC_WORD_DEFAULT and the one in Cc1101Hal::transmit().
 *
 * Ground truth for the numbers below: a real D101 CIU SPI bus capture
 * (fw/dump-spi/dump.txt) is byte-identical across all 96 CMT2300A configuration
 * registers to a CMOSTEK RFPDK export (fw/d101-rfpdk/a.exp), so that export's
 * parameter list IS the device's PHY - 2.4 kbps, 6.0 kHz deviation, GFSK BT 0.5,
 * 4-byte sync, 10-byte TX preamble / 5-byte RX detect, whitening/Manchester/FEC/CRC
 * all disabled, payload bit order "start from msb". fw/RF_PHY_SPEC.md carries the
 * SDR confirmation of the same figures.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome::nartis_rf_meter {

/* ================================================================
 * Crystal
 * ================================================================ */

/* Every CC1101 register formula is referred to the crystal. 26 MHz is what the
 * common E07/CC1101 modules fit, and it is also what the CMT2300A bridge runs
 * (XTAL 26 MHz throughout cmt2300a_defs.h), so the two backends share the same
 * reference and the same rounding behaviour. A 27 MHz module would need this
 * changed and every derived value re-checked. */
static constexpr uint32_t CC1101_XTAL_HZ = 26000000u;

/// Synthesiser step, Fxosc / 2^16 = 396.728 Hz. Only used for logging the residual.
static constexpr float CC1101_FREQ_STEP_HZ = 26000000.0f / 65536.0f;

/* ================================================================
 * SPI header byte
 * ================================================================ */

/* The first byte of every access is R/W | BURST | address<5:0>. Status registers
 * (0x30-0x3D) collide with the command strobes and must be read with BURST set,
 * otherwise the chip executes a strobe instead of returning the register. */
static constexpr uint8_t CC1101_HDR_WRITE = 0x00;
static constexpr uint8_t CC1101_HDR_READ = 0x80;
static constexpr uint8_t CC1101_HDR_BURST = 0x40;

/* ================================================================
 * Configuration registers (0x00-0x2E)
 * ================================================================ */
static constexpr uint8_t CC1101_REG_IOCFG2 = 0x00;    // GDO2 pin configuration
static constexpr uint8_t CC1101_REG_IOCFG1 = 0x01;    // GDO1 pin configuration (shared with MISO)
static constexpr uint8_t CC1101_REG_IOCFG0 = 0x02;    // GDO0 pin configuration
static constexpr uint8_t CC1101_REG_FIFOTHR = 0x03;   // RX/TX FIFO thresholds
static constexpr uint8_t CC1101_REG_SYNC1 = 0x04;     // Sync word, high byte
static constexpr uint8_t CC1101_REG_SYNC0 = 0x05;     // Sync word, low byte
static constexpr uint8_t CC1101_REG_PKTLEN = 0x06;    // Packet length (fixed mode)
static constexpr uint8_t CC1101_REG_PKTCTRL1 = 0x07;  // Packet automation control
static constexpr uint8_t CC1101_REG_PKTCTRL0 = 0x08;  // Packet automation control
static constexpr uint8_t CC1101_REG_ADDR = 0x09;      // Device address
static constexpr uint8_t CC1101_REG_CHANNR = 0x0A;    // Channel number
static constexpr uint8_t CC1101_REG_FSCTRL1 = 0x0B;   // Synthesiser control (IF)
static constexpr uint8_t CC1101_REG_FSCTRL0 = 0x0C;   // Synthesiser control (freq offset)
static constexpr uint8_t CC1101_REG_FREQ2 = 0x0D;     // Carrier word <23:16>
static constexpr uint8_t CC1101_REG_FREQ1 = 0x0E;     // Carrier word <15:8>
static constexpr uint8_t CC1101_REG_FREQ0 = 0x0F;     // Carrier word <7:0>
static constexpr uint8_t CC1101_REG_MDMCFG4 = 0x10;   // CHANBW_E/M + DRATE_E
static constexpr uint8_t CC1101_REG_MDMCFG3 = 0x11;   // DRATE_M
static constexpr uint8_t CC1101_REG_MDMCFG2 = 0x12;   // Modulation / Manchester / sync mode
static constexpr uint8_t CC1101_REG_MDMCFG1 = 0x13;   // FEC / preamble length / channel spacing exp
static constexpr uint8_t CC1101_REG_MDMCFG0 = 0x14;   // Channel spacing mantissa
static constexpr uint8_t CC1101_REG_DEVIATN = 0x15;   // DEVIATION_E/M
static constexpr uint8_t CC1101_REG_MCSM2 = 0x16;     // RX timeout
static constexpr uint8_t CC1101_REG_MCSM1 = 0x17;     // CCA / RXOFF / TXOFF
static constexpr uint8_t CC1101_REG_MCSM0 = 0x18;     // Auto-calibration / PO timeout
static constexpr uint8_t CC1101_REG_FOCCFG = 0x19;    // Frequency offset compensation
static constexpr uint8_t CC1101_REG_BSCFG = 0x1A;     // Bit synchronisation
static constexpr uint8_t CC1101_REG_AGCCTRL2 = 0x1B;  // AGC control
static constexpr uint8_t CC1101_REG_AGCCTRL1 = 0x1C;  // AGC control
static constexpr uint8_t CC1101_REG_AGCCTRL0 = 0x1D;  // AGC control
static constexpr uint8_t CC1101_REG_WOREVT1 = 0x1E;   // Wake-on-radio event, high
static constexpr uint8_t CC1101_REG_WOREVT0 = 0x1F;   // Wake-on-radio event, low
static constexpr uint8_t CC1101_REG_WORCTRL = 0x20;   // Wake-on-radio control
static constexpr uint8_t CC1101_REG_FREND1 = 0x21;    // RX front end
static constexpr uint8_t CC1101_REG_FREND0 = 0x22;    // TX front end (PA_POWER -> PATABLE index)
static constexpr uint8_t CC1101_REG_FSCAL3 = 0x23;    // Synthesiser calibration
static constexpr uint8_t CC1101_REG_FSCAL2 = 0x24;
static constexpr uint8_t CC1101_REG_FSCAL1 = 0x25;
static constexpr uint8_t CC1101_REG_FSCAL0 = 0x26;
static constexpr uint8_t CC1101_REG_RCCTRL1 = 0x27;  // RC oscillator configuration
static constexpr uint8_t CC1101_REG_RCCTRL0 = 0x28;
static constexpr uint8_t CC1101_REG_FSTEST = 0x29;   // Test registers - datasheet says do not write
static constexpr uint8_t CC1101_REG_PTEST = 0x2A;
static constexpr uint8_t CC1101_REG_AGCTEST = 0x2B;
static constexpr uint8_t CC1101_REG_TEST2 = 0x2C;  // Must track the RX filter bandwidth
static constexpr uint8_t CC1101_REG_TEST1 = 0x2D;
static constexpr uint8_t CC1101_REG_TEST0 = 0x2E;  // VCO selection calibration

/* ================================================================
 * Status registers (0x30-0x3D) - read-only, burst bit mandatory
 * ================================================================ */
static constexpr uint8_t CC1101_REG_PARTNUM = 0x30;    // 0x00 on a genuine CC1101
static constexpr uint8_t CC1101_REG_VERSION = 0x31;    // 0x14 on current die revisions
static constexpr uint8_t CC1101_REG_FREQEST = 0x32;    // Frequency offset estimate from FOC
static constexpr uint8_t CC1101_REG_LQI = 0x33;        // Link quality
static constexpr uint8_t CC1101_REG_RSSI = 0x34;       // RSSI, 2's complement half-dB
static constexpr uint8_t CC1101_REG_MARCSTATE = 0x35;  // Main radio control state machine
static constexpr uint8_t CC1101_REG_WORTIME1 = 0x36;
static constexpr uint8_t CC1101_REG_WORTIME0 = 0x37;
static constexpr uint8_t CC1101_REG_PKTSTATUS = 0x38;  // GDO levels, CS, CRC_OK
static constexpr uint8_t CC1101_REG_VCO_VC_DAC = 0x39;
static constexpr uint8_t CC1101_REG_TXBYTES = 0x3A;  // <6:0> count, <7> underflow
static constexpr uint8_t CC1101_REG_RXBYTES = 0x3B;  // <6:0> count, <7> overflow
static constexpr uint8_t CC1101_REG_RCCTRL1_STATUS = 0x3C;
static constexpr uint8_t CC1101_REG_RCCTRL0_STATUS = 0x3D;

/* Multi-byte registers: PATABLE auto-increments through its 8 entries on a burst
 * access and wraps; the FIFO is one address that reads RX and writes TX. */
static constexpr uint8_t CC1101_REG_PATABLE = 0x3E;
static constexpr uint8_t CC1101_REG_FIFO = 0x3F;

static constexpr uint8_t CC1101_PARTNUM_EXPECTED = 0x00;
static constexpr uint8_t CC1101_VERSION_EXPECTED = 0x14;

static constexpr size_t CC1101_FIFO_SIZE = 64;
static constexpr size_t CC1101_PATABLE_SIZE = 8;

/* The largest NUM_RXBYTES that is a real count, not a bus fault. The RX FIFO is 64
 * bytes, but the receive chain also holds one byte in a prefetch register ahead of
 * it, and RXBYTES counts that byte too - so 65 is a documented, legitimate state and
 * NOT evidence of anything wrong. TI's errata note SWRZ020E ("RX FIFO problem")
 * shows exactly this: it is at 65, with the next byte, that the FIFO overflows, and
 * its workaround is to read n - 1 bytes while the chip is still receiving, which is
 * what drain_rx() does unconditionally anyway. With this profile - APPEND_STATUS = 0
 * in PKTCTRL1, CRC_EN = 0 in PKTCTRL0, FEC_EN = 0 in MDMCFG1 - there are no status,
 * CRC or FEC bytes in flight to make the accounting any different. A count ABOVE 65
 * cannot be produced by the part at all, so that one really is a bus fault. */
static constexpr uint8_t CC1101_RXBYTES_MAX_VALID = (uint8_t) (CC1101_FIFO_SIZE + 1);

static constexpr uint8_t CC1101_TXBYTES_UNDERFLOW = 0x80;
static constexpr uint8_t CC1101_RXBYTES_OVERFLOW = 0x80;
static constexpr uint8_t CC1101_FIFO_COUNT_MASK = 0x7F;

/* ================================================================
 * Command strobes (0x30-0x3D with the burst bit clear)
 * ================================================================ */
static constexpr uint8_t CC1101_STROBE_SRES = 0x30;     // Reset chip
static constexpr uint8_t CC1101_STROBE_SFSTXON = 0x31;  // Calibrate and hold the synthesiser
static constexpr uint8_t CC1101_STROBE_SXOFF = 0x32;    // Crystal off
static constexpr uint8_t CC1101_STROBE_SCAL = 0x33;     // Calibrate synthesiser, then IDLE
static constexpr uint8_t CC1101_STROBE_SRX = 0x34;      // Enable RX
static constexpr uint8_t CC1101_STROBE_STX = 0x35;      // Enable TX
static constexpr uint8_t CC1101_STROBE_SIDLE = 0x36;    // Leave RX/TX
static constexpr uint8_t CC1101_STROBE_SWOR = 0x38;     // Wake-on-radio
static constexpr uint8_t CC1101_STROBE_SPWD = 0x39;     // Power down on CS high
static constexpr uint8_t CC1101_STROBE_SFRX = 0x3A;     // Flush RX FIFO (IDLE or overflow only)
static constexpr uint8_t CC1101_STROBE_SFTX = 0x3B;     // Flush TX FIFO (IDLE or underflow only)
static constexpr uint8_t CC1101_STROBE_SWORRST = 0x3C;
static constexpr uint8_t CC1101_STROBE_SNOP = 0x3D;  // No-op: the cheapest way to fetch the status byte

/* ================================================================
 * Status byte (returned on MISO during every header byte)
 * ================================================================ */
static constexpr uint8_t CC1101_STATUS_CHIP_RDYN = 0x80;
static constexpr uint8_t CC1101_STATUS_STATE_MASK = 0x70;
static constexpr uint8_t CC1101_STATUS_FIFO_MASK = 0x0F;

static constexpr uint8_t CC1101_STATE_IDLE = 0;
static constexpr uint8_t CC1101_STATE_RX = 1;
static constexpr uint8_t CC1101_STATE_TX = 2;
static constexpr uint8_t CC1101_STATE_FSTXON = 3;
static constexpr uint8_t CC1101_STATE_CALIBRATE = 4;
static constexpr uint8_t CC1101_STATE_SETTLING = 5;
static constexpr uint8_t CC1101_STATE_RXFIFO_OVERFLOW = 6;
static constexpr uint8_t CC1101_STATE_TXFIFO_UNDERFLOW = 7;

/* ================================================================
 * MARCSTATE (0x35) - the fine-grained state, <4:0>
 * ================================================================ */
static constexpr uint8_t CC1101_MARCSTATE_MASK = 0x1F;
static constexpr uint8_t CC1101_MARC_SLEEP = 0x00;
static constexpr uint8_t CC1101_MARC_IDLE = 0x01;
static constexpr uint8_t CC1101_MARC_XOFF = 0x02;
static constexpr uint8_t CC1101_MARC_MANCAL = 0x05;
static constexpr uint8_t CC1101_MARC_FS_LOCK = 0x0A;
static constexpr uint8_t CC1101_MARC_ENDCAL = 0x0C;
static constexpr uint8_t CC1101_MARC_RX = 0x0D;
static constexpr uint8_t CC1101_MARC_RX_END = 0x0E;
static constexpr uint8_t CC1101_MARC_RXFIFO_OVERFLOW = 0x11;
static constexpr uint8_t CC1101_MARC_FSTXON = 0x12;
static constexpr uint8_t CC1101_MARC_TX = 0x13;
static constexpr uint8_t CC1101_MARC_TX_END = 0x14;
static constexpr uint8_t CC1101_MARC_TXFIFO_UNDERFLOW = 0x16;

/* ================================================================
 * Field encodings used by the driver
 * ================================================================ */

/* PKTCTRL0<1:0>.
 *
 * RX runs INFINITE. The CC1101's variable-length engine reads the first byte after
 * sync as the packet length, and RF433-1's first payload byte is total_length - 1
 * (FRAME_HEADER_SPEC.md sec. 1) - off by one, and in any case the frames this component
 * builds reach ~292 bytes, which an 8-bit length field cannot express. Infinite mode
 * lets the engine stream forever and the component bounds the frame by the LEN byte
 * itself, exactly as the drain_rx() contract in radio_hal.h describes.
 *
 * TX starts FIXED for frames that PKTLEN can name on its own, and uses the
 * infinite-to-fixed transition (TI DN500) for anything longer - see
 * Cc1101Hal::transmit(). */
static constexpr uint8_t CC1101_PKTCTRL0_LENGTH_FIXED = 0x00;
static constexpr uint8_t CC1101_PKTCTRL0_LENGTH_VARIABLE = 0x01;
static constexpr uint8_t CC1101_PKTCTRL0_LENGTH_INFINITE = 0x02;

/* PKTCTRL0 with whitening off (bit 6), normal FIFO packet format (bits 5:4 = 00)
 * and the PHY CRC off (bit 2). RF433-1 has none of the three: whitening is OFF
 * (RF_PHY_SPEC.md sec. 4.6, confirmed three ways - no dewhitening in the decoder, a
 * clean 0x55 preamble in the capture, and WHITEN_EN clear in PKT24), and the only
 * CRC is the software CRC-16/DNP (poly 0x3D65) the transport layer applies. Turning
 * the chip's CRC on would compute the wrong polynomial and reject every frame. */
static constexpr uint8_t CC1101_PKTCTRL0_TX = CC1101_PKTCTRL0_LENGTH_FIXED;
static constexpr uint8_t CC1101_PKTCTRL0_RX = CC1101_PKTCTRL0_LENGTH_INFINITE;

/* ----------------------------------------------------------------------------
 * Sync word
 * ----------------------------------------------------------------------------
 *
 * RF433-1's sync word is FOUR bytes, MSB-first on air, zero bit errors tolerated,
 * and the on-air order is 55 55 F6 72:
 *
 *      ... 0x55 0x55 0x55 | 55 55 F6 72 | LEN TYPE ... CRC
 *          <- preamble -> | <-  sync -> | <- payload ->
 *
 * THE ORDER MATTERS, AND THE PROJECT'S OWN SPECS HAVE IT BACKWARDS.
 * fw/RF_PHY_SPEC.md sec. 4.4 and fw/FRAME_HEADER_SPEC.md:823 both print the sync
 * word as "72 F6 55 55". That figure is the CMT2300A REGISTER listing PKT10..PKT13
 * read as though it were an air sequence; it is not one. Both documents should be
 * treated as carrying a documentation bug here. They are outside this component and
 * are deliberately not edited from it - this comment is the record, so that the next
 * reader who opens the spec is not misled by it. The spec already attaches the
 * caveat that the figure is register-derived rather than SDR-verified, and the
 * capture tooling never validated it either: fw/tools/decode_capture.py anchors on
 * the CD 2C address bytes at frame offset 2, not on the sync word.
 *
 * Three independent things establish the real order.
 *
 *  1. THE CMT2300A FILL RULE. CMOSTEK AN143 (fw/tools/an143.txt, the SYNC_VALUE
 *     table and the paragraph under it): sync bytes fill SYNC_VALUE<63:0> from the
 *     TOP, so for a 4-byte sync PKT13 (<63:56>) is the FIRST byte on air and PKT10
 *     (<39:32>) the last. AN143's own worked example says it outright - for the
 *     2-byte sync 0x5678, 0x56 goes into <63:56> = PKT13 and 0x78 into
 *     <55:48> = PKT12.
 *
 *  2. CROSS-CHECK ON A PHY WHERE THE ANSWER IS ALREADY KNOWN. The sibling RF433-2
 *     projects run a hardware-proven CMT2300A driver: for an on-air 55 19 CF they
 *     write PKT13 = 0x55, PKT12 = 0x19, PKT11 = 0xCF (nartis_rf_2_meter and
 *     uart_nartis_rf, cmt2300a_hal.cpp). PKT13 first, descending. The rule holds on
 *     silicon, not just on paper.
 *
 *  3. AN EXACT CROSS-CHECK ON THIS PHY. The RFPDK export fw/d101-rfpdk/a.exp carries
 *     "Sync Value = 1431655926" = 0x555555F6. By the AN143 rule its MSB 0x55 lands
 *     in <63:56> = PKT13 and its LSB 0xF6 in <39:32> = PKT10, i.e. the register bank
 *     must read PKT10 = F6, PKT11 = 55, PKT12 = 55, PKT13 = 55 - which is exactly
 *     the "41 F6 42 55 43 55 44 55" write in the real CIU bus capture
 *     (fw/dump-spi/dump.txt). On air that default is 55 55 55 F6, ENDING in the F6
 *     marker, which is also why the SDR decode headers under fw/iq3/ say
 *     "sync ..F6". The CIU's "normal mode" patch then rewrites the bank to
 *     PKT10 = 72, PKT11 = F6, PKT12 = 55, PKT13 = 55 (the "41 72 42 F6 43 55 44 55"
 *     write in the same capture), i.e. 55 55 F6 72 on air: it EXTENDS the marker
 *     from one byte to two, immediately ahead of the length byte. That is physically
 *     coherent. The reversed reading, 72 F6 55 55, would instead place two
 *     preamble-indistinguishable bytes BETWEEN the marker and LEN, which is not a
 *     thing a framing does.
 *
 * WHAT THIS MEANS FOR THE CC1101. The leading 55 55 of the sync word is byte-
 * identical to the preamble, so it genuinely does merge with it - the correlator
 * only has to find the two bytes that are not 0x55. SYNC1/SYNC0 hold F6 72 with
 * MDMCFG2.SYNC_MODE = 010 (16 of 16, no bit errors tolerated), and NOTHING has to be
 * carried through the FIFO in either direction: no bytes prepended on transmit, none
 * discarded on receive. (An earlier revision of this driver did exactly that, on the
 * reversed reading of the sync word. It was wrong and is gone.)
 *
 * The byte counts then line up exactly, which is worth recording: the CIU emits 10
 * preamble bytes (a.exp "Preamble Tx Size = 10") plus the two 0x55 sync bytes = 12
 * bytes of 0x55 ahead of F6 72. MDMCFG1.NUM_PREAMBLE = 101 is 12 preamble bytes, so
 * the CC1101 puts 55 x12 | F6 72 | LEN ... on the air - byte-identical to a real
 * CIU, not merely close enough for its correlator.
 *
 * NO BIT REVERSAL. RF433-1 is MSB-first on air ("Payload Bit Order = Start from msb"
 * in a.exp) and the CC1101 shifts MSB-first, so these registers hold the on-air
 * bytes LITERALLY. The sibling RF433-2 projects keep reverse8()-ed values here
 * because that PHY is LSB-first; do not copy that by symmetry.
 *
 * WHY 16 BITS IS THE ONLY OPTION ANYWAY. RF_PHY_SPEC.md sec. 4.4 offers "Option A:
 * 32-bit sync 0x72F65555" and sec. 11 item 4 lists 16-vs-32 as an open question to
 * settle on hardware. The datasheet closes it regardless of byte order: the CC1101
 * has no 32-bit sync REGISTER, and MDMCFG2.SYNC_MODE = 011/111 emulates one by
 * REPEATING SYNC1:SYNC0 - the correlator would be looking for F6 72 F6 72, which the
 * meter never transmits, and every reply would be missed. A genuine four-byte sync
 * word is not programmable on this part. With the order established above that costs
 * nothing at all: the two bytes a 32-bit mode could not have expressed are 55 55,
 * and 55 55 is the preamble.
 *
 * NOT YET CONFIRMED ON AIR FOR THIS PROTOCOL. The order above is derived (AN143 plus
 * the register capture) and cross-validated on RF433-2; it has not been read off an
 * RF433-1 SDR decode, and RF_PHY_SPEC.md sec. 11 still lists it as an open item.
 * That is why this is a YAML key (`sync_word`, see Cc1101Hal::set_sync_word()) and
 * not a hard constant: if the meter never answers and nothing is ever received,
 * `sync_word: 0x72F6` is the one alternative worth trying, and it costs a line of
 * YAML rather than a rebuild.
 */
static constexpr uint16_t CC1101_SYNC_WORD_DEFAULT = 0xF672u;

/* MDMCFG2 fields assembled in Cc1101Hal::write_config_(). DEM_DCFILT_OFF stays 0
 * (DC blocking on - required at anything but the very highest data rates) and
 * MANCHESTER_EN stays 0 (a.exp: "Manchester = Disable"). */
static constexpr uint8_t CC1101_MDMCFG2_MOD_2FSK = 0x00;  // MOD_FORMAT = 000
static constexpr uint8_t CC1101_MDMCFG2_MOD_GFSK = 0x10;  // MOD_FORMAT = 001, Gaussian BT = 0.5
static constexpr uint8_t CC1101_MDMCFG2_SYNC_16_16 = 0x02;  // SYNC_MODE = 010, exact 16-bit match

/* MDMCFG3: data rate mantissa. R = (256 + DRATE_M) * 2^DRATE_E * Fxosc / 2^28.
 * Target 2400 baud needs (256 + M) * 2^E = 2400 * 2^28 / 26e6 = 24778.7, which only
 * E = 6 can express with M in [0, 255]: M = 24778.7/64 - 256 = 131.2 -> 131.
 * (256 + 131) * 64 * 26e6 / 2^28 = 2398.97 baud, i.e. -0.04 % - four orders of
 * magnitude inside what the bit synchroniser tolerates over a ~300 byte frame.
 *
 * Note this is exactly double the sibling RF433-2 projects' DRATE_E = 5, DRATE_M =
 * 131, which is the arithmetic saying what the two PHYs say: 2400 bps here, 1200
 * there, same mantissa, one more exponent step.
 *
 * 2400 bps is SDR-MEASURED, not inferred (RF_PHY_SPEC.md sec. 4.2: preamble fundamental
 * 1200 Hz = Rb/2, 2400 transitions/s counted over 40 ms, and full CRC-16/DNP
 * validating decodes at BITRATE = 2400). It is a fixed property of the PHY, so
 * unlike bandwidth and deviation it is not exposed as a tunable. */
static constexpr uint8_t CC1101_DRATE_E = 6;
static constexpr uint8_t CC1101_DRATE_M = 131;

/* The same rate as an integer, in milli-baud: (256 + 131) * 2^6 * 26e6 / 2^28 =
 * 2398.96774... baud exactly (the fraction reduces to 78609375 / 32768). Kept here
 * so transmit()'s airtime arithmetic uses the rate the chip is actually programmed
 * with rather than the nominal 2400, and so that the two cannot drift apart if
 * DRATE_E/M are ever retuned. Integer milli-baud also keeps that arithmetic free of
 * floating point on a path that runs per transmission. */
static constexpr uint32_t CC1101_DRATE_MILLIBAUD = 2398968u;

/* What the packet engine puts on the air ahead of the FIFO data, which airtime has
 * to account for: NUM_PREAMBLE = 101 in MDMCFG1 below is 12 preamble bytes, and
 * SYNC1/SYNC0 is 2 more. These are the register settings restated as numbers, so
 * MDMCFG1 and the TX deadline cannot disagree about the preamble length. Nothing
 * else precedes the FIFO data: the whole frame, and only the frame, goes through
 * the FIFO. */
static constexpr uint32_t CC1101_PREAMBLE_BYTES = 12u;
static constexpr uint32_t CC1101_SYNC_BYTES = 2u;

/* FOCCFG. Everything but FOC_BS_CS_GATE is fixed: FOC_PRE_K = 10 (3K before sync),
 * FOC_POST_K = 1 (K/2 after), FOC_LIMIT = 10 (+-BW/4), i.e. 0x16. FOC_BS_CS_GATE is
 * bit 5, so setting it gives 0x36 and changes nothing else - see the long note at
 * Cc1101Hal::set_foc_cs_gate() for why that bit is a bring-up knob rather than a
 * decision this driver can make from the datasheet. */
static constexpr uint8_t CC1101_FOCCFG_BASE = 0x16;
static constexpr uint8_t CC1101_FOCCFG_BS_CS_GATE = 0x20;

/* TEST2/TEST1 are not "do not write" registers like FSTEST/PTEST/AGCTEST: TI gives
 * two characterised pairs for them and says which to use, keyed on the RX channel
 * filter bandwidth. 0x88/0x31 is the pair for bandwidths ABOVE 101 kHz and 0x81/0x35
 * the pair at or below it (CC1101 datasheet SWRS061, the TEST2/TEST1 register
 * descriptions, and what SmartRF Studio emits either side of that bandwidth). The
 * driver picks between them from the solved bandwidth in Cc1101Hal::solve_params_(),
 * so the whole 58-812 kHz range set_rx_bandwidth_hz() accepts stays characterised
 * instead of only the lower half of it.
 *
 * Worth knowing for this profile: the 101562 Hz default is 562 Hz ABOVE the
 * threshold, so it selects the WIDE pair. That is intended, not an accident of
 * rounding - see CC1101_DEFAULT_RX_BANDWIDTH_HZ. */
static constexpr uint8_t CC1101_TEST2_NARROW_BW = 0x81;
static constexpr uint8_t CC1101_TEST1_NARROW_BW = 0x35;
static constexpr uint8_t CC1101_TEST2_WIDE_BW = 0x88;
static constexpr uint8_t CC1101_TEST1_WIDE_BW = 0x31;
/* The bandwidth that divides the two pairs above: at or below it the narrow pair,
 * strictly above it the wide pair. */
static constexpr uint32_t CC1101_TEST_REG_BW_THRESHOLD_HZ = 101000u;

/* FSCTRL1.FREQ_IF, as programmed in the table below: IF = FREQ_IF * Fxosc / 2^10.
 * Restated here as a number so solve_params_() can check the solved RX bandwidth
 * against it without re-deriving the register. */
static constexpr uint8_t CC1101_FREQ_IF = 6;
static constexpr uint32_t CC1101_IF_HZ = (uint32_t) (((uint64_t) CC1101_FREQ_IF * CC1101_XTAL_HZ) >> 10);

/* ================================================================
 * Defaults for the YAML-settable knobs
 * ================================================================ */

/* Channel 0, 433.820 MHz - the meter's wake frequency and the alternative-mode TX
 * carrier (see CHANNEL_CARRIER_HZ in radio_hal.h, which is the authority; this is a
 * resting value only). The component calls set_frequency()/set_rx_frequency() with
 * the selected channel before every exchange, so this is what the chip is tuned to
 * between init() and the first channel selection and nothing more. */
static constexpr uint32_t CC1101_DEFAULT_FREQUENCY_HZ = 433820000u;

/* RX is centred on the channel carrier and the chip's frequency offset compensation
 * is left to pull in the rest. At 433.8 MHz a worst-case 20 ppm + 20 ppm crystal
 * pair (the tolerances a.exp was generated for) is +-17.4 kHz, which is inside the
 * +-BW/4 = +-25 kHz FOC range the default bandwidth gives. The knob exists for the
 * case where a particular meter sits further out than that, or where FOC turns out
 * to mis-track and the window wants nudging by hand. */
static constexpr int32_t CC1101_DEFAULT_RX_OFFSET_HZ = 0;

/* +10 dBm is the top of the CC1101's 433 MHz PATABLE. A real D101 CIU transmits
 * +14 dBm (RF_PHY_SPEC.md sec. 4.9: CMT level 0x0E, regs CMT4=0x1C TX8=0x56 TX9=0x0B -
 * which is exactly what Cmt2300aHal::apply_runtime_overrides() writes), so the
 * uplink here is about 4 dB short of the reference link budget. That is a real but
 * modest handicap - the sibling RF433-2 projects face ~10 dB against a +20 dBm CIU. */
static constexpr int8_t CC1101_DEFAULT_OUTPUT_POWER_DBM = 10;

/* 101562 Hz = CHANBW_E 3, CHANBW_M 0 - the chip's narrowest setting above 100 kHz.
 *
 * It is NOT chosen for the signal: Carson bandwidth here is only 2*(6.0 + 2.4) =
 * ~17 kHz, so even the chip's 58035 Hz floor (E=3, M=3) would pass the reply with
 * room to spare. It is chosen for the OFFSET budget. The CC1101's frequency offset
 * compensation reaches +-BW/4, and at 433.8 MHz a worst-case 20 ppm + 20 ppm
 * crystal pair - the tolerances the real CIU's RFPDK profile was generated for - is
 * +-17.4 kHz. The 58 kHz floor gives +-14.5 kHz and does not cover that; 101562 Hz
 * gives +-25.4 kHz and does. It also matches the "~100 kHz filter + AFC" assumption
 * this component was already written around for the CMT2300A.
 *
 * The price of the wider filter is noise bandwidth: ~2.4 dB of sensitivity against
 * the 58 kHz setting. On a link where the uplink is already the weak direction that
 * is the right trade, but a bring-up where the meter is close and the replies are
 * strong can try `bandwidth: 58035Hz` and compare CRC-valid reply rates.
 *
 * 101562 > CC1101_TEST_REG_BW_THRESHOLD_HZ by 562 Hz, so solve_params_() picks the
 * WIDE TEST2/TEST1 pair (0x88/0x31). That is the correct, characterised pair for
 * this filter - the threshold is TI's, not ours. */
static constexpr uint32_t CC1101_DEFAULT_RX_BANDWIDTH_HZ = 101562u;

/* +-6.0 kHz, which the chip rounds to 5950 Hz (DEV_E = 1, DEV_M = 7 -> DEVIATN =
 * 0x17). SDR-measured: the two FSK tones are 11.9 kHz apart (RF_PHY_SPEC.md sec. 4.2),
 * and a.exp's header says "Deviation = 6.0 kHz" outright. h = 2*6000/2400 ~= 5.
 *
 * BEWARE OF THE SPEC'S REGISTER HINT. RF_PHY_SPEC.md sec. 4.2 and sec. 5 both say
 * "DEVIATN ~= 0x35 (DEVIATION_E = 1, DEVIATION_M = 5 -> ~5.93 kHz)". Every part of
 * that parenthesis is wrong and they are wrong in different directions:
 *   - 0x35 encodes E = 3, M = 5, which is (8+5) * 2^3 * Fxosc/2^17 = 20629 Hz -
 *     three and a half times the measured deviation;
 *   - E = 1, M = 5 is (8+5) * 2 * Fxosc/2^17 = 5157 Hz, not 5.93 kHz;
 *   - the 5.93 kHz figure actually belongs to E = 1, M = 7 = 5950.9 Hz, i.e. 0x17.
 * The driver does not use 0x35 or any other hard-coded byte: solve_params_()
 * searches all 64 (E, M) pairs for the one nearest the requested Hz and lands on
 * 0x17 by itself. Do not "correct" the solver back to the spec's hint. */
static constexpr uint32_t CC1101_DEFAULT_DEVIATION_HZ = 6000u;

/* ================================================================
 * PATABLE - 433 MHz optimum settings (CC1101 datasheet, Table 39)
 * ================================================================ */

/* These are the TI-characterised PA settings for the 433 MHz band; using the 868 or
 * 915 MHz column here would give both the wrong power and worse spectral purity.
 * The table is sparse, so set_output_power_dbm() snaps to the nearest entry and
 * dump_config() reports what was actually programmed. */
struct Cc1101PaSetting {
  int8_t dbm;
  uint8_t pa;
};

/* `inline` rather than `static`: the lookup below is an inline function that takes
 * the table's address, so the table has to be one entity across translation units. */
inline constexpr Cc1101PaSetting CC1101_PA_TABLE_433[] = {
    {-30, 0x12}, {-20, 0x0E}, {-15, 0x1D}, {-10, 0x34}, {0, 0x60}, {5, 0x84}, {7, 0xC8}, {10, 0xC0},
};
inline constexpr size_t CC1101_PA_TABLE_433_SIZE = sizeof(CC1101_PA_TABLE_433) / sizeof(CC1101_PA_TABLE_433[0]);

/// Nearest 433 MHz PATABLE entry to `dbm`; writes back the dBm actually selected.
inline uint8_t cc1101_pa_setting_for_dbm(int8_t dbm, int8_t *actual_dbm) {
  size_t best = 0;
  int best_err = INT16_MAX;
  for (size_t i = 0; i < CC1101_PA_TABLE_433_SIZE; i++) {
    const int err = (CC1101_PA_TABLE_433[i].dbm > dbm) ? (CC1101_PA_TABLE_433[i].dbm - dbm)
                                                       : (dbm - CC1101_PA_TABLE_433[i].dbm);
    if (err < best_err) {
      best_err = err;
      best = i;
    }
  }
  if (actual_dbm != nullptr) {
    *actual_dbm = CC1101_PA_TABLE_433[best].dbm;
  }
  return CC1101_PA_TABLE_433[best].pa;
}

/// The dBm a real D101 CIU transmits at (RF_PHY_SPEC.md sec. 4.9). Kept as a constant so
/// the boot log can state the deficit rather than a hard-coded "4 dB".
static constexpr int8_t CC1101_CIU_REFERENCE_POWER_DBM = 14;

/* ================================================================
 * The fixed part of the register profile
 * ================================================================ */

/* Only registers whose value does not depend on a tunable live here. FREQ2/1/0,
 * MDMCFG4 (which carries CHANBW_E/M alongside DRATE_E), MDMCFG2 (modulation is a
 * tunable), DEVIATN, FOCCFG (the carrier-sense gate is a tunable) and TEST2/TEST1
 * (keyed on the RX bandwidth) are computed in Cc1101Hal::write_config_() and written
 * straight afterwards, so a value appearing in this table is one nothing else can
 * override.
 *
 * The baseline is the SmartRF Studio export for "433 MHz, 2-FSK, 2.4 kbps", which
 * is the configuration TI characterises for exactly this symbol rate; the entries
 * that differ from it are the ones RF433-1 forces, and each says why. */
struct Cc1101RegInit {
  uint8_t addr;
  uint8_t value;
};

static constexpr Cc1101RegInit CC1101_CONFIG[] = {
    // GDO2 is not wired on any board this supports; 0x2E is the high-impedance
    // setting, which keeps it from fighting anything on the module.
    {CC1101_REG_IOCFG2, 0x2E},
    // GDO1 IS the MISO pin. 0x2E (high impedance) is the reset value and the only
    // one that leaves SPI reads working - never program this to a signal source.
    {CC1101_REG_IOCFG1, 0x2E},
    // GDO0 = "asserts when sync word has been sent or received, de-asserts at the
    // end of the packet". In fixed-length TX that falling edge is an exact TX-done
    // marker - and it stays exact through the infinite-to-fixed transition, because
    // the packet only ever ENDS in fixed mode. It is why gdo0 is worth wiring even
    // though the driver polls MARCSTATE when it is absent.
    {CC1101_REG_IOCFG0, 0x06},
    // FIFO_THR = 7 -> 33 bytes in TX / 32 bytes in RX. transmit() mirrors the same
    // split in software (refill once the TX FIFO has drained to <= 32 bytes), so
    // the threshold and the refill low-water mark cannot drift apart. The RX half is
    // inert here: this backend counts RXBYTES over SPI rather than watching a
    // threshold line, which is what lets rx_chunk_size() be 1.
    {CC1101_REG_FIFOTHR, 0x07},
    // SYNC1/SYNC0 are deliberately NOT here: the sync word is a YAML tunable
    // (`sync_word`), so it is written from Cc1101Hal::write_config_() alongside the
    // other solved values. See the long sync-word note above for why a value that
    // is derived rather than SDR-confirmed is worth making settable.
    // Rewritten per transmit(); 0xFF is simply a harmless resting value, since RX
    // runs in infinite-length mode where PKTLEN is not consulted.
    {CC1101_REG_PKTLEN, 0xFF},
    // PQT = 0 (no preamble-quality gate), no address check - RF433-1 frames carry
    // the CIU's own address and there is no recipient field, so hardware address
    // filtering has nothing to filter on (RF_PHY_SPEC.md sec. 6) - no CRC autoflush
    // and, importantly, APPEND_STATUS = 0. Infinite-length mode never ends a packet,
    // so the two status bytes would never be appended anyway, and leaving the bit
    // set would corrupt the stream the moment the mode changed.
    {CC1101_REG_PKTCTRL1, 0x00},
    // Resting state is the RX shape: infinite length, no whitening, no PHY CRC.
    {CC1101_REG_PKTCTRL0, CC1101_PKTCTRL0_RX},
    {CC1101_REG_ADDR, 0x00},
    // The carrier is programmed into FREQ2/1/0 directly from set_frequency() and
    // set_rx_frequency(), so the channel-number offset must contribute nothing.
    {CC1101_REG_CHANNR, 0x00},
    // FREQ_IF = 6 -> 6 * Fxosc / 2^10 = 152.34 kHz (CC1101_IF_HZ). The IF has to sit
    // above half the channel bandwidth; 152 kHz clears 101.56/2 = 50.8 kHz with room
    // to spare and is what SmartRF picks for any bandwidth up to ~100 kHz.
    // solve_params_() warns if a configured bandwidth outgrows it.
    {CC1101_REG_FSCTRL1, CC1101_FREQ_IF},
    // No static frequency offset - the RX offset knob moves the carrier word itself,
    // which keeps the two mechanisms from stacking.
    {CC1101_REG_FSCTRL0, 0x00},
    {CC1101_REG_MDMCFG3, CC1101_DRATE_M},
    // MDMCFG2 is deliberately NOT here: MOD_FORMAT is the `modulation` tunable, so
    // the register is assembled in Cc1101Hal::write_config_().
    // FEC_EN = 0 (a.exp: "FEC = Disable"), NUM_PREAMBLE = 101 = 12 bytes,
    // CHANSPC_E = 2. Twelve is EXACT here, not a safe-side rounding of the chip's
    // 8-or-12 choice: the real CIU sends 10 preamble bytes (a.exp "Preamble Tx
    // Size = 10") and the first two bytes of its 55 55 F6 72 sync word are 0x55 as
    // well, so 12 bytes of 0x55 reach the air before the F6 72 marker. A CC1101 with
    // NUM_PREAMBLE = 12 and SYNC1/SYNC0 = F6 72 therefore puts
    // 55 x12 | F6 72 | LEN ... on the air, byte-identical to the CIU. (The meter's
    // own receiver only needs about 5 - a.exp "Preamble Rx Size = 5" - so there is
    // margin on top of the match.) Those 12 bytes cost 40 ms of airtime at 2400
    // baud, which is why the TX deadline is computed from CC1101_PREAMBLE_BYTES
    // rather than guessed. Channel spacing is inert because CHANNR = 0.
    {CC1101_REG_MDMCFG1, 0x52},
    {CC1101_REG_MDMCFG0, 0xF8},
    // RX_TIME = 111: no RX timeout. The component decides when an exchange has
    // failed; the radio must never drop out of RX on its own.
    {CC1101_REG_MCSM2, 0x07},
    // CCA_MODE = 00 (always transmit). The link is strict half-duplex request/reply
    // and the meter only speaks when spoken to, so clear-channel assessment could
    // only ever block a request we must send. RXOFF_MODE = 11 (stay in RX),
    // TXOFF_MODE = 00 (IDLE after the packet) - which is what makes MARCSTATE
    // returning to IDLE a reliable TX-done indication.
    {CC1101_REG_MCSM1, 0x0C},
    // FS_AUTOCAL = 01: calibrate on every IDLE -> RX/TX transition. transmit() and
    // begin_rx() reprogram FREQ between them - and on this protocol they program
    // DIFFERENT carriers, since the meter may reply on a channel we do not transmit
    // on - so an automatic recalibration on each turn is exactly what is needed.
    // PO_TIMEOUT = 10 (~150 us) is the datasheet's recommendation for a crystal that
    // is already running.
    {CC1101_REG_MCSM0, 0x18},
    // FOCCFG is deliberately NOT here either: FOC_BS_CS_GATE is a YAML tunable.
    // Bit synchronisation: 2Ki/3Kp before sync, halved after, BS_LIMIT = 00 (no data
    // rate offset compensation). The programmed 2398.97 baud is 0.04 % from nominal,
    // far inside what the loop pulls in without help.
    {CC1101_REG_BSCFG, 0x6C},
    // AGC, straight from the low-data-rate 2-FSK export: 33 dB magnitude target,
    // maximum LNA and DVGA gain available, LNA priority, medium hysteresis, 16 dB
    // filter. The reply is weak, so nothing here is allowed to cap the gain.
    {CC1101_REG_AGCCTRL2, 0x03},
    {CC1101_REG_AGCCTRL1, 0x40},
    {CC1101_REG_AGCCTRL0, 0x91},
    // Wake-on-radio is unused; RC_PD = 1 powers down the RC oscillator it needs.
    {CC1101_REG_WORCTRL, 0xFB},
    {CC1101_REG_FREND1, 0x56},
    // PA_POWER = 0, so the PA always uses PATABLE[0] - which is the single byte
    // set_output_power_dbm() programmes. Both 2-FSK and the CC1101's GFSK are
    // constant envelope, so one entry is all that is ever read whichever
    // `modulation` is selected.
    {CC1101_REG_FREND0, 0x10},
    // Calibration seeds for the 433 MHz band. FS_AUTOCAL overwrites FSCAL3<3:0>,
    // FSCAL2 and FSCAL1 on the first calibration, so these only have to be sane
    // starting points; FSCAL0 is band-dependent and is kept.
    {CC1101_REG_FSCAL3, 0xE9},
    {CC1101_REG_FSCAL2, 0x2A},
    {CC1101_REG_FSCAL1, 0x00},
    {CC1101_REG_FSCAL0, 0x1F},
    // TEST2/TEST1 are not here either: they are keyed on the RX filter bandwidth,
    // which is a YAML tunable, so solve_params_() picks the characterised pair and
    // write_config_() writes it.
    // VCO_SEL_CAL_EN = 0, the value for carriers below 861 MHz.
    {CC1101_REG_TEST0, 0x09},
};
static constexpr size_t CC1101_CONFIG_SIZE = sizeof(CC1101_CONFIG) / sizeof(CC1101_CONFIG[0]);

}  // namespace esphome::nartis_rf_meter
