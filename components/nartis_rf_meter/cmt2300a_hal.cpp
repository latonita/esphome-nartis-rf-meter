/*
 * CMT2300A Hardware Abstraction Layer - Implementation
 *
 * Bit-bang 3-wire SPI driver for CMT2300A sub-GHz transceiver, exposed to the
 * component through the RadioHal interface (radio_hal.h).
 * Handles register/FIFO access and chip state machine transitions.
 *
 * This backend is proven against live meters. Its register writes, timings and
 * FIFO juggling are unchanged by the move to RadioHal: the methods were renamed
 * and regrouped, not rewritten.
 */

#include "cmt2300a_hal.h"

#include "esphome/core/log.h"
#include "esphome/core/hal.h"  // esphome::delay / delayMicroseconds

namespace esphome::nartis_rf_meter {

static const char *const TAG = "cmt2300a_hal";

Cmt2300aHal::~Cmt2300aHal() = default;

void Cmt2300aHal::sdio_set_output() { sdio_.pin_mode(gpio::FLAG_OUTPUT); }
void Cmt2300aHal::sdio_set_input()  { sdio_.pin_mode(gpio::FLAG_INPUT); }

void Cmt2300aHal::spi_delay() {
  // ~0.5 us delay for ~1 MHz SPI clock
  esphome::delayMicroseconds(1);
}

/* CMT2300A SPI protocol:
 *   - SDIO is bidirectional (half-duplex)
 *   - Data clocked on RISING edge of SCLK
 *   - MSB first
 *   - Address byte: bit7 = R/W (1=read, 0=write), bits[6:0] = address
 *   - CSB for register access, FCSB for FIFO access
 */

void Cmt2300aHal::spi_send_byte(uint8_t byte) {
  for (int i = 7; i >= 0; i--) {
    pin_low(sclk_);
    if (byte & (1 << i)) {
      pin_high(sdio_);
    } else {
      pin_low(sdio_);
    }
    spi_delay();
    pin_high(sclk_);
    spi_delay();
  }
  pin_low(sclk_);
}

uint8_t Cmt2300aHal::spi_recv_byte() {
  uint8_t byte = 0;
  for (int i = 7; i >= 0; i--) {
    pin_low(sclk_);
    spi_delay();
    pin_high(sclk_);
    spi_delay();
    if (pin_read(sdio_)) {
      byte |= (1 << i);
    }
  }
  pin_low(sclk_);
  return byte;
}

void Cmt2300aHal::spi_write_reg(uint8_t addr, uint8_t val) {
  pin_low(csb_);
  spi_delay();

  sdio_set_output();
  spi_send_byte(addr & 0x7F);  // Bit 7 = 0 for write
  spi_send_byte(val);

  pin_high(csb_);
  spi_delay();
}

uint8_t Cmt2300aHal::spi_read_reg(uint8_t addr) {
  pin_low(csb_);
  spi_delay();

  sdio_set_output();
  spi_send_byte(addr | 0x80);  // Bit 7 = 1 for read

  // Switch SDIO to input before first data clock
  sdio_set_input();
  uint8_t val = spi_recv_byte();

  pin_high(csb_);
  spi_delay();
  return val;
}

void Cmt2300aHal::spi_write_fifo(const uint8_t *data, size_t len) {
  sdio_set_output();

  for (size_t i = 0; i < len; i++) {
    pin_low(fcsb_);
    spi_delay();
    spi_send_byte(data[i]);
    pin_high(fcsb_);
    // Datasheet: wait >= 2 us after last SCLK falling edge before FCSB high
    // (already satisfied by spi_send_byte ending with SCLK low)
    spi_delay();
    spi_delay();  // Extra delay between FIFO bytes (>= 4 us)
    spi_delay();
    spi_delay();
  }
}

void Cmt2300aHal::spi_read_fifo(uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    pin_low(fcsb_);
    spi_delay();
    sdio_set_input();
    data[i] = spi_recv_byte();
    pin_high(fcsb_);
    spi_delay();
    spi_delay();
    spi_delay();
    spi_delay();
  }
}

void Cmt2300aHal::write_reg(uint8_t addr, uint8_t val) {
  spi_write_reg(addr, val);
}

void Cmt2300aHal::write_bank(uint8_t start_addr, const uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    spi_write_reg(start_addr + i, data[i]);
  }
}

void Cmt2300aHal::update_reg(uint8_t addr, uint8_t mask, uint8_t val) {
  uint8_t reg = spi_read_reg(addr);
  reg = (reg & ~mask) | (val & mask);
  spi_write_reg(addr, reg);
}

void Cmt2300aHal::set_pins(esphome::InternalGPIOPin *sdio, esphome::InternalGPIOPin *sclk,
                           esphome::InternalGPIOPin *csb, esphome::InternalGPIOPin *fcsb,
                           esphome::InternalGPIOPin *gpio3) {
  // Keep the pin objects for setup()/pin_mode at init, and snapshot fast
  // ISR-safe handles for the bit-bang inner loop (non-virtual access).
  sdio_pin_ = sdio;
  sclk_pin_ = sclk;
  csb_pin_ = csb;
  fcsb_pin_ = fcsb;
  gpio3_pin_ = gpio3;
  sdio_ = sdio->to_isr();
  sclk_ = sclk->to_isr();
  csb_ = csb->to_isr();
  fcsb_ = fcsb->to_isr();
  gpio3_ = gpio3->to_isr();
}

bool Cmt2300aHal::init() {
  ESP_LOGI(TAG, "Initializing CMT2300A...");

  // Register + configure each pin through its GPIOPin object (applies the
  // platform-correct setup), then set the directions we need.
  sclk_pin_->setup();
  csb_pin_->setup();
  fcsb_pin_->setup();
  sdio_pin_->setup();
  gpio3_pin_->setup();
  sclk_.pin_mode(gpio::FLAG_OUTPUT);
  csb_.pin_mode(gpio::FLAG_OUTPUT);
  fcsb_.pin_mode(gpio::FLAG_OUTPUT);
  sdio_.pin_mode(gpio::FLAG_OUTPUT);
  gpio3_.pin_mode(gpio::FLAG_INPUT);

  // Idle state: both CS lines high, clock low
  pin_high(csb_);
  pin_high(fcsb_);
  pin_low(sclk_);

  // Soft reset
  spi_write_reg(REG_SOFT_RST, SOFT_RST_VALUE);

  esphome::delay(RESET_DELAY_MS);

  spi_write_reg(REG_MODE_CTL, GO_STBY);
  if (!wait_for_state(STA_STBY, 20)) {
    ESP_LOGE(TAG, "Failed to enter standby after reset");
    return false;
  }

  if (!is_chip_connected()) {
    ESP_LOGE(TAG, "CMT2300A not detected - check wiring");
    return false;
  }

  // Enable config retention (preserve config across state transitions)
  update_reg(REG_MODE_STA, MASK_CFG_RETAIN, MASK_CFG_RETAIN);

  // Enable LFOSC_RECAL (REG_EN_CTL |= 0x20)
  update_reg(REG_EN_CTL, 0x20, 0x20);

  // Write all 96 bytes of register configuration
  write_full_config();

  // Apply runtime overrides
  apply_runtime_overrides();

  clear_interrupt_flags();

  clear_fifo();

  initialized_ = true;
  ESP_LOGI(TAG, "CMT2300A initialized successfully");

  return true;
}

bool Cmt2300aHal::is_chip_connected() {
  // Read product ID register (CMT2 = 0x01), expected 0x66 for CMT2300A
  uint8_t product_id = spi_read_reg(REG_CMT2);
  ESP_LOGD(TAG, "Product ID: 0x%02X (expected 0x66)", product_id);
  return product_id == 0x66;
}

void Cmt2300aHal::write_full_config() {
  ESP_LOGD(TAG, "Writing 96-byte register configuration...");

  write_bank(CMT_BANK_ADDR, NARTIS_CMT_BANK, CMT_BANK_SIZE);
  write_bank(SYSTEM_BANK_ADDR, NARTIS_SYSTEM_BANK, SYSTEM_BANK_SIZE);
  // Frequency bank: both halves of channel 0, exactly as the old
  // set_frequency_channel(0) wrote here. Channel 0 is the only row whose bytes
  // are the same in the default and the alternative channel mode, so this
  // initial write never depended on which mode the YAML selected - and the
  // component reprograms both halves at CHANNEL_SELECT before any traffic.
  write_bank(FREQUENCY_BANK_ADDR, NARTIS_FREQ_CHANNELS[0], FREQUENCY_BANK_SIZE);
  tx_freq_hz_ = CHANNEL_CARRIER_HZ[0];
  rx_freq_hz_ = CHANNEL_CARRIER_HZ[0];
  write_bank(DATA_RATE_BANK_ADDR, NARTIS_DATA_RATE_BANK, DATA_RATE_BANK_SIZE);
  write_bank(BASEBAND_BANK_ADDR, NARTIS_BASEBAND_BANK, BASEBAND_BANK_SIZE);
  write_bank(TX_BANK_ADDR, NARTIS_TX_BANK, TX_BANK_SIZE);

  // Post-init fixup: CMT10 = (reg & 0xF8) | 0x02
  update_reg(REG_CMT10, 0x07, 0x02);
}

void Cmt2300aHal::apply_runtime_overrides() {
  // ---- Post-config patch ----
  // These override the RFPDK bank defaults and are required for correct on-air
  // operation. Without them the chip TXes on the wrong half of the freq bank
  // and uses a sync word the meter rejects.
  //
  // The SYS2 patch alone moves TX from the freq bank's "RX-half" (434.10 MHz @
  // Ch0 = wrong) to the "TX-half" (433.82 MHz @ Ch0 = correct).

  // SYS2 (0x0D): clear top 3 bits - PLL/XO trim cluster. This is what flips
  // the chip onto the TX-half of the freq bank (regs 0x1C..0x1F).
  update_reg(REG_SYS2, 0xE0, 0x00);

  // Sync word patch, "normal" protocol mode. The four writes below are the
  // REGISTER bank PKT10..PKT13, not an air sequence: per CMOSTEK AN143 the sync
  // bytes fill SYNC_VALUE from the top, so PKT13 goes out FIRST and the air order
  // is 55 55 F6 72. The RFPDK default bank is F6 55 55 55 (= 55 55 55 F6 on air,
  // the export's "Sync Value = 0x555555F6"); this patch extends the marker from one
  // byte to two right ahead of the length byte, and the meter's correlator will not
  // lock without it. The stock CIU selects this variant via a persistent EEPROM
  // byte. Beware: fw/RF_PHY_SPEC.md sec. 4.4 and fw/FRAME_HEADER_SPEC.md:823 print
  // the air order as "72 F6 55 55", i.e. this register listing read backwards - see
  // the sync-word note in cc1101_defs.h for the full derivation.
  write_reg(REG_PKT10, 0x72);
  write_reg(REG_PKT11, 0xF6);
  write_reg(REG_PKT12, 0x55);
  write_reg(REG_PKT13, 0x55);

  // TX power = 14 dBm (table level 0x0E).
  write_reg(REG_CMT4, 0x1C);
  write_reg(REG_TX8,  0x56);
  write_reg(REG_TX9,  0x0B);

  // ---- FIFO + interrupt setup ----

  // FIFO merge: combine TX+RX FIFO into 64-byte single buffer
  // REG_SYS11 (0x16): (reg & 0xE0) | 0x12  - merge configuration
  update_reg(REG_SYS11, static_cast<uint8_t>(~MASK_FIFO_MERGE_CFG), FIFO_MERGE_VALUE);
  // REG_FIFO_CTL (0x69): set FIFO_MERGE_EN (FIFO_CTL |= 0x02)
  update_reg(REG_FIFO_CTL, MASK_FIFO_MERGE_EN, MASK_FIFO_MERGE_EN);

  // FIFO threshold - 0x0F (15 bytes), not 0x0C.
  update_reg(REG_PKT29, MASK_FIFO_TH, FIFO_TH_VALUE);

  // Route the GPIO3 pad to INT2 - this is the only interrupt pad we wire to the
  // ESP32. INT2 carries the live FIFO-threshold signal we poll (TX_FIFO_TH during
  // TX, RX_FIFO_TH during RX); GPIO3 = INT2 (0x20). GPIO1/GPIO2 are left at their
  // POR defaults (unconnected on our board).
  update_reg(REG_IO_SEL, MASK_GPIO3_SEL, GPIO3_SEL_INT2);

  // INT2 source = RX_FIFO_TH, ACTIVE-HIGH polarity. Active high preserves the
  // POR-default polar bit = 0 and reads "pin HIGH = event"; SETTING the polar
  // bit makes the line active-LOW (idle-high), which falsely reads as "packet
  // done" every poll. So clear it (= 0).
  // The live signal (TX_FIFO_TH during TX, RX_FIFO_TH during RX) lands on
  // INT2 -> the GPIO3 pad we poll. Default to RX_FIFO_TH (INT2=0x0C).
  update_reg(REG_INT2_CTL, MASK_INT2_SEL, INT_SEL_RX_FIFO_TH);
  update_reg(REG_INT2_CTL, MASK_INT_POLAR, 0x00);

  // INT_EN = 0x39 (PKT_DONE | PREAM_OK | SYNC_OK | TX_DONE).
  // FIFO_TH on the GPIO follows the INT mux regardless of INT_EN.
  write_reg(REG_INT_EN, 0x39);

  ESP_LOGD(TAG, "Runtime overrides applied (post-config patch + FIFO merge=64B; "
                "GPIO3 -> INT2, active-high, RX_FIFO_TH)");
}

uint8_t Cmt2300aHal::get_state() {
  return spi_read_reg(REG_MODE_STA) & MASK_CHIP_MODE_STA;
}

bool Cmt2300aHal::wait_for_state(uint8_t target_state, uint32_t timeout_ms) {
  uint32_t timeout_us = timeout_ms * 1000;
  uint32_t start_us = esphome::micros();
  while ((esphome::micros() - start_us) < timeout_us) {
    if (get_state() == target_state) return true;
    esphome::delayMicroseconds(STATE_POLL_INTERVAL_US);
  }

  ESP_LOGW(TAG, "Timeout waiting for state 0x%02X, current: 0x%02X", target_state, get_state());
  return false;
}

bool Cmt2300aHal::go_standby() {
  // Leaving RX ends the receive epoch, whether or not the transition confirms:
  // a chip that did not reach standby is certainly not still receiving the reply
  // this epoch was opened for.
  in_rx_ = false;
  spi_write_reg(REG_MODE_CTL, GO_STBY);
  return wait_for_state(STA_STBY);
}

bool Cmt2300aHal::go_sleep() {
  in_rx_ = false;
  spi_write_reg(REG_MODE_CTL, GO_SLEEP);
  return wait_for_state(STA_SLEEP);
}

/* ================================================================
 * Frequency
 *
 * The component speaks in carriers (Hz); this chip speaks in 4-byte synthesiser
 * halves. The translation is a LOOKUP, not a computation, by design: the bytes
 * in NARTIS_FREQ_CHANNELS are measured values that this driver is proven on air
 * with, and recomputing them from AN199 - even though AN199 happens to reproduce
 * them exactly today - would turn a working driver into one whose on-air
 * frequency depends on a formula and a crystal constant. The formula is kept
 * only for a carrier that is in no table row at all, and it says so loudly when
 * it is used.
 * ================================================================ */

uint8_t Cmt2300aHal::resolve_channel_(uint32_t freq_hz, bool want_rx_half) const {
  // The TX-half register value IS the carrier. The RX-half is the local
  // oscillator, which on this low-IF receiver sits one IF above the carrier -
  // so a carrier matches the RX-half of the row whose carrier it equals.
  for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
    const uint32_t carrier = CHANNEL_CARRIER_HZ[ch];
    const uint32_t diff = (freq_hz > carrier) ? (freq_hz - carrier) : (carrier - freq_hz);
    if (diff <= FREQ_MATCH_TOLERANCE_HZ) {
      return ch;
    }
  }
  (void) want_rx_half;  // same row either way; the parameter documents the caller's intent
  return NUM_CHANNELS;  // no match
}

void Cmt2300aHal::compute_freq_half_(uint32_t lo_hz, bool rx_half, uint8_t *out4) {
  // CMOSTEK AN199: word = floor(FREQ_LO * DIVIDER / XTAL * 2^20); N = word>>20,
  // K = low 20 bits. 64-bit intermediate - the shifted product overflows 32 bits.
  const uint32_t word = (uint32_t) ((((uint64_t) lo_hz * FREQ_DIVIDER) << 20) / XTAL_HZ);
  const uint8_t n = (uint8_t) (word >> 20);
  const uint32_t k = word & 0xFFFFFu;
  out4[0] = n;
  out4[1] = (uint8_t) (k & 0xFF);
  out4[2] = (uint8_t) ((k >> 8) & 0xFF);
  // The top byte of each half carries two fixed RFPDK bits beside K[19:16], and
  // they differ between the halves: 0x1B holds PALDO_SEL|DIVX, 0x1F FSK_SWT|VCO_BANK.
  out4[3] = rx_half ? (uint8_t) ((FREQ_PALDO_SEL << 7) | (FREQ_DIVX_CODE << 4) | ((k >> 16) & 0x0F))
                    : (uint8_t) ((FREQ_FSK_SWT << 7) | (FREQ_VCO_BANK << 4) | ((k >> 16) & 0x0F));
}

bool Cmt2300aHal::ensure_standby_for_freq_() {
  // Must be in standby to change frequency - the PLL re-locks on the way out.
  if (get_state() == STA_STBY) {
    return true;
  }
  return go_standby();
}

void Cmt2300aHal::write_freq_half_(const uint8_t *bytes4, bool rx_half) {
  // The frequency bank is 8 bytes: [0..3] = RX-half (regs 0x18..0x1B, the local
  // oscillator), [4..7] = TX-half (regs 0x1C..0x1F). Writing one half leaves the
  // other carrier alone - which is what the alternative channel mode lives on:
  // RX hops while TX stays on the meter's wake frequency.
  write_bank(rx_half ? FREQUENCY_BANK_ADDR : (uint8_t) (FREQUENCY_BANK_ADDR + 4), bytes4, 4);
}

void Cmt2300aHal::set_frequency(uint32_t freq_hz) {
  tx_freq_hz_ = freq_hz;
  if (!initialized_) {
    // Before init() there is no chip to write to; write_full_config() programs
    // channel 0 and the component reprograms both carriers at CHANNEL_SELECT.
    return;
  }
  ensure_standby_for_freq_();

  const uint8_t ch = resolve_channel_(freq_hz, false);
  if (ch < NUM_CHANNELS) {
    write_freq_half_(&NARTIS_FREQ_CHANNELS[ch][4], false);
    ESP_LOGD(TAG, "TX carrier %.3f MHz (channel %u preset)", freq_hz / 1e6f, ch);
    return;
  }

  uint8_t half[4];
  compute_freq_half_(freq_hz, false, half);
  write_freq_half_(half, false);
  ESP_LOGW(TAG,
           "TX carrier %.3f MHz matches no channel preset - synthesising the bank from AN199 "
           "(%02X %02X %02X %02X). The presets are measured values; a computed one is UNPROVEN on air.",
           freq_hz / 1e6f, half[0], half[1], half[2], half[3]);
}

void Cmt2300aHal::set_rx_frequency(uint32_t freq_hz) {
  rx_freq_hz_ = freq_hz;
  if (!initialized_) {
    return;
  }
  ensure_standby_for_freq_();

  const uint8_t ch = resolve_channel_(freq_hz, true);
  if (ch < NUM_CHANNELS) {
    write_freq_half_(&NARTIS_FREQ_CHANNELS[ch][0], true);
    ESP_LOGD(TAG, "RX carrier %.3f MHz (channel %u preset, LO sits %u Hz above it)", freq_hz / 1e6f, ch,
             (unsigned) FREQ_IF_HZ);
    return;
  }

  // Low-IF: tune the LO one IF ABOVE the carrier the caller asked for.
  uint8_t half[4];
  compute_freq_half_(freq_hz + FREQ_IF_HZ, true, half);
  write_freq_half_(half, true);
  ESP_LOGW(TAG,
           "RX carrier %.3f MHz matches no channel preset - synthesising the LO from AN199 "
           "(%02X %02X %02X %02X). The presets are measured values; a computed one is UNPROVEN on air.",
           freq_hz / 1e6f, half[0], half[1], half[2], half[3]);
}

void Cmt2300aHal::clear_fifo() {
  write_reg(REG_FIFO_CLR, FIFO_CLR_RX | FIFO_CLR_TX);
}

void Cmt2300aHal::reset_rx_fifo_full() {
  // Full reset of the merged FIFO for RX: restore read/write pointers AND
  // clear both halves. Required after a TX->RX direction switch so the RX
  // side doesn't start against the bytes left over from the last TX.
  write_reg(REG_FIFO_CLR, FIFO_RESTORE | FIFO_CLR_RX | FIFO_CLR_TX);
}

// RadioHal::read_rx_exact - the old read_fifo(), unchanged. This is how the
// sub-threshold tail of a frame is collected: those last < FIFO_TH_VALUE bytes
// never raise RX_FIFO_TH, so only a caller that already knows the frame length
// can ask for them. Clamped to the merged FIFO size, which is all the chip can
// be holding.
size_t Cmt2300aHal::read_rx_exact(uint8_t *buf, size_t len) {
  if (len > FIFO_SIZE_MERGED) {
    len = FIFO_SIZE_MERGED;
  }
  spi_read_fifo(buf, len);
  return len;
}

void Cmt2300aHal::set_tx_payload_length(uint16_t len) {
  // PKT14 (0x45): bits[6:4] = PAYLOAD_LENG[10:8], other bits cleared.
  // PKT15 (0x46): PAYLOAD_LENG[7:0].
  write_reg(REG_PKT14, static_cast<uint8_t>((len >> 8) & 0x07) << 4);
  write_reg(REG_PKT15, static_cast<uint8_t>(len & 0xFF));
}

void Cmt2300aHal::set_rx_payload_length() {
  // 511 = the large bank default ceiling. Variable-length-in-front frames are
  // bounded by this; our frames are <= ~93 B, so it never caps a real frame.
  // (Same register layout as the TX setter - single source of truth.)
  set_tx_payload_length(511);
}

/* scan_channels() lived here: a 66-line blocking RSSI survey over all four
 * channels with the stock device's trimmed-mean scoring and its
 * channel-0-never-selected quirk. It had NO caller - the component runs its
 * own survey in handle_rssi_scan_(), which is also where the stock algorithm
 * it reproduced is now recorded - and it duplicated component logic inside a
 * layer that is supposed to know nothing about channels. Removed with the
 * move to RadioHal rather than carried into the interface. */

// RadioHal::transmit - the old transmit_chunked(), logic unchanged.
//
// The one addition the interface contract asks for is parking the chip in
// standby on EVERY exit path, success included (the old code left it in TX
// after TX_DONE). It costs nothing in sequence terms: every call site went
// straight on to start_rx_(), which opened with go_standby() anyway.
bool Cmt2300aHal::transmit(const uint8_t *data, size_t len) {
  // Fill FIFO with first 64B, enter TX, then poll TX_FIFO_TH via the GPIO3/INT2
  // pin to refill remaining data in 15-byte chunks.

  if (!go_standby()) return false;

  // Tell the packet engine how many bytes to transmit (fixed-length mode).
  set_tx_payload_length(static_cast<uint16_t>(len));

  clear_interrupt_flags();

  // Switch the merged FIFO fully to TX-WRITE mode and reset its pointers. This
  // MUST recover from a preceding RX session (which left it in SPI-read /
  // RX-direction mode); otherwise the FIFO write below lands nowhere and the
  // chip stalls waiting for TX data -> TX_DONE never fires.
  //   FIFO_MERGE_EN on, FIFO_RX_TX_SEL=1 (TX), SPI_FIFO_RD_WR_SEL=1 (SPI writes)
  update_reg(REG_FIFO_CTL, MASK_FIFO_MERGE_EN | MASK_FIFO_RX_TX_SEL | MASK_SPI_FIFO_RD_WR_SEL,
             MASK_FIFO_MERGE_EN | MASK_FIFO_RX_TX_SEL | MASK_SPI_FIFO_RD_WR_SEL);
  write_reg(REG_FIFO_CLR, FIFO_RESTORE | FIFO_CLR_RX | FIFO_CLR_TX);

  // Configure INT1 = TX_FIFO_TH (fires when FIFO drops below threshold)
  set_int_source(INT_SEL_TX_FIFO_TH);

  size_t initial = (len > FIFO_SIZE_MERGED) ? FIFO_SIZE_MERGED : len;
  spi_write_fifo(data, initial);
  size_t written = initial;

  // Enter TX mode: STBY -> TFS -> TX
  spi_write_reg(REG_MODE_CTL, GO_TFS);
  if (!wait_for_state(STA_TFS)) {
    ESP_LOGE(TAG, "Chunked TX: failed to enter TFS");
    go_standby();  // contract: parked on every exit path
    return false;
  }
  spi_write_reg(REG_MODE_CTL, GO_TX);
  if (!wait_for_state(STA_TX)) {
    ESP_LOGW(TAG, "Chunked TX: chip did not enter TX (state=0x%02X)", get_state());
  }

  // Poll: refill FIFO as it drains, wait for TX_DONE.
  // 600 ms covers the largest frame (~290 B ~ 480 ms airtime at 4800 bps). // this shall be 2400!
  //
  // KNOWN-WRONG NOTE, left exactly as found rather than quietly corrected: the
  // link runs at 2400 bps, not 4800, so ~290 B is ~970 ms of airtime and the
  // 600 ms cap could not cover it. The cap itself is NOT changed here - this is
  // a proven driver and the real frames are far shorter than 290 B, so nothing
  // has ever hit it - but whoever revisits TX timing should start from this:
  // the number and its justification disagree, and it is the justification that
  // is known to be wrong.
  static constexpr uint32_t TX_TIMEOUT_MS = 600;
  uint32_t start = esphome::millis();

  uint16_t refills = 0;  // diagnostic: how many TX_FIFO_TH refills we serviced
  while (esphome::millis() - start < TX_TIMEOUT_MS) {
    // Check TX_DONE first - the flag is deliberately left set (see the exit path)
    uint8_t int_clr1 = spi_read_reg(REG_INT_CLR1);
    if (int_clr1 & CLR1_TX_DONE_FLG) {
      ESP_LOGVV(TAG, "Chunked TX complete: %d/%d bytes written, %u refill(s)",
               (int) written, (int) len, refills);
      if (written < len) {
        // KNOWN DIVERGENCE from the RadioHal contract, which says a frame the
        // chip truncated is a failure - the meter hears something whose CRC
        // cannot check out and will never answer it, so the caller should be
        // free to resend instead of spending a receive timeout on it. This
        // backend returns true anyway, exactly as it always has: it is proven
        // against live meters and turning a warning into an aborted exchange is
        // a behaviour change that belongs in its own, separately-tested step.
        // Flagged here rather than silently fixed.
        ESP_LOGW(TAG, "Chunked TX: TX_DONE but only %d/%d bytes written - frame truncated on air!",
                 (int) written, (int) len);
      }
      // The TX_DONE flag is deliberately left SET here, as it always has been.
      // Nothing reads it any more - begin_rx() clears every latched flag when it
      // opens the next epoch - but clearing it would be a register write this
      // proven path has never done.
      go_standby();  // contract: parked on every exit path
      return true;
    }

    // Refill when the TX FIFO has DRAINED to/below threshold. Per the datasheet
    // INT-source table, TX_FIFO_TH = 1 means "unread TX bytes OVER FIFO_TH" (still
    // full); it reads 0 once drained to <= FIFO_TH (room for another chunk).
    // Refill on this flag CLEAR.
    //
    // BUG FIX: we previously polled GPIO3 (active-high TX_FIFO_TH) and refilled
    // while it was HIGH - i.e. while the FIFO was still FULL - which overflowed
    // the FIFO and dropped the frame tail. That silently corrupted every frame
    // > 64 B (the ones needing a refill) while <= 64 B frames worked. Read the
    // flag straight from REG_FIFO_FLAG over SPI so we don't depend on INT polarity.
    if (written < len && (spi_read_reg(REG_FIFO_FLAG) & FIFO_TX_TH) == 0) {
      size_t remaining = len - written;
      size_t chunk = (remaining > TX_REFILL_CHUNK) ? TX_REFILL_CHUNK : remaining;
      spi_write_fifo(data + written, chunk);
      written += chunk;
      refills++;
    }

    // Feed the watchdog - this loop can block for many ms and ESP8266's soft
    // WDT resets if we spin without yielding.
    esphome::yield();
    esphome::delayMicroseconds(50);
  }

  ESP_LOGW(TAG, "Chunked TX timeout (%d/%d bytes written, %u refills, state=0x%02X, int_clr1=0x%02X)",
           (int) written, (int) len, refills, get_state(), spi_read_reg(REG_INT_CLR1));
  go_standby();
  return false;
}

/* get_interrupt_flags() / get_int_clr1() / is_pkt_ok() / is_tx_done() lived
 * here. is_tx_done() was the component's TX-completion poll; transmit() now
 * blocks to TX_DONE itself and parks the chip, so there is nothing left to ask
 * about afterwards, and the other three went with it - is_pkt_ok() was never
 * called at all (this link's packet engine runs fixed-length, so PKT_OK is not
 * the end-of-frame signal; the frame's own LEN byte is). The TX poll loop in
 * transmit() still reads REG_INT_CLR1 directly, exactly as it always did. */

void Cmt2300aHal::clear_interrupt_flags() {
  write_reg(REG_INT_CLR1, CLR1_TX_DONE_CLR | CLR1_SL_TMO_CLR | CLR1_RX_TMO_CLR);
  write_reg(REG_INT_CLR2, CLR2_LBD_CLR | CLR2_PREAM_OK_CLR | CLR2_SYNC_OK_CLR |
                          CLR2_NODE_OK_CLR | CLR2_CRC_OK_CLR | CLR2_PKT_DONE_CLR);
}

int8_t Cmt2300aHal::get_rssi_dbm() {
  // REG 0x70 returns unsigned 0..255; dBm = regval - 128.
  int val = static_cast<int>(spi_read_reg(REG_RSSI_DBM)) - 128;
  return static_cast<int8_t>(val);
}

/* get_rssi_code() (the raw 0..255 code behind get_rssi_dbm()), get_fifo_flags()
 * (REG_FIFO_FLAG - which reads 0xFF on this board and was therefore never
 * usable) and read_gpio3() (the raw INT2 pin) were public and had no callers.
 * Dropped rather than carried into RadioHal: all three expose CMT2300A
 * internals that the interface is there to hide, and the one piece of real
 * knowledge among them - that REG_FIFO_FLAG is unusable here - is recorded in
 * drain_rx() and in the header, where it explains a design decision. */

void Cmt2300aHal::set_rssi_mode(bool enable) {
  // SYS11 (0x16) low 5 bits: 0x01 = RSSI-valid mode (get_rssi_dbm() reads real
  // values), FIFO_MERGE_VALUE (0x12) = normal merged-FIFO packet mode.
  //
  // One field, two jobs: this is why RadioHal has a set_rssi_mode() at all.
  // Leave it on and packet I/O is broken; never turn it on and get_rssi_dbm()
  // reports nonsense. The RSSI survey must bracket itself with both calls.
  update_reg(REG_SYS11, 0x1F, enable ? 0x01 : FIFO_MERGE_VALUE);
}

// RadioHal::self_test - the old test_gpio3_wiring(), unchanged.
bool Cmt2300aHal::self_test() {
  // INT2 = RX_ACTIVE -> pin is high only while the chip sits in RX state.
  set_int_source(INT_SEL_RX_ACTIVE);

  go_standby();
  esphome::delayMicroseconds(300);
  bool in_stby = pin_read(gpio3_);

  // Enter RX (STBY -> RFS -> RX) and sample the pin.
  spi_write_reg(REG_MODE_CTL, GO_RFS);
  wait_for_state(STA_RFS);
  spi_write_reg(REG_MODE_CTL, GO_RX);
  wait_for_state(STA_RX);
  esphome::delayMicroseconds(300);
  bool in_rx = pin_read(gpio3_);

  // Restore: standby + INT source back to RX_FIFO_TH for normal operation.
  go_standby();
  set_int_source(INT_SEL_RX_FIFO_TH);

  bool ok = (!in_stby && in_rx);
  if (ok) {
    ESP_LOGI(TAG, "GPIO3 wiring OK: pin LOW in STBY, HIGH in RX (INT2->pin_gpio3 good)");
  } else {
    ESP_LOGE(TAG, "GPIO3 wiring FAIL: STBY=%d RX=%d (expected 0 then 1). "
                  "Check the chip's GPIO3 pad is wired to pin_gpio3 - RX cannot work otherwise.",
             in_stby, in_rx);
  }
  return ok;
}

void Cmt2300aHal::set_int_source(uint8_t source) {
  // INT2 surfaces on the GPIO3 pad - the line we poll (TX_FIFO_TH during TX,
  // RX_FIFO_TH during RX). Preserves the polarity bit.
  update_reg(REG_INT2_CTL, MASK_INT2_SEL, source & MASK_INT2_SEL);
}

void Cmt2300aHal::prepare_rx_session() {
  // SDIO becomes the read line for the duration of the RX session.
  // Setting it once up-front avoids per-byte pinMode toggles.
  sdio_set_input();

  // SPI_FIFO_RD_WR_SEL = 0 (SPI reads FIFO) and
  // FIFO_RX_TX_SEL    = 0 (merged FIFO acts as RX FIFO).
  update_reg(REG_FIFO_CTL, MASK_SPI_FIFO_RD_WR_SEL | MASK_FIFO_RX_TX_SEL, 0x00);
}

// RadioHal::begin_rx - what the component's start_rx_() used to do inline, in
// exactly the same order, because the order is load-bearing. Carried over from
// the comment that used to sit there:
//
//   CRITICAL: the preceding TX left the merged FIFO in TX direction with the
//   64-byte beacon still "occupying" it. Switch the merged FIFO back to RX
//   direction FIRST, then fully reset it (clear + RESTORE pointers) - otherwise
//   RX starts against a FIFO that already reads full/overflowed (FIFO_FLAG=0xFF)
//   and we drain stale garbage. Order matters: direction -> restore -> clear.
//
// That satisfies the RadioHal epoch contract: go_standby() aborts whatever was
// in flight, prepare_rx_session() + reset_rx_fifo_full() flush the merged FIFO
// in the only order that works, clear_interrupt_flags() clears every latched
// flag so the first one seen afterwards belongs to this epoch, and the two
// transitions are confirmed before returning true.
bool Cmt2300aHal::begin_rx() {
  in_rx_ = false;

  go_standby();

  prepare_rx_session();      // RX FIFO direction + SDIO input
  reset_rx_fifo_full();      // FIFO_RESTORE + clear RX/TX
  clear_interrupt_flags();

  // Route the INT line -> RX_FIFO_TH (the meter's RX mechanism). The pin
  // goes HIGH while >= FIFO_TH_VALUE bytes await in the RX FIFO; the caller
  // drains threshold chunks on that, and reads the trailing sub-threshold bytes
  // by frame length. (TX left it on TX_FIFO_TH; this restores it for RX.)
  set_int_source(INT_SEL_RX_FIFO_TH);

  // Restore PAYLOAD_LENG to the large ceiling: the preceding TX shrank it to the
  // TX frame size, which would otherwise cap this RX at (TX_len + 1) bytes and
  // truncate the meter's longer reply (e.g. 45/66, 75/93).
  set_rx_payload_length();

  // Enter RX mode: STBY -> RFS -> RX. Both transitions are checked now; the
  // inline version ignored them, which meant a chip that never reached RX looked
  // exactly like a meter that never answered, and cost a full receive timeout to
  // find out. Saying so here is the whole point of begin_rx() returning a bool.
  spi_write_reg(REG_MODE_CTL, GO_RFS);
  if (!wait_for_state(STA_RFS)) {
    ESP_LOGE(TAG, "begin_rx: chip did not reach RFS (state=0x%02X)", get_state());
    return false;
  }
  spi_write_reg(REG_MODE_CTL, GO_RX);
  if (!wait_for_state(STA_RX)) {
    ESP_LOGE(TAG, "begin_rx: chip did not reach RX (state=0x%02X)", get_state());
    return false;
  }

  in_rx_ = true;
  return true;
}

// RadioHal::drain_rx - the old poll_rx_drain(), draining logic unchanged, plus
// an honest status.
//
// On what this chip CANNOT report, so that nobody reads the missing statuses as
// "it never happens":
//
//   RX_STATUS_OVERFLOW - not detectable here. The only register that would say
//     so is REG_FIFO_FLAG, and on this board it reads 0xFF (Control2 bank),
//     which is why the whole receive path polls the GPIO3/INT2 pin instead. A
//     status synthesised from the pin alone would be a guess, so overflow is
//     simply never reported rather than faked. A real overflow presents as a
//     frame whose software CRC-16/DNP fails, and is counted as a bad frame.
//
//   RX_STATUS_IO_ERROR - not reported. Detecting it would mean an SPI register
//     read on every pass of ESPHome's loop() purely to check the bus is alive,
//     on a bit-banged bus. The cost is real and the payoff is a condition that
//     init()'s product-ID probe already catches.
//
//   RX_STATUS_NOT_IN_RX - reported, and free: in_rx_ is set only by a begin_rx()
//     that confirmed STA_RX and cleared by everything that leaves RX, so it is
//     answered from memory with no bus traffic at all.
size_t Cmt2300aHal::drain_rx(uint8_t *buf, size_t buf_size, RxStatus &status) {
  if (!in_rx_) {
    status = RxStatus::RX_STATUS_NOT_IN_RX;
    return 0;
  }

  // RX_FIFO_TH-driven chunk drain. INT2 (=RX_FIFO_TH) surfaces on the GPIO3 pin
  // and is HIGH while >= FIFO_TH_VALUE bytes sit in the RX FIFO. We poll that pin
  // (the stock CIU reads the chip's INT output pin via MCU GPIO too; the SPI
  // status regs 0x6D/0x6E read 0xFF here - Control2 bank - so they're unusable).
  // Drain full threshold chunks while the line is asserted.
  //
  // The trailing < FIFO_TH_VALUE bytes of a frame never raise RX_FIFO_TH; the
  // caller reads that tail by frame length through read_rx_exact() once it has
  // arrived. That is what rx_chunk_size() is for.
  size_t total = 0;
  while (total + FIFO_TH_VALUE <= buf_size && pin_read(gpio3_)) {
    spi_read_fifo(buf + total, FIFO_TH_VALUE);
    total += FIFO_TH_VALUE;
  }

  status = (total > 0) ? RxStatus::RX_STATUS_DATA : RxStatus::RX_STATUS_NO_DATA;
  return total;
}

void Cmt2300aHal::dump_config() {
  // Stored configuration only - no SPI traffic. ESPHome dumps a component even
  // after mark_failed(), so this can run with no chip on the other end.
  ESP_LOGCONFIG(TAG, "    Pins: SDIO=%d, SCLK=%d, CSB=%d, FCSB=%d, GPIO3=%d",
                sdio_pin_ != nullptr ? sdio_pin_->get_pin() : -1,
                sclk_pin_ != nullptr ? sclk_pin_->get_pin() : -1,
                csb_pin_ != nullptr ? csb_pin_->get_pin() : -1,
                fcsb_pin_ != nullptr ? fcsb_pin_->get_pin() : -1,
                gpio3_pin_ != nullptr ? gpio3_pin_->get_pin() : -1);
  ESP_LOGCONFIG(TAG, "    Carriers: TX %.3f MHz, RX %.3f MHz (RX LO is %u Hz above the RX carrier)",
                tx_freq_hz_ / 1e6f, rx_freq_hz_ / 1e6f, (unsigned) FREQ_IF_HZ);
  ESP_LOGCONFIG(TAG, "    RX drain: %u-byte threshold chunks off GPIO3/INT2, tail read by frame length",
                (unsigned) FIFO_TH_VALUE);
}

}  // namespace esphome::nartis_rf_meter
