/*
 * CMT2300A Hardware Abstraction Layer — Implementation
 *
 * Bit-bang 3-wire SPI driver for CMT2300A sub-GHz transceiver.
 * Handles register/FIFO access and chip state machine transitions.
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

uint8_t Cmt2300aHal::read_reg(uint8_t addr) {
  return spi_read_reg(addr);
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
    ESP_LOGE(TAG, "CMT2300A not detected — check wiring");
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
  // Frequency bank written per-channel (default to CH0). Honor the custom-channel
  // flag so the initial bank matches what set_frequency_channel() will later write.
  set_frequency_channel(0);
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

  // SYS2 (0x0D): clear top 3 bits — PLL/XO trim cluster. This is what flips
  // the chip onto the TX-half of the freq bank (regs 0x1C..0x1F).
  update_reg(REG_SYS2, 0xE0, 0x00);

  // Sync word patch ("normal" mode = 0x72 F6 55 55). RFPDK default was
  // F6 55 55 55 — the meter's correlator won't lock on the wrong byte. The
  // stock CIU selects this variant via a persistent EEPROM byte.
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
  // REG_SYS11 (0x16): (reg & 0xE0) | 0x12  — merge configuration
  update_reg(REG_SYS11, static_cast<uint8_t>(~MASK_FIFO_MERGE_CFG), FIFO_MERGE_VALUE);
  // REG_FIFO_CTL (0x69): set FIFO_MERGE_EN (FIFO_CTL |= 0x02)
  update_reg(REG_FIFO_CTL, MASK_FIFO_MERGE_EN, MASK_FIFO_MERGE_EN);

  // FIFO threshold — 0x0F (15 bytes), not 0x0C.
  update_reg(REG_PKT29, MASK_FIFO_TH, FIFO_TH_VALUE);

  // Route the GPIO3 pad to INT2 — this is the only interrupt pad we wire to the
  // ESP32. INT2 carries the live FIFO-threshold signal we poll (TX_FIFO_TH during
  // TX, RX_FIFO_TH during RX); GPIO3 = INT2 (0x20). GPIO1/GPIO2 are left at their
  // POR defaults (unconnected on our board).
  update_reg(REG_IO_SEL, MASK_GPIO3_SEL, GPIO3_SEL_INT2);

  // INT2 source = RX_FIFO_TH, ACTIVE-HIGH polarity. Active high preserves the
  // POR-default polar bit = 0 and reads "pin HIGH = event"; SETTING the polar
  // bit makes the line active-LOW (idle-high), which falsely reads as "packet
  // done" every poll. So clear it (= 0).
  // The live signal (TX_FIFO_TH during TX, RX_FIFO_TH during RX) lands on
  // INT2 → the GPIO3 pad we poll. Default to RX_FIFO_TH (INT2=0x0C).
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
  spi_write_reg(REG_MODE_CTL, GO_STBY);
  return wait_for_state(STA_STBY);
}

bool Cmt2300aHal::go_sleep() {
  spi_write_reg(REG_MODE_CTL, GO_SLEEP);
  return wait_for_state(STA_SLEEP);
}

bool Cmt2300aHal::go_rx() {
  // Must go STBY → RFS → RX
  if (get_state() != STA_STBY) {
    if (!go_standby()) return false;
  }

  clear_rx_fifo();
  clear_interrupt_flags();

  spi_write_reg(REG_MODE_CTL, GO_RFS);
  if (!wait_for_state(STA_RFS)) {
    ESP_LOGE(TAG, "Failed to enter RFS");
    return false;
  }

  spi_write_reg(REG_MODE_CTL, GO_RX);
  if (!wait_for_state(STA_RX)) {
    ESP_LOGE(TAG, "Failed to enter RX");
    return false;
  }

  return true;
}

bool Cmt2300aHal::go_tx() {
  // Must go STBY → TFS → TX
  if (get_state() != STA_STBY) {
    if (!go_standby()) return false;
  }

  clear_interrupt_flags();

  spi_write_reg(REG_MODE_CTL, GO_TFS);
  if (!wait_for_state(STA_TFS)) {
    ESP_LOGE(TAG, "Failed to enter TFS");
    return false;
  }

  spi_write_reg(REG_MODE_CTL, GO_TX);
  if (!wait_for_state(STA_TX)) {
    ESP_LOGE(TAG, "Failed to enter TX");
    return false;
  }

  return true;
}

void Cmt2300aHal::set_frequency_channel(uint8_t ch) {
  if (ch >= NUM_CHANNELS) {
    ESP_LOGW(TAG, "Invalid channel %d (max %d)", ch, NUM_CHANNELS - 1);
    return;
  }

  // Must be in standby to change frequency
  bool was_not_standby = (get_state() != STA_STBY);
  if (was_not_standby) {
    go_standby();
  }

  const uint8_t (*channels)[8] = use_custom_channels_ ? NARTIS_CUSTOM_CHANNELS : NARTIS_FREQ_CHANNELS;
  write_bank(FREQUENCY_BANK_ADDR, channels[ch], FREQUENCY_BANK_SIZE);

  ESP_LOGD(TAG, "Set frequency channel %d (%s preset)", ch, use_custom_channels_ ? "custom" : "default");
}

void Cmt2300aHal::set_rx_channel(uint8_t ch) {
  if (ch >= NUM_CHANNELS) {
    ESP_LOGW(TAG, "Invalid RX channel %d (max %d)", ch, NUM_CHANNELS - 1);
    return;
  }
  bool was_not_standby = (get_state() != STA_STBY);
  if (was_not_standby) {
    go_standby();
  }

  // The frequency bank is 8 bytes: [0..3] = RX-half, [4..7] = TX-half. Write
  // only the 4 RX bytes (regs 0x18..0x1B) so the TX frequency (set earlier to
  // Ch0/433.82) is preserved — RX hops, TX stays on the meter's wake frequency.
  const uint8_t (*channels)[8] = use_custom_channels_ ? NARTIS_CUSTOM_CHANNELS : NARTIS_FREQ_CHANNELS;
  write_bank(FREQUENCY_BANK_ADDR, channels[ch], 4);

  ESP_LOGD(TAG, "Set RX channel %d (%s preset, RX-only — TX unchanged)",
           ch, use_custom_channels_ ? "custom" : "default");
}

void Cmt2300aHal::clear_fifo() {
  write_reg(REG_FIFO_CLR, FIFO_CLR_RX | FIFO_CLR_TX);
}

void Cmt2300aHal::clear_tx_fifo() {
  write_reg(REG_FIFO_CLR, FIFO_CLR_TX);
}

void Cmt2300aHal::clear_rx_fifo() {
  write_reg(REG_FIFO_CLR, FIFO_CLR_RX);
}

void Cmt2300aHal::reset_rx_fifo_full() {
  // Full reset of the merged FIFO for RX: restore read/write pointers AND
  // clear both halves. Required after a TX→RX direction switch so the RX
  // side doesn't start against the bytes left over from the last TX.
  write_reg(REG_FIFO_CLR, FIFO_RESTORE | FIFO_CLR_RX | FIFO_CLR_TX);
}

size_t Cmt2300aHal::write_fifo(const uint8_t *data, size_t len) {
  if (len > FIFO_SIZE_MERGED) {
    ESP_LOGW(TAG, "FIFO write truncated: %d > %d", (int) len, FIFO_SIZE_MERGED);
    len = FIFO_SIZE_MERGED;
  }
  spi_write_fifo(data, len);
  return len;
}

size_t Cmt2300aHal::read_fifo(uint8_t *data, size_t max_len) {
  if (max_len > FIFO_SIZE_MERGED) {
    max_len = FIFO_SIZE_MERGED;
  }
  spi_read_fifo(data, max_len);
  return max_len;
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
  // (Same register layout as the TX setter — single source of truth.)
  set_tx_payload_length(511);
}

uint8_t Cmt2300aHal::scan_channels(int8_t *out_score) {
  // Picks the quietest channel.
  // Per channel: enable RSSI mode → GO_RX → 2 ms settle → 1 initial sample,
  //              6 more samples with 2 ms between each, track running max & min,
  //              score = (sum_of_6 - max - min) / 4.
  // Channel selection (preserves the ch0-never-picked quirk):
  //   - best_ch starts at 0
  //   - ch1 always seeds best_ch=1, best_score=ch1_score
  //   - ch2/ch3 replace only if strictly less than current best
  //   - ch0's score is computed but never compared, so ch0 is unreachable

  int8_t best_score = 127;
  uint8_t best_ch = 0;

  for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
    go_standby();
    set_frequency_channel(ch);
    // Switch SYS11 to RSSI-valid mode: (SYS11 & 0xE0) | 0x01
    update_reg(REG_SYS11, 0x1F, 0x01);

    spi_write_reg(REG_MODE_CTL, GO_RX);
    if (!wait_for_state(STA_RX)) {
      ESP_LOGW(TAG, "scan_channels: failed to enter RX on channel %u", ch);
      if (out_score != nullptr) out_score[ch] = 127;
      continue;
    }

    esphome::delayMicroseconds(RSSI_SCAN_SAMPLE_DELAY_US);
    int8_t initial = get_rssi_dbm();
    int8_t max_seen = initial;
    int8_t min_seen = initial;
    int sum = 0;

    for (uint8_t i = 0; i < RSSI_SCAN_LOOP_SAMPLES; i++) {
      esphome::delayMicroseconds(RSSI_SCAN_SAMPLE_DELAY_US);
      int8_t s = get_rssi_dbm();
      sum += s;
      if (s > max_seen) max_seen = s;
      if (s < min_seen) min_seen = s;
    }

    int trimmed = (sum - max_seen - min_seen) / (RSSI_SCAN_LOOP_SAMPLES - 2);
    int8_t score = static_cast<int8_t>(trimmed);

    if (out_score != nullptr) out_score[ch] = score;
    ESP_LOGD(TAG, "scan_channels: ch%u score=%d dBm (min=%d max=%d)",
             ch, score, min_seen, max_seen);

    // Comparison logic — preserve the ch0-unreachable quirk.
    if (ch == 1) {
      best_ch = 1;
      best_score = score;
    } else if (ch > 1 && score < best_score) {
      best_ch = ch;
      best_score = score;
    }
  }

  go_standby();
  // Restore SYS11 to FIFO merge config: (SYS11 & 0xE0) | FIFO_MERGE_VALUE
  update_reg(REG_SYS11, 0x1F, FIFO_MERGE_VALUE);
  set_frequency_channel(best_ch);

  ESP_LOGI(TAG, "scan_channels: best channel %u (%d dBm)", best_ch, best_score);
  return best_ch;
}

bool Cmt2300aHal::transmit_chunked(const uint8_t *data, size_t len) {
  // Fill FIFO with first 64B, enter TX, then poll TX_FIFO_TH via the GPIO3/INT2
  // pin to refill remaining data in 15-byte chunks.

  if (!go_standby()) return false;

  // Tell the packet engine how many bytes to transmit (fixed-length mode).
  set_tx_payload_length(static_cast<uint16_t>(len));

  clear_interrupt_flags();

  // Switch the merged FIFO fully to TX-WRITE mode and reset its pointers. This
  // MUST recover from a preceding RX session (which left it in SPI-read /
  // RX-direction mode); otherwise the FIFO write below lands nowhere and the
  // chip stalls waiting for TX data → TX_DONE never fires.
  //   FIFO_MERGE_EN on, FIFO_RX_TX_SEL=1 (TX), SPI_FIFO_RD_WR_SEL=1 (SPI writes)
  update_reg(REG_FIFO_CTL, MASK_FIFO_MERGE_EN | MASK_FIFO_RX_TX_SEL | MASK_SPI_FIFO_RD_WR_SEL,
             MASK_FIFO_MERGE_EN | MASK_FIFO_RX_TX_SEL | MASK_SPI_FIFO_RD_WR_SEL);
  write_reg(REG_FIFO_CLR, FIFO_RESTORE | FIFO_CLR_RX | FIFO_CLR_TX);

  // Configure INT1 = TX_FIFO_TH (fires when FIFO drops below threshold)
  set_int_source(INT_SEL_TX_FIFO_TH);

  size_t initial = (len > FIFO_SIZE_MERGED) ? FIFO_SIZE_MERGED : len;
  spi_write_fifo(data, initial);
  size_t written = initial;

  // Enter TX mode: STBY → TFS → TX
  spi_write_reg(REG_MODE_CTL, GO_TFS);
  if (!wait_for_state(STA_TFS)) {
    ESP_LOGE(TAG, "Chunked TX: failed to enter TFS");
    return false;
  }
  spi_write_reg(REG_MODE_CTL, GO_TX);
  if (!wait_for_state(STA_TX)) {
    ESP_LOGW(TAG, "Chunked TX: chip did not enter TX (state=0x%02X)", get_state());
  }

  // Poll: refill FIFO as it drains, wait for TX_DONE.
  // 600 ms covers the largest frame (~290 B ≈ 480 ms airtime at 4800 bps). // this shall be 2400!
  static constexpr uint32_t TX_TIMEOUT_MS = 600;
  uint32_t start = esphome::millis();

  uint16_t refills = 0;  // diagnostic: how many TX_FIFO_TH refills we serviced
  while (esphome::millis() - start < TX_TIMEOUT_MS) {
    // Check TX_DONE first — leave flag set for caller's is_tx_done() check
    uint8_t int_clr1 = spi_read_reg(REG_INT_CLR1);
    if (int_clr1 & CLR1_TX_DONE_FLG) {
      ESP_LOGVV(TAG, "Chunked TX complete: %d/%d bytes written, %u refill(s)",
               (int) written, (int) len, refills);
      if (written < len) {
        ESP_LOGW(TAG, "Chunked TX: TX_DONE but only %d/%d bytes written — frame truncated on air!",
                 (int) written, (int) len);
      }
      return true;
    }

    // Refill when the TX FIFO has DRAINED to/below threshold. Per the datasheet
    // INT-source table, TX_FIFO_TH = 1 means "unread TX bytes OVER FIFO_TH" (still
    // full); it reads 0 once drained to <= FIFO_TH (room for another chunk).
    // Refill on this flag CLEAR.
    //
    // BUG FIX: we previously polled GPIO3 (active-high TX_FIFO_TH) and refilled
    // while it was HIGH — i.e. while the FIFO was still FULL — which overflowed
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

    // Feed the watchdog — this loop can block for many ms and ESP8266's soft
    // WDT resets if we spin without yielding.
    esphome::yield();
    esphome::delayMicroseconds(50);
  }

  ESP_LOGW(TAG, "Chunked TX timeout (%d/%d bytes written, %u refills, state=0x%02X, int_clr1=0x%02X)",
           (int) written, (int) len, refills, get_state(), spi_read_reg(REG_INT_CLR1));
  go_standby();
  return false;
}

uint8_t Cmt2300aHal::get_interrupt_flags() {
  return spi_read_reg(REG_INT_FLAG);
}

uint8_t Cmt2300aHal::get_int_clr1() {
  return spi_read_reg(REG_INT_CLR1);
}

void Cmt2300aHal::clear_interrupt_flags() {
  write_reg(REG_INT_CLR1, CLR1_TX_DONE_CLR | CLR1_SL_TMO_CLR | CLR1_RX_TMO_CLR);
  write_reg(REG_INT_CLR2, CLR2_LBD_CLR | CLR2_PREAM_OK_CLR | CLR2_SYNC_OK_CLR |
                          CLR2_NODE_OK_CLR | CLR2_CRC_OK_CLR | CLR2_PKT_DONE_CLR);
}

bool Cmt2300aHal::is_pkt_ok() {
  return (get_interrupt_flags() & FLAG_PKT_OK) != 0;
}

bool Cmt2300aHal::is_tx_done() {
  return (get_int_clr1() & CLR1_TX_DONE_FLG) != 0;
}

int8_t Cmt2300aHal::get_rssi_dbm() {
  // REG 0x70 returns unsigned 0..255; dBm = regval - 128.
  int val = static_cast<int>(spi_read_reg(REG_RSSI_DBM)) - 128;
  return static_cast<int8_t>(val);
}

uint8_t Cmt2300aHal::get_rssi_code() {
  return spi_read_reg(REG_RSSI_CODE);
}

void Cmt2300aHal::set_rssi_mode(bool enable) {
  // SYS11 (0x16) low 5 bits: 0x01 = RSSI-valid mode (get_rssi_dbm() reads real
  // values), FIFO_MERGE_VALUE (0x12) = normal merged-FIFO packet mode.
  update_reg(REG_SYS11, 0x1F, enable ? 0x01 : FIFO_MERGE_VALUE);
}

uint8_t Cmt2300aHal::get_fifo_flags() {
  return spi_read_reg(REG_FIFO_FLAG);
}

bool Cmt2300aHal::read_gpio3() {
  return pin_read(gpio3_);
}

bool Cmt2300aHal::test_gpio3_wiring() {
  // INT2 = RX_ACTIVE → pin is high only while the chip sits in RX state.
  set_int_source(INT_SEL_RX_ACTIVE);

  go_standby();
  esphome::delayMicroseconds(300);
  bool in_stby = pin_read(gpio3_);

  // Enter RX (STBY → RFS → RX) and sample the pin.
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
    ESP_LOGI(TAG, "GPIO3 wiring OK: pin LOW in STBY, HIGH in RX (INT2→pin_gpio3 good)");
  } else {
    ESP_LOGE(TAG, "GPIO3 wiring FAIL: STBY=%d RX=%d (expected 0 then 1). "
                  "Check the chip's GPIO3 pad is wired to pin_gpio3 — RX cannot work otherwise.",
             in_stby, in_rx);
  }
  return ok;
}

void Cmt2300aHal::set_int_source(uint8_t source) {
  // INT2 surfaces on the GPIO3 pad — the line we poll (TX_FIFO_TH during TX,
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

size_t Cmt2300aHal::poll_rx_drain(uint8_t *buf, size_t buf_size) {
  // RX_FIFO_TH-driven chunk drain. INT2 (=RX_FIFO_TH) surfaces on the GPIO3 pin
  // and is HIGH while >= FIFO_TH_VALUE bytes sit in the RX FIFO. We poll that pin
  // (the stock CIU reads the chip's INT output pin via MCU GPIO too; the SPI
  // status regs 0x6D/0x6E read 0xFF here — Control2 bank — so they're unusable).
  // Drain full threshold chunks while the line is asserted.
  //
  // The trailing < FIFO_TH_VALUE bytes of a frame never raise RX_FIFO_TH; the
  // caller (poll_rx_) reads that tail by frame length once it has arrived.
  size_t total = 0;
  while (total + FIFO_TH_VALUE <= buf_size && pin_read(gpio3_)) {
    spi_read_fifo(buf + total, FIFO_TH_VALUE);
    total += FIFO_TH_VALUE;
  }
  return total;
}

}  // namespace esphome::nartis_rf_meter
