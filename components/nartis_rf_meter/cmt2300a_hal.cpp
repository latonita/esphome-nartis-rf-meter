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

/* ================================================================
 * Destructor — no resources to clean up (polling-based RX, no ISR/queue)
 * ================================================================ */

Cmt2300aHal::~Cmt2300aHal() = default;

/* ================================================================
 * Pin helpers — ESPHome ISRInternalGPIOPin (fast, non-virtual, portable)
 * pin_high/pin_low/pin_read are inline static in the header.
 * ================================================================ */

void Cmt2300aHal::sdio_set_output() { sdio_.pin_mode(gpio::FLAG_OUTPUT); }
void Cmt2300aHal::sdio_set_input()  { sdio_.pin_mode(gpio::FLAG_INPUT); }

void Cmt2300aHal::spi_delay() {
  // ~0.5 us delay for ~1 MHz SPI clock
  esphome::delayMicroseconds(1);
}

/* ================================================================
 * Bit-bang SPI primitives
 *
 * CMT2300A SPI protocol:
 *   - SDIO is bidirectional (half-duplex)
 *   - Data clocked on RISING edge of SCLK
 *   - MSB first
 *   - Address byte: bit7 = R/W (1=read, 0=write), bits[6:0] = address
 *   - CSB for register access, FCSB for FIFO access
 * ================================================================ */

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

/* ================================================================
 * Public register access
 * ================================================================ */

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

/* ================================================================
 * Pin configuration
 * ================================================================ */

void Cmt2300aHal::set_pins(esphome::InternalGPIOPin *sdio, esphome::InternalGPIOPin *sclk,
                           esphome::InternalGPIOPin *csb, esphome::InternalGPIOPin *fcsb,
                           esphome::InternalGPIOPin *gpio1) {
  // Keep the pin objects for setup()/pin_mode at init, and snapshot fast
  // ISR-safe handles for the bit-bang inner loop (non-virtual access).
  sdio_pin_ = sdio;
  sclk_pin_ = sclk;
  csb_pin_ = csb;
  fcsb_pin_ = fcsb;
  gpio1_pin_ = gpio1;
  sdio_ = sdio->to_isr();
  sclk_ = sclk->to_isr();
  csb_ = csb->to_isr();
  fcsb_ = fcsb->to_isr();
  gpio1_ = gpio1->to_isr();
}

/* ================================================================
 * Initialization
 * ================================================================ */

bool Cmt2300aHal::init() {
  ESP_LOGI(TAG, "Initializing CMT2300A...");

  // Register + configure each pin through its GPIOPin object (applies the
  // platform-correct setup), then set the directions we need.
  sclk_pin_->setup();
  csb_pin_->setup();
  fcsb_pin_->setup();
  sdio_pin_->setup();
  gpio1_pin_->setup();
  sclk_.pin_mode(gpio::FLAG_OUTPUT);
  csb_.pin_mode(gpio::FLAG_OUTPUT);
  fcsb_.pin_mode(gpio::FLAG_OUTPUT);
  sdio_.pin_mode(gpio::FLAG_OUTPUT);
  gpio1_.pin_mode(gpio::FLAG_INPUT);

  // Idle state: both CS lines high, clock low
  pin_high(csb_);
  pin_high(fcsb_);
  pin_low(sclk_);

  // Soft reset
  spi_write_reg(REG_SOFT_RST, SOFT_RST_VALUE);

  // Wait for chip to stabilize
  esphome::delay(RESET_DELAY_MS);

  // Enter standby
  spi_write_reg(REG_MODE_CTL, GO_STBY);
  if (!wait_for_state(STA_STBY, 20)) {
    ESP_LOGE(TAG, "Failed to enter standby after reset");
    return false;
  }

  // Verify chip is responding
  if (!is_chip_connected()) {
    ESP_LOGE(TAG, "CMT2300A not detected — check wiring");
    return false;
  }

  // Enable config retention (preserve config across state transitions)
  update_reg(REG_MODE_STA, MASK_CFG_RETAIN, MASK_CFG_RETAIN);

  // Enable LFOSC_RECAL (firmware: REG_EN_CTL |= 0x20)
  update_reg(REG_EN_CTL, 0x20, 0x20);

  // Write all 96 bytes of register configuration
  write_full_config();

  // Apply runtime overrides
  apply_runtime_overrides();

  // Clear all interrupt flags
  clear_interrupt_flags();

  // Clear FIFO
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

  // Write 6 configuration banks
  write_bank(CMT_BANK_ADDR, NARTIS_CMT_BANK, CMT_BANK_SIZE);
  write_bank(SYSTEM_BANK_ADDR, NARTIS_SYSTEM_BANK, SYSTEM_BANK_SIZE);
  // Frequency bank written per-channel (default to CH0)
  write_bank(FREQUENCY_BANK_ADDR, NARTIS_FREQ_CHANNELS[0], FREQUENCY_BANK_SIZE);
  write_bank(DATA_RATE_BANK_ADDR, NARTIS_DATA_RATE_BANK, DATA_RATE_BANK_SIZE);
  write_bank(BASEBAND_BANK_ADDR, NARTIS_BASEBAND_BANK, BASEBAND_BANK_SIZE);
  write_bank(TX_BANK_ADDR, NARTIS_TX_BANK, TX_BANK_SIZE);

  // Post-init fixup: CMT10 = (reg & 0xF8) | 0x02
  update_reg(REG_CMT10, 0x07, 0x02);
}

void Cmt2300aHal::apply_runtime_overrides() {
  // FIFO merge: combine TX+RX FIFO into 64-byte single buffer
  // REG_SYS11 (0x16): (reg & 0xE0) | 0x12  — merge configuration
  update_reg(REG_SYS11, static_cast<uint8_t>(~MASK_FIFO_MERGE_CFG), FIFO_MERGE_VALUE);
  // REG_FIFO_CTL (0x69): set FIFO_MERGE_EN (firmware: FIFO_CTL |= 0x02)
  update_reg(REG_FIFO_CTL, MASK_FIFO_MERGE_EN, MASK_FIFO_MERGE_EN);

  // FIFO threshold: 12 bytes
  // REG_PKT29 (0x54): (reg & 0xE0) | 0x0C
  update_reg(REG_PKT29, MASK_FIFO_TH, FIFO_TH_VALUE);

  // GPIO1 = INT1 (for packet done / TX done interrupts)
  update_reg(REG_IO_SEL, MASK_GPIO1_SEL, GPIO1_SEL_INT1);

  // INT1 source = PKT_DONE (fires on both RX complete and TX complete)
  update_reg(REG_INT1_CTL, MASK_INT1_SEL, INT_SEL_PKT_DONE);

  // INT polarity: active high
  update_reg(REG_INT1_CTL, MASK_INT_POLAR, MASK_INT_POLAR);

  // Enable PKT_DONE interrupt
  write_reg(REG_INT_EN, INT_EN_PKT_DONE | INT_EN_TX_DONE);

  ESP_LOGD(TAG, "Runtime overrides applied (FIFO merge=64B, GPIO1=INT1/PKT_DONE)");
}

/* ================================================================
 * State Machine Control
 * ================================================================ */

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

  // Clear FIFO and interrupt flags before RX
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

  // Clear interrupt flags before TX
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

/* ================================================================
 * Frequency Channel Selection
 * ================================================================ */

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

  write_bank(FREQUENCY_BANK_ADDR, NARTIS_FREQ_CHANNELS[ch], FREQUENCY_BANK_SIZE);
  ESP_LOGD(TAG, "Set frequency channel %d", ch);
}

/* ================================================================
 * FIFO Operations
 * ================================================================ */

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
  // PKT14 (0x45): bits[6:4] = PAYLOAD_LENG[10:8], other bits cleared (matches firmware).
  // PKT15 (0x46): PAYLOAD_LENG[7:0].
  write_reg(REG_PKT14, static_cast<uint8_t>((len >> 8) & 0x07) << 4);
  write_reg(REG_PKT15, static_cast<uint8_t>(len & 0xFF));
}

uint8_t Cmt2300aHal::scan_channels(int8_t *out_score) {
  // Replica of firmware rssi_channel_select (0x000134A4).
  // Per channel: enable RSSI mode → GO_RX → 2 ms settle → 1 initial sample,
  //              6 more samples with 2 ms between each, track running max & min,
  //              score = (sum_of_6 - max - min) / 4.
  // Channel selection (firmware behavior, including the ch0-never-picked quirk):
  //   - best_ch starts at 0
  //   - ch1 always seeds best_ch=1, best_score=ch1_score
  //   - ch2/ch3 replace only if strictly less than current best
  //   - ch0's score is computed but never compared, so ch0 is unreachable

  int8_t best_score = 127;
  uint8_t best_ch = 0;

  for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
    go_standby();
    set_frequency_channel(ch);
    // Switch SYS11 to RSSI-valid mode (firmware FUN_00013daa: (SYS11 & 0xE0) | 0x01)
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

    // Firmware comparison logic — preserve ch0-unreachable quirk verbatim.
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
  // Matching firmware cmt_tx_chunked_write (0x131B8): fill FIFO with first 64B, enter TX,
  // then poll TX_FIFO_TH via GPIO1 to refill remaining data in 15-byte chunks.

  if (!go_standby()) return false;

  // Tell the packet engine how many bytes to transmit (fixed-length mode).
  set_tx_payload_length(static_cast<uint16_t>(len));

  clear_tx_fifo();
  clear_interrupt_flags();

  // Set merged FIFO direction to TX (firmware: FIFO_CTL |= 0x05)
  update_reg(REG_FIFO_CTL, MASK_FIFO_RX_TX_SEL | MASK_SPI_FIFO_RD_WR_SEL,
             MASK_FIFO_RX_TX_SEL | MASK_SPI_FIFO_RD_WR_SEL);

  // Configure INT1 = TX_FIFO_TH (fires when FIFO drops below threshold)
  set_int1_source(INT_SEL_TX_FIFO_TH);

  // Write first chunk (up to 64 bytes)
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

  // Poll: refill FIFO as it drains, wait for TX_DONE
  static constexpr uint32_t TX_TIMEOUT_MS = 2000;
  uint32_t start = esphome::millis();

  while (esphome::millis() - start < TX_TIMEOUT_MS) {
    // Check TX_DONE first — leave flag set for caller's is_tx_done() check
    uint8_t int_clr1 = spi_read_reg(REG_INT_CLR1);
    if (int_clr1 & CLR1_TX_DONE_FLG) {
      ESP_LOGD(TAG, "Chunked TX complete: %d bytes", (int) len);
      return true;
    }

    // If more data to write, check if FIFO needs refill via GPIO1 (TX_FIFO_TH)
    if (written < len && pin_read(gpio1_)) {
      size_t remaining = len - written;
      size_t chunk = (remaining > TX_REFILL_CHUNK) ? TX_REFILL_CHUNK : remaining;
      spi_write_fifo(data + written, chunk);
      written += chunk;
    }

    esphome::delayMicroseconds(50);
  }

  ESP_LOGW(TAG, "Chunked TX timeout (%d/%d bytes written)", (int) written, (int) len);
  go_standby();
  return false;
}

/* ================================================================
 * Interrupt & Status
 * ================================================================ */

uint8_t Cmt2300aHal::get_interrupt_flags() {
  return spi_read_reg(REG_INT_FLAG);
}

uint8_t Cmt2300aHal::get_int_clr1() {
  return spi_read_reg(REG_INT_CLR1);
}

void Cmt2300aHal::clear_interrupt_flags() {
  // Write clear bits to INT_CLR1 and INT_CLR2
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
  // REG 0x70 returns unsigned 0..255; dBm = regval - 128 (firmware FUN_00013f94).
  int val = static_cast<int>(spi_read_reg(REG_RSSI_DBM)) - 128;
  return static_cast<int8_t>(val);
}

uint8_t Cmt2300aHal::get_rssi_code() {
  return spi_read_reg(REG_RSSI_CODE);
}

uint8_t Cmt2300aHal::get_fifo_flags() {
  return spi_read_reg(REG_FIFO_FLAG);
}

bool Cmt2300aHal::read_gpio1() {
  return pin_read(gpio1_);
}

/* ================================================================
 * Chunked RX — for packets larger than 64-byte FIFO
 * ================================================================ */

void Cmt2300aHal::set_int1_source(uint8_t source) {
  // Preserve INT_POLAR bit, update INT1_SEL field.
  // Kept for the TX-chunked path; the RX path no longer relies on the
  // chip's GPIO1 interrupt line — we poll REG_FIFO_FLAG via SPI instead.
  update_reg(REG_INT1_CTL, MASK_INT1_SEL, source & MASK_INT1_SEL);
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
  // RX completion is signaled on the GPIO1 pin (configured INT1 = PKT_DONE),
  // read directly as a GPIO input. This mirrors the CIU firmware, which polls
  // the chip's INT *output pins* (memory-mapped MCU GPIO) rather than reading
  // the SPI status registers 0x6D/0x6E — those live in the chip's "Control2
  // bank" and read back 0xFF in this bit-bang setup, so they are unusable.
  //
  // NOTE: this reads a whole packet once PKT_DONE asserts, so it requires the
  // frame to fit in the 64-byte FIFO. All pairing frames (0x06=25B, 0x53=59B,
  // keepalive=25B) do. Larger GET responses (>64B) will need GPIO-driven
  // FIFO_TH chunking — a separate follow-up.
  if (buf_size == 0) return 0;
  if (!pin_read(gpio1_)) {
    return 0;  // no completed packet yet
  }

  // PKT_DONE asserted — the full frame sits in the RX FIFO. byte[0] = len-1.
  uint8_t lenb = 0;
  spi_read_fifo(&lenb, 1);
  size_t total = static_cast<size_t>(lenb) + 1;
  if (total > buf_size) total = buf_size;
  buf[0] = lenb;
  if (total > 1) {
    spi_read_fifo(buf + 1, total - 1);
  }
  return total;
}

}  // namespace esphome::nartis_rf_meter
