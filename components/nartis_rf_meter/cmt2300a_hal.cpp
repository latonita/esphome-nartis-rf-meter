/*
 * CMT2300A Hardware Abstraction Layer — Implementation
 *
 * Bit-bang 3-wire SPI driver for CMT2300A sub-GHz transceiver.
 * Handles register/FIFO access and chip state machine transitions.
 */

#include "cmt2300a_hal.h"

#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <esp_rom_sys.h>  // esp_rom_delay_us (ISR-safe)

namespace esphome::nartis_rf_meter {

static const char *const TAG = "cmt2300a_hal";

/* ================================================================
 * Destructor — cleanup FreeRTOS queue
 * ================================================================ */

Cmt2300aHal::~Cmt2300aHal() {
  if (isr_installed_ && gpio1_pin_ != nullptr) {
    gpio1_pin_->detach_interrupt();
  }
  if (rx_queue_ != nullptr) {
    vQueueDelete(rx_queue_);
  }
}

/* ================================================================
 * Pin helpers — ESP32 GPIO direct register access for speed
 * ================================================================ */

void Cmt2300aHal::pin_high(uint8_t pin) {
  gpio_set_level(static_cast<gpio_num_t>(pin), 1);
}

void Cmt2300aHal::pin_low(uint8_t pin) {
  gpio_set_level(static_cast<gpio_num_t>(pin), 0);
}

bool Cmt2300aHal::pin_read(uint8_t pin) {
  return gpio_get_level(static_cast<gpio_num_t>(pin)) != 0;
}

void Cmt2300aHal::sdio_set_output() {
  gpio_set_direction(static_cast<gpio_num_t>(pin_sdio_), GPIO_MODE_OUTPUT);
}

void Cmt2300aHal::sdio_set_input() {
  gpio_set_direction(static_cast<gpio_num_t>(pin_sdio_), GPIO_MODE_INPUT);
}

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
    pin_low(pin_sclk_);
    if (byte & (1 << i)) {
      pin_high(pin_sdio_);
    } else {
      pin_low(pin_sdio_);
    }
    spi_delay();
    pin_high(pin_sclk_);
    spi_delay();
  }
  pin_low(pin_sclk_);
}

uint8_t Cmt2300aHal::spi_recv_byte() {
  uint8_t byte = 0;
  for (int i = 7; i >= 0; i--) {
    pin_low(pin_sclk_);
    spi_delay();
    pin_high(pin_sclk_);
    spi_delay();
    if (pin_read(pin_sdio_)) {
      byte |= (1 << i);
    }
  }
  pin_low(pin_sclk_);
  return byte;
}

void Cmt2300aHal::spi_write_reg(uint8_t addr, uint8_t val) {
  pin_low(pin_csb_);
  spi_delay();

  sdio_set_output();
  spi_send_byte(addr & 0x7F);  // Bit 7 = 0 for write
  spi_send_byte(val);

  pin_high(pin_csb_);
  spi_delay();
}

uint8_t Cmt2300aHal::spi_read_reg(uint8_t addr) {
  pin_low(pin_csb_);
  spi_delay();

  sdio_set_output();
  spi_send_byte(addr | 0x80);  // Bit 7 = 1 for read

  // Switch SDIO to input before first data clock
  sdio_set_input();
  uint8_t val = spi_recv_byte();

  pin_high(pin_csb_);
  spi_delay();
  return val;
}

void Cmt2300aHal::spi_write_fifo(const uint8_t *data, size_t len) {
  sdio_set_output();

  for (size_t i = 0; i < len; i++) {
    pin_low(pin_fcsb_);
    spi_delay();
    spi_send_byte(data[i]);
    pin_high(pin_fcsb_);
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
    pin_low(pin_fcsb_);
    spi_delay();
    sdio_set_input();
    data[i] = spi_recv_byte();
    pin_high(pin_fcsb_);
    spi_delay();
    spi_delay();
    spi_delay();
    spi_delay();
  }
}

/* ================================================================
 * ISR-safe SPI primitives
 *
 * These use only gpio_set_level / gpio_get_level / esp_rom_delay_us
 * which are all safe to call from IRAM ISR context.
 * SDIO must already be configured as input before calling these.
 * ================================================================ */

void IRAM_ATTR Cmt2300aHal::spi_delay_isr() {
  esp_rom_delay_us(1);
}

uint8_t IRAM_ATTR Cmt2300aHal::spi_recv_byte_isr(uint8_t pin_sclk, uint8_t pin_sdio) {
  uint8_t byte = 0;
  auto sclk = static_cast<gpio_num_t>(pin_sclk);
  auto sdio = static_cast<gpio_num_t>(pin_sdio);
  for (int i = 7; i >= 0; i--) {
    gpio_set_level(sclk, 0);
    spi_delay_isr();
    gpio_set_level(sclk, 1);
    spi_delay_isr();
    if (gpio_get_level(sdio)) {
      byte |= (1 << i);
    }
  }
  gpio_set_level(sclk, 0);
  return byte;
}

void IRAM_ATTR Cmt2300aHal::spi_read_fifo_isr(uint8_t *data, size_t len,
                                                uint8_t pin_fcsb, uint8_t pin_sclk, uint8_t pin_sdio) {
  auto fcsb = static_cast<gpio_num_t>(pin_fcsb);
  for (size_t i = 0; i < len; i++) {
    gpio_set_level(fcsb, 0);
    spi_delay_isr();
    data[i] = spi_recv_byte_isr(pin_sclk, pin_sdio);
    gpio_set_level(fcsb, 1);
    spi_delay_isr();
    spi_delay_isr();
    spi_delay_isr();
    spi_delay_isr();
  }
}

/* ================================================================
 * GPIO1 ISR — triggered by RX_FIFO_TH (auto-clearing per AN143)
 *
 * Reads FIFO_TH_VALUE (12) bytes from FIFO and pushes to queue.
 * ~300 us execution time at 1 MHz bit-bang SPI.
 * ================================================================ */

void IRAM_ATTR Cmt2300aHal::gpio1_isr_handler(Cmt2300aHal *self) {

  FifoChunk chunk;
  chunk.len = FIFO_TH_VALUE;
  spi_read_fifo_isr(chunk.data, chunk.len,
                     self->pin_fcsb_, self->pin_sclk_, self->pin_sdio_);

  BaseType_t higher_prio_woken = pdFALSE;
  xQueueSendFromISR(self->rx_queue_, &chunk, &higher_prio_woken);

  if (higher_prio_woken) {
    portYIELD_FROM_ISR();
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
  // Extract raw pin numbers for fast bit-bang SPI (direct ESP-IDF GPIO calls)
  pin_sdio_ = sdio->get_pin();
  pin_sclk_ = sclk->get_pin();
  pin_csb_ = csb->get_pin();
  pin_fcsb_ = fcsb->get_pin();
  pin_gpio1_ = gpio1->get_pin();
  // Keep InternalGPIOPin pointer for GPIO1 interrupt management
  gpio1_pin_ = gpio1;
}

/* ================================================================
 * Initialization
 * ================================================================ */

bool Cmt2300aHal::init() {
  ESP_LOGI(TAG, "Initializing CMT2300A...");

  // Configure GPIO pins (ESP-IDF direct GPIO for bit-bang SPI)
  gpio_config_t out_conf = {};
  out_conf.pin_bit_mask = (1ULL << pin_sclk_) | (1ULL << pin_csb_) | (1ULL << pin_fcsb_) | (1ULL << pin_sdio_);
  out_conf.mode = GPIO_MODE_OUTPUT;
  out_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  out_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  out_conf.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&out_conf);

  gpio_config_t in_conf = {};
  in_conf.pin_bit_mask = (1ULL << pin_gpio1_);
  in_conf.mode = GPIO_MODE_INPUT;
  in_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  in_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  in_conf.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&in_conf);

  // Idle state: both CS lines high, clock low
  pin_high(pin_csb_);
  pin_high(pin_fcsb_);
  pin_low(pin_sclk_);

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

  // Create FreeRTOS queue for chunked RX
  rx_queue_ = xQueueCreate(RX_QUEUE_CAPACITY, sizeof(FifoChunk));
  if (rx_queue_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create RX queue");
    return false;
  }

  // Attach GPIO1 interrupt handler (disabled until enable_rx_interrupt)
  if (!isr_installed_ && gpio1_pin_ != nullptr) {
    gpio1_pin_->attach_interrupt(gpio1_isr_handler, this, esphome::gpio::INTERRUPT_RISING_EDGE);
    gpio_intr_disable(static_cast<gpio_num_t>(pin_gpio1_));
    isr_installed_ = true;
  }

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

bool Cmt2300aHal::transmit_chunked(const uint8_t *data, size_t len) {
  // Matching firmware cmt_tx_chunked_write (0x131B8): fill FIFO with first 64B, enter TX,
  // then poll TX_FIFO_TH via GPIO1 to refill remaining data in 15-byte chunks.

  if (!go_standby()) return false;
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
    if (written < len && pin_read(pin_gpio1_)) {
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
  return static_cast<int8_t>(spi_read_reg(REG_RSSI_DBM));
}

uint8_t Cmt2300aHal::get_rssi_code() {
  return spi_read_reg(REG_RSSI_CODE);
}

uint8_t Cmt2300aHal::get_fifo_flags() {
  return spi_read_reg(REG_FIFO_FLAG);
}

bool Cmt2300aHal::read_gpio1() {
  return pin_read(pin_gpio1_);
}

/* ================================================================
 * Chunked RX — for packets larger than 64-byte FIFO
 * ================================================================ */

void Cmt2300aHal::set_int1_source(uint8_t source) {
  // Preserve INT_POLAR bit, update INT1_SEL field
  update_reg(REG_INT1_CTL, MASK_INT1_SEL, source & MASK_INT1_SEL);
}

void Cmt2300aHal::prepare_fifo_read() {
  // Set SDIO pin to input once before the RX session.
  // This avoids calling gpio_set_direction (which takes a spinlock)
  // inside the ISR.
  sdio_set_input();

  // Configure SPI_FIFO_RD_WR_SEL = 0 (SPI reads FIFO) and
  // FIFO_RX_TX_SEL = 0 (merged FIFO used as RX)
  update_reg(REG_FIFO_CTL, MASK_SPI_FIFO_RD_WR_SEL | MASK_FIFO_RX_TX_SEL, 0x00);
}

void Cmt2300aHal::enable_rx_interrupt() {
  if (!isr_installed_) return;

  // Flush any stale chunks from previous RX session
  FifoChunk discard;
  while (xQueueReceive(rx_queue_, &discard, 0) == pdTRUE) {}

  // Enable GPIO1 interrupt (type already set by attach_interrupt in init)
  gpio_intr_enable(static_cast<gpio_num_t>(pin_gpio1_));
}

void Cmt2300aHal::disable_rx_interrupt() {
  gpio_intr_disable(static_cast<gpio_num_t>(pin_gpio1_));
}

size_t Cmt2300aHal::drain_rx_queue(uint8_t *buf, size_t buf_size) {
  FifoChunk chunk;
  size_t total = 0;
  while (xQueueReceive(rx_queue_, &chunk, 0) == pdTRUE) {
    size_t copy_len = chunk.len;
    if (total + copy_len > buf_size) {
      copy_len = buf_size - total;
    }
    memcpy(buf + total, chunk.data, copy_len);
    total += copy_len;
    if (total >= buf_size) break;
  }
  return total;
}

}  // namespace esphome::nartis_rf_meter
