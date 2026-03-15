/*
 * CMT2300A Hardware Abstraction Layer
 *
 * Layer 1: Drives CMT2300A transceiver via bit-bang 3-wire SPI.
 * Zero knowledge of packets, protocols, or framing.
 *
 * Pin interface:
 *   SDIO  — bidirectional data (shared MOSI/MISO)
 *   SCLK  — serial clock
 *   CSB   — chip select for register access (active low)
 *   FCSB  — chip select for FIFO access (active low)
 *   GPIO1 — interrupt output from CMT2300A (active high)
 *
 * FIFO reception:
 *   For packets > 64 bytes (FIFO size), the HAL uses a GPIO interrupt
 *   on the RX_FIFO_TH signal to read 12-byte chunks directly in the ISR,
 *   pushing them to a FreeRTOS queue. The main loop drains the queue
 *   into an accumulation buffer. RX_FIFO_TH is auto-clearing (AN143).
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include "cmt2300a_defs.h"

#include "esphome/core/gpio.h"

// ESP-IDF GPIO for direct pin control (bit-bang SPI requires fast GPIO access)
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

namespace esphome::nartis_rf_meter {

/// Chunk of FIFO data read in ISR context
struct FifoChunk {
  uint8_t data[FIFO_TH_VALUE];  // 12 bytes
  uint8_t len;                   // actual bytes read
};

class Cmt2300aHal {
 public:
  Cmt2300aHal() = default;
  ~Cmt2300aHal();

  /// Set GPIO pins. Must be called before init().
  void set_pins(esphome::InternalGPIOPin *sdio, esphome::InternalGPIOPin *sclk,
                esphome::InternalGPIOPin *csb, esphome::InternalGPIOPin *fcsb,
                esphome::InternalGPIOPin *gpio1);

  /// Initialize the CMT2300A: soft reset, verify chip, write full register config.
  /// Returns true on success.
  bool init();

  /// Check if the chip is responding (read product ID register).
  bool is_chip_connected();

  /* ---- State Control ---- */

  /// Transition to Standby mode. Returns true if state reached within timeout.
  bool go_standby();

  /// Transition to Sleep mode.
  bool go_sleep();

  /// Transition to RX mode (via RFS → RX).
  bool go_rx();

  /// Transition to TX mode (via TFS → TX).
  bool go_tx();

  /// Read current chip state from MODE_STA register.
  uint8_t get_state();

  /// Wait for a specific state with timeout. Returns true if reached.
  bool wait_for_state(uint8_t target_state, uint32_t timeout_ms = STATE_POLL_TIMEOUT_MS);

  /* ---- Frequency Channel ---- */

  /// Switch frequency registers to channel 0-3.
  void set_frequency_channel(uint8_t ch);

  /* ---- FIFO Operations ---- */

  /// Clear both TX and RX FIFO.
  void clear_fifo();

  /// Clear TX FIFO only.
  void clear_tx_fifo();

  /// Clear RX FIFO only.
  void clear_rx_fifo();

  /// Write data to TX FIFO. Returns number of bytes written (limited by FIFO size).
  size_t write_fifo(const uint8_t *data, size_t len);

  /// Chunked TX: write first 64B to FIFO, enter TX, poll TX_FIFO_TH to refill
  /// remaining data in 15-byte chunks. Returns true if TX_DONE received.
  /// Handles packets up to MAX_RF_FRAME_SIZE bytes.
  bool transmit_chunked(const uint8_t *data, size_t len);

  /// Read data from RX FIFO.
  size_t read_fifo(uint8_t *data, size_t max_len);

  /* ---- Interrupt & Status ---- */

  /// Read INT_FLAG register (0x6D).
  uint8_t get_interrupt_flags();

  /// Read INT_CLR1 register (0x6A) for TX_DONE flag.
  uint8_t get_int_clr1();

  /// Clear all interrupt flags.
  void clear_interrupt_flags();

  /// Check if a complete packet has been received (PKT_OK flag).
  bool is_pkt_ok();

  /// Check if TX is complete (TX_DONE flag in INT_CLR1).
  bool is_tx_done();

  /// Read RSSI in dBm (signed).
  int8_t get_rssi_dbm();

  /// Read raw RSSI code.
  uint8_t get_rssi_code();

  /// Check FIFO flags register.
  uint8_t get_fifo_flags();

  /// Read GPIO1 pin state (interrupt line).
  bool read_gpio1();

  /* ---- Chunked RX (for packets > 64 bytes) ---- */

  /// Set INT1 source register (e.g., INT_SEL_RX_FIFO_TH or INT_SEL_PKT_DONE).
  void set_int1_source(uint8_t source);

  /// Prepare SDIO pin for FIFO reading (set input once before RX session).
  /// Must be called before enabling RX interrupt.
  void prepare_fifo_read();

  /// Enable GPIO1 interrupt for chunked RX (ISR reads FIFO on RX_FIFO_TH).
  void enable_rx_interrupt();

  /// Disable GPIO1 interrupt.
  void disable_rx_interrupt();

  /// Drain all available chunks from the RX queue into buffer.
  /// Returns total bytes copied. Non-blocking.
  size_t drain_rx_queue(uint8_t *buf, size_t buf_size);

  /* ---- Low-Level Register Access ---- */

  /// Write a single register.
  void write_reg(uint8_t addr, uint8_t val);

  /// Read a single register.
  uint8_t read_reg(uint8_t addr);

  /// Write a bank of consecutive registers.
  void write_bank(uint8_t start_addr, const uint8_t *data, size_t len);

  /// Read-modify-write: (reg & ~mask) | (val & mask).
  void update_reg(uint8_t addr, uint8_t mask, uint8_t val);

 private:
  /* ---- Bit-bang SPI primitives ---- */

  /// Send 8 bits on SDIO (MSB first), SDIO configured as output.
  void spi_send_byte(uint8_t byte);

  /// Receive 8 bits from SDIO (MSB first), SDIO configured as input.
  uint8_t spi_recv_byte();

  /// ISR-safe receive: no gpio_set_direction, SDIO must already be input.
  static uint8_t IRAM_ATTR spi_recv_byte_isr(uint8_t pin_sclk, uint8_t pin_sdio);

  /// Register write: CSB low, send addr|WRITE, send data, CSB high.
  void spi_write_reg(uint8_t addr, uint8_t val);

  /// Register read: CSB low, send addr|READ, switch SDIO to input, read data, CSB high.
  uint8_t spi_read_reg(uint8_t addr);

  /// FIFO write: FCSB low, send bytes, FCSB high.
  void spi_write_fifo(const uint8_t *data, size_t len);

  /// FIFO read: FCSB low, read bytes, FCSB high.
  void spi_read_fifo(uint8_t *data, size_t len);

  /// ISR-safe FIFO read: SDIO must already be input. No gpio_set_direction calls.
  static void IRAM_ATTR spi_read_fifo_isr(uint8_t *data, size_t len,
                                           uint8_t pin_fcsb, uint8_t pin_sclk, uint8_t pin_sdio);

  /// Set SDIO pin direction.
  void sdio_set_output();
  void sdio_set_input();

  /// GPIO helpers.
  void pin_high(uint8_t pin);
  void pin_low(uint8_t pin);
  bool pin_read(uint8_t pin);

  /// Tiny delay for SPI timing (~0.5-1 MHz clock).
  void spi_delay();

  /// ISR-safe delay (~1 us busy wait).
  static void IRAM_ATTR spi_delay_isr();

  /// Write the complete 96-byte register configuration.
  void write_full_config();

  /// Apply runtime overrides (FIFO merge, FIFO threshold, GPIO/interrupt config).
  void apply_runtime_overrides();

  /// GPIO1 ISR handler — reads FIFO_TH_VALUE bytes and pushes to rx_queue_.
  static void IRAM_ATTR gpio1_isr_handler(Cmt2300aHal *self);

  /* ---- Pin assignments ---- */
  // Raw pin numbers extracted from InternalGPIOPin for fast bit-bang SPI access
  uint8_t pin_sdio_{0};
  uint8_t pin_sclk_{0};
  uint8_t pin_csb_{0};
  uint8_t pin_fcsb_{0};
  uint8_t pin_gpio1_{0};
  // InternalGPIOPin pointer for GPIO1 interrupt attach/detach
  esphome::InternalGPIOPin *gpio1_pin_{nullptr};

  bool initialized_{false};

  /* ---- Chunked RX state ---- */
  QueueHandle_t rx_queue_{nullptr};
  bool isr_installed_{false};
};

}  // namespace esphome::nartis_rf_meter
