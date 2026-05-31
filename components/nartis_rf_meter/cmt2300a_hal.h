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
 *   GPIO1 — radio status output (unused: we poll via SPI, matching the
 *           original CIU firmware which is also fully polled)
 *
 * FIFO reception:
 *   Polling-based. The component's main loop calls poll_rx_drain() each
 *   iteration; it reads REG_FIFO_FLAG via SPI and drains whatever is
 *   available — a full FIFO_TH_VALUE-byte chunk in one burst when the
 *   RX_FIFO_TH flag is set, or byte-by-byte for the packet tail.
 *
 *   At the meter's 4800 bps on-air rate (~600 B/s), bytes trickle in
 *   one every ~1.7 ms, so the ESPHome loop() rate is more than fast
 *   enough to keep up. No GPIO interrupt is required.
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include "cmt2300a_defs.h"

#include "esphome/core/gpio.h"  // gpio::Flags, InternalGPIOPin

namespace esphome::nartis_rf_meter {

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

  /// Set per-TX payload length (PKT14 high bits + PKT15 low byte).
  /// Firmware writes this before every TX: bank leaves chip in fixed-length mode
  /// with PKT15=0xFF (511 bytes), so TX_DONE never fires for shorter packets.
  void set_tx_payload_length(uint16_t len);

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

  /// Scan all NUM_CHANNELS channels, return the channel index with the lowest
  /// trimmed-mean RSSI. Exact replica of CIU firmware rssi_channel_select (0x000134A4):
  ///   per channel: enable RSSI mode → GO_RX → 2 ms settle → 1 initial sample,
  ///                then 6 more samples with 2 ms between each,
  ///                score = (sum_of_6 - max - min) / 4  (trimmed mean of middle 4).
  /// Note: firmware never selects channel 0 (off-by-one bug: ch0's score is computed
  /// but never compared). This impl preserves that behavior so the meter's expectations
  /// match — channels 1, 2, 3 are eligible; the device returns 1 if none is better.
  /// If out_score is non-null, must point to at least NUM_CHANNELS int8_t and is
  /// filled with each channel's trimmed-mean dBm score.
  /// Leaves chip in STBY with FIFO merge restored, parked on the chosen channel.
  uint8_t scan_channels(int8_t *out_score = nullptr);

  /// Check FIFO flags register.
  uint8_t get_fifo_flags();

  /// Read GPIO1 pin state (interrupt line).
  bool read_gpio1();

  /// Set INT1 source field of REG_INT1_CTL. Used internally by the
  /// TX-chunked path; harmless on the RX path since we no longer read
  /// the GPIO1 line as an interrupt.
  void set_int1_source(uint8_t source);

  /* ---- Chunked RX (polling-based, no ISR) ---- */

  /// Prepare the SDIO pin + FIFO control for an RX session. Call once
  /// after entering RX mode, before the first poll_rx_drain() call.
  void prepare_rx_session();

  /// Non-blocking poll: read REG_FIFO_FLAG via SPI and drain whatever
  /// bytes are currently sitting in the RX FIFO into `buf`.
  ///
  /// Burst-reads a full FIFO_TH_VALUE chunk when the RX_FIFO_TH flag is
  /// set, otherwise reads byte-by-byte (typical for the packet tail).
  /// Stops at `buf_size` even if more bytes are pending.
  ///
  /// Returns the number of bytes appended to `buf` this call (0 if the
  /// FIFO is empty).
  size_t poll_rx_drain(uint8_t *buf, size_t buf_size);

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

  /// Register write: CSB low, send addr|WRITE, send data, CSB high.
  void spi_write_reg(uint8_t addr, uint8_t val);

  /// Register read: CSB low, send addr|READ, switch SDIO to input, read data, CSB high.
  uint8_t spi_read_reg(uint8_t addr);

  /// FIFO write: FCSB low, send bytes, FCSB high.
  void spi_write_fifo(const uint8_t *data, size_t len);

  /// FIFO read: FCSB low, read bytes, FCSB high.
  /// Assumes SDIO has already been set to input (prepare_rx_session()).
  void spi_read_fifo(uint8_t *data, size_t len);

  /// Set SDIO pin direction.
  void sdio_set_output();
  void sdio_set_input();

  /// GPIO helpers (operate on the fast ISR-safe pin handles).
  static void pin_high(esphome::ISRInternalGPIOPin &pin) { pin.digital_write(true); }
  static void pin_low(esphome::ISRInternalGPIOPin &pin) { pin.digital_write(false); }
  static bool pin_read(esphome::ISRInternalGPIOPin &pin) { return pin.digital_read(); }

  /// Tiny delay for SPI timing (~0.5-1 MHz clock).
  void spi_delay();

  /// Write the complete 96-byte register configuration.
  void write_full_config();

  /// Apply runtime overrides (FIFO merge, FIFO threshold, GPIO/interrupt config).
  void apply_runtime_overrides();

  /* ---- Pin assignments ---- */
  // Configured GPIO pin objects (used for setup() + pin_mode at init).
  esphome::InternalGPIOPin *sdio_pin_{nullptr};
  esphome::InternalGPIOPin *sclk_pin_{nullptr};
  esphome::InternalGPIOPin *csb_pin_{nullptr};
  esphome::InternalGPIOPin *fcsb_pin_{nullptr};
  esphome::InternalGPIOPin *gpio1_pin_{nullptr};
  // Fast non-virtual handles for the bit-bang inner loop (via pin->to_isr()).
  esphome::ISRInternalGPIOPin sdio_;
  esphome::ISRInternalGPIOPin sclk_;
  esphome::ISRInternalGPIOPin csb_;
  esphome::ISRInternalGPIOPin fcsb_;
  esphome::ISRInternalGPIOPin gpio1_;

  bool initialized_{false};
};

}  // namespace esphome::nartis_rf_meter
