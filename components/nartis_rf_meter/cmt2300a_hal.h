/*
 * CMT2300A Hardware Abstraction Layer - a RadioHal backend for RF433-1.
 *
 * Layer 1: Drives CMT2300A transceiver via bit-bang 3-wire SPI.
 * Zero knowledge of packets, protocols, or framing - everything above the
 * radio reaches it as opaque bytes through the RadioHal interface.
 *
 * Pin interface:
 *   SDIO  - bidirectional data (shared MOSI/MISO)
 *   SCLK  - serial clock
 *   CSB   - chip select for register access (active low)
 *   FCSB  - chip select for FIFO access (active low)
 *   GPIO3 - chip interrupt output, routed to INT2. Carries the FIFO-threshold
 *           level we poll (RX_FIFO_TH in RX, TX_FIFO_TH in TX). On this chip
 *           GPIO3 can only output INT2 (not INT1), which is why INT2 is used.
 *
 * FIFO reception (polling, no ISR):
 *   The component's main loop calls drain_rx() each iteration. It polls
 *   the GPIO3/INT2 line - HIGH while >= FIFO_TH_VALUE unread bytes sit in the
 *   RX FIFO (RX_FIFO_TH, an auto-clearing level per the datasheet) - and burst-
 *   reads a full FIFO_TH_VALUE chunk via SPI each time the line is asserted.
 *   The trailing < FIFO_TH_VALUE bytes never raise RX_FIFO_TH, so the caller
 *   reads that tail by frame length through read_rx_exact() once it has arrived.
 *   That is the whole reason RadioHal has a read_rx_exact() and an
 *   rx_chunk_size() at all.
 *
 *   We poll the physical pin rather than the SPI FIFO-flag register (0x6E):
 *   that register reads 0xFF here (Control2 bank), so it is unusable - matching
 *   the stock CIU, which also reads the chip's INT pin via MCU GPIO. The same
 *   fact is why drain_rx() cannot report RX_STATUS_OVERFLOW (see the .cpp).
 *
 * Frequency:
 *   set_frequency() is the TX carrier and set_rx_frequency() the RX carrier;
 *   they are independent and either may be changed at runtime between
 *   exchanges. Both resolve the requested carrier back to the proven register
 *   blobs in NARTIS_FREQ_CHANNELS and write those bytes unchanged - the AN199
 *   formula is a loudly-logged fallback only. This chip is a low-IF receiver,
 *   so the RX-half register value is the LO, one IF above the carrier; callers
 *   never see that, they pass carriers.
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include "cmt2300a_defs.h"
#include "radio_hal.h"

#include "esphome/core/gpio.h"  // gpio::Flags, InternalGPIOPin

namespace esphome::nartis_rf_meter {

class Cmt2300aHal : public RadioHal {
 public:
  Cmt2300aHal() = default;
  ~Cmt2300aHal() override;

  const char *radio_name() const override { return "CMT2300A"; }

  /// Set GPIO pins. Must be called before init(). Not part of RadioHal: the
  /// bit-banged 3-wire bus is this chip's own business.
  void set_pins(esphome::InternalGPIOPin *sdio, esphome::InternalGPIOPin *sclk,
                esphome::InternalGPIOPin *csb, esphome::InternalGPIOPin *fcsb,
                esphome::InternalGPIOPin *gpio3);

  /// Initialize the CMT2300A: soft reset, verify chip, write full register config.
  /// Returns true on success.
  bool init() override;

  /// TX carrier, Hz. Writes only the TX-half of the frequency bank (0x1C..0x1F).
  /// May be called at runtime; parks the chip in standby first if it is not
  /// already there, because the synthesiser registers cannot be changed under way.
  void set_frequency(uint32_t freq_hz) override;

  /// RX carrier, Hz. Writes only the RX-half of the frequency bank (0x18..0x1B),
  /// leaving the TX-half alone - which is exactly what RF433-1's alternative
  /// channel mode needs: RX hops while TX stays on the meter's wake frequency.
  void set_rx_frequency(uint32_t freq_hz) override;

  /// Chunked TX: write first 64B to FIFO, enter TX, poll TX_FIFO_TH to refill
  /// remaining data in 15-byte chunks. Blocks to TX_DONE (~600 ms cap) and parks
  /// the chip in standby. Returns true if TX_DONE was seen.
  bool transmit(const uint8_t *frame, size_t len) override;

  /// Start a verified RX epoch. Carries out, in this exact order, what the
  /// component's start_rx_() used to do inline - the order is load-bearing, see
  /// the comment on the implementation. Returns false if the chip did not reach
  /// RFS or RX, in which case drain_rx() answers RX_STATUS_NOT_IN_RX.
  bool begin_rx() override;

  /// Drain whole FIFO_TH_VALUE chunks while the GPIO3/INT2 threshold line is
  /// asserted. Returns bytes appended and sets `status`.
  size_t drain_rx(uint8_t *buf, size_t buf_size, RxStatus &status) override;

  /// Read `len` bytes straight out of the RX FIFO with no threshold signal -
  /// how the sub-threshold tail of every frame is collected.
  size_t read_rx_exact(uint8_t *buf, size_t len) override;

  /// The threshold chunk: drain_rx() hands over bytes only in multiples of this.
  size_t rx_chunk_size() const override { return FIFO_TH_VALUE; }

  /// Honest bound, and it is a weak one. Unlike the sibling component's
  /// CMT2300A backend, this drain is capped by nothing but the buffer it is
  /// handed: it keeps reading threshold chunks for as long as the line stays
  /// asserted, so a sender that keeps the FIFO topped up could in principle be
  /// drained for more than one FIFO's worth in a single call. FIFO_SIZE_MERGED
  /// is what it can return from an already-filled FIFO, which is the bound that
  /// matters in practice at 2400 bps, where the chip cannot refill faster than
  /// we read. No cap was ADDED here on purpose: this backend is proven against
  /// live meters and a cap would change its draining behaviour.
  ///
  /// Nothing in this component currently decides anything from this number -
  /// frames are bounded by their own LEN byte, and the "buffer full" check uses
  /// the real buffer size - so the weakness costs nothing today. A backend whose
  /// drain IS chunk-capped should report the true product (chunk * cap).
  size_t max_drain_chunk() const override { return FIFO_SIZE_MERGED; }

  /// Read RSSI in dBm (signed). Only meaningful with set_rssi_mode(true) active.
  int8_t get_rssi_dbm() override;

  /// Enable RSSI-valid mode (SYS11 low bits = 0x01) so get_rssi_dbm() returns real
  /// values; pass false to restore the FIFO-merge config for normal packet I/O.
  /// The two share one register field, so packet I/O is broken while RSSI mode
  /// is on - this is not an optional nicety, it must be turned back off.
  void set_rssi_mode(bool enable) override;

  /// Transition to Standby mode. Returns true if state reached within timeout.
  bool go_standby() override;

  /// Transition to Sleep mode - the deep park between poll cycles.
  bool go_sleep() override;

  /// One-time wiring self-test for the GPIO3/INT2 line. Temporarily routes
  /// INT2 = RX_ACTIVE (high only while the chip is in RX), reads the pin in
  /// STBY (expect low) and in RX (expect high), then restores INT2 = RX_FIFO_TH.
  /// Returns true if the pin tracked the state (i.e. the chip's GPIO3 pad is
  /// wired correctly to the configured pin_gpio3). Logs the raw readings either way.
  bool self_test() override;

  /// Wiring and the carriers currently programmed, for the component's
  /// dump_config(). Stored configuration only - no SPI traffic, because ESPHome
  /// dumps a component even after it has been marked failed, i.e. possibly with
  /// no chip on the other end of the bus.
  void dump_config() override;

 private:
  /// Check if the chip is responding (read product ID register).
  bool is_chip_connected();

  /// Read current chip state from MODE_STA register.
  uint8_t get_state();

  /// Wait for a specific state with timeout. Returns true if reached.
  bool wait_for_state(uint8_t target_state, uint32_t timeout_ms = STATE_POLL_TIMEOUT_MS);

  /// Resolve a carrier to a NARTIS_FREQ_CHANNELS row, or NUM_CHANNELS if none is
  /// within FREQ_MATCH_TOLERANCE_HZ. `want_rx_half` picks which half of the row
  /// the carrier is compared against: the TX-half IS the carrier, the RX-half is
  /// the carrier plus the low-IF offset.
  uint8_t resolve_channel_(uint32_t freq_hz, bool want_rx_half) const;

  /// Fill a 4-byte frequency half from the AN199 formula. Fallback path only -
  /// see the comment at its definition for why this is not the normal route.
  static void compute_freq_half_(uint32_t lo_hz, bool rx_half, uint8_t *out4);

  /// Park in standby if not already there, so the synthesiser registers can be
  /// written. Returns true if the chip is (now) in standby.
  bool ensure_standby_for_freq_();

  /// Write a 4-byte half of the frequency bank. `rx_half` selects 0x18..0x1B
  /// (RX / local oscillator) versus 0x1C..0x1F (TX).
  void write_freq_half_(const uint8_t *bytes4, bool rx_half);

  void clear_fifo();

  /// Full merged-FIFO reset for RX: FIFO_RESTORE + clear RX/TX. Use after a
  /// TX->RX direction switch so RX doesn't see leftover TX bytes.
  void reset_rx_fifo_full();

  /// Set per-TX payload length (PKT14 high bits + PKT15 low byte).
  /// Must be written before every TX: the bank leaves the chip in fixed-length
  /// mode with PKT15=0xFF (511 bytes), so TX_DONE never fires for shorter packets.
  void set_tx_payload_length(uint16_t len);

  /// Restore PAYLOAD_LENG to the large bank default (511) before RX.
  /// set_tx_payload_length() shrinks it to the exact TX size before each
  /// transmit; if left there, the packet engine caps the *next* RX at that
  /// size, truncating every incoming frame to (TX_len + 1) bytes. Restoring the
  /// ceiling lets the chip read the on-air length byte and accept the full
  /// variable-length frame. Called from begin_rx() before GO_RX.
  void set_rx_payload_length();

  /// Clear all interrupt flags.
  void clear_interrupt_flags();

  /// Set the INT2 source field of REG_INT2_CTL. INT2 is routed to the GPIO3
  /// pad we poll for the FIFO-threshold signal (TX_FIFO_TH during TX,
  /// RX_FIFO_TH during RX).
  void set_int_source(uint8_t source);

  /// Prepare the SDIO pin + FIFO control for an RX session. Called by begin_rx()
  /// before entering RX mode.
  void prepare_rx_session();

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

  void write_reg(uint8_t addr, uint8_t val);
  void write_bank(uint8_t start_addr, const uint8_t *data, size_t len);

  /// Read-modify-write: (reg & ~mask) | (val & mask).
  void update_reg(uint8_t addr, uint8_t mask, uint8_t val);

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

  // Configured GPIO pin objects (used for setup() + pin_mode at init).
  esphome::InternalGPIOPin *sdio_pin_{nullptr};
  esphome::InternalGPIOPin *sclk_pin_{nullptr};
  esphome::InternalGPIOPin *csb_pin_{nullptr};
  esphome::InternalGPIOPin *fcsb_pin_{nullptr};
  esphome::InternalGPIOPin *gpio3_pin_{nullptr};
  // Fast non-virtual handles for the bit-bang inner loop (via pin->to_isr()).
  esphome::ISRInternalGPIOPin sdio_;
  esphome::ISRInternalGPIOPin sclk_;
  esphome::ISRInternalGPIOPin csb_;
  esphome::ISRInternalGPIOPin fcsb_;
  esphome::ISRInternalGPIOPin gpio3_;

  bool initialized_{false};
  /// Set by a begin_rx() that confirmed STA_RX, cleared by anything that leaves
  /// RX. This is what lets drain_rx() answer RX_STATUS_NOT_IN_RX honestly without
  /// adding an SPI state read to every poll of the loop.
  bool in_rx_{false};
  // Carriers currently programmed, Hz. Default to channel 0 both halves, which
  // is what init()'s bank write puts in the chip.
  uint32_t tx_freq_hz_{CHANNEL_CARRIER_HZ[0]};
  uint32_t rx_freq_hz_{CHANNEL_CARRIER_HZ[0]};
};

}  // namespace esphome::nartis_rf_meter
