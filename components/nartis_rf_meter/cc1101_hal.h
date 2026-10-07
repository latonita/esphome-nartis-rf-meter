/*
 * TI CC1101 hardware abstraction layer - bit-bang 4-wire SPI.
 *
 * The second RadioHal backend for the RF433-1 (Nartis D101) link. Everything above
 * the radio is unchanged; this file only has to put the PHY of fw/RF_PHY_SPEC.md on
 * the air: 2-FSK, 2400 bps NRZ, +-6 kHz deviation, MSB-first, 0x55 preamble, 4-byte
 * sync 55 55 F6 72 on air, no whitening / Manchester / FEC / PHY CRC.
 *
 * ============================================================================
 * THE ONE THING TO GET RIGHT WHEN COMPARING THIS WITH THE SIBLING PROJECTS
 * ============================================================================
 * The nartis_rf_2_meter and uart_nartis_rf components have a file with this name,
 * this layout and most of this code - but they speak RF433-2, which is LSB-FIRST on
 * air. Their transmit() runs RadioHal::reverse8() over every payload byte, their
 * drain_rx() reverses it back, and their SYNC1/SYNC0 hold bit-reversed bytes.
 *
 * RF433-1 is MSB-FIRST ("Payload Bit Order = Start from msb" in the real CIU's RFPDK
 * profile, fw/d101-rfpdk/a.exp), which is the CC1101's native shift order. So:
 * payload bytes go to the FIFO verbatim, come back verbatim, and SYNC1/SYNC0 hold
 * 0xF6 0x72 literally. There is no call to reverse8() anywhere in this backend and
 * there must not be one. Adding it "for symmetry" with the siblings would put a
 * radio on the air that nothing can decode.
 * ============================================================================
 *
 * Differences from the CMT2300A backend in this same component:
 *
 *   - The CC1101's FIFO reports an exact byte count over SPI (RXBYTES), so
 *     rx_chunk_size() is 1 and the component's sub-threshold-tail machinery never
 *     fires. The CMT2300A backend needs a GPIO threshold line because its SPI FIFO
 *     flag register reads 0xFF on this board; nothing of that kind is needed here.
 *   - Because the FIFO state IS readable, drain_rx() can report OVERFLOW and
 *     IO_ERROR, which the CMT2300A backend cannot. The component already consumes
 *     both through RX_POLL_FAULT and radio_error_count_.
 *   - The 4-byte on-air sync word is 55 55 F6 72, and its leading 55 55 is byte-
 *     identical to the preamble, so the CC1101's 16-bit SYNC1/SYNC0 covers it with
 *     nothing left over: SYNC1/SYNC0 = F6 72 at SYNC_MODE = 010 (16 of 16, exact),
 *     NUM_PREAMBLE = 12, and no bytes are carried through the FIFO in either
 *     direction. See the long sync-word note in cc1101_defs.h for how that byte
 *     order is established, why it contradicts fw/RF_PHY_SPEC.md sec. 4.4, and why a
 *     32-bit sync word is not implementable on this part regardless.
 *   - RF433-1 transmits and receives on DIFFERENT carriers in alternative-channel
 *     mode, so set_frequency() and set_rx_frequency() are independent and both are
 *     reprogrammed on every channel hop. The CC1101 is a true-frequency radio:
 *     both are programmed as the carrier, with NO IF offset. CHANNEL_LOW_IF_HZ in
 *     radio_hal.h is CMT2300A context and must not be applied here.
 *   - Frames reach ~292 bytes, far past what an 8-bit PKTLEN can name, so TX uses
 *     the infinite-to-fixed length transition (TI DN500). See transmit().
 *
 * Pins: SCLK, MISO (SO), MOSI (SI), CS (CSn) and an optional GDO0. The bus is
 * bit-banged rather than going through ESPHome's `spi` component, for the same
 * reason the CMT2300A backend does: the radio backends stay self-contained, so a
 * YAML that uses one of them pulls in nothing from the other and no shared bus
 * arbitration can interleave with the half-duplex request/reply timing.
 *
 * ---------------------------------------------------------------------------
 * RISK 1 - link budget, but a mild one. The CC1101 tops out at about +10 dBm at
 * 433 MHz, while a real D101 CIU transmits +14 dBm (RF_PHY_SPEC.md sec. 4.9). That is a
 * ~4 dB deficit on the uplink - noticeably better than the sibling RF433-2 projects,
 * which are ~10 dB down on a +20 dBm CIU. It is still the direction to suspect first
 * if the meter hears a real CIU but not us; the cure is a module with an external PA,
 * not a larger `output_power`.
 *
 * RISK 2 - this backend is UNPROVEN ON HARDWARE. Every number in it is derived from
 * the SDR-measured PHY and the RFPDK export that matches the real CIU's SPI dump
 * byte for byte, but no frame has yet been exchanged with a meter over a CC1101.
 * The CMT2300A path is the one proven against live meters.
 *
 * Worth knowing for bring-up: the documented failure mode of the CMT2300A backend on
 * THIS protocol is not demodulation - h ~= 5 is an easy waveform and the CMT's
 * counting CDR handles it - it is association without authentication. If the CC1101
 * also hears replies but the exchange still does not complete, the problem is above
 * the radio and swapping silicon will not move it.
 * ---------------------------------------------------------------------------
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "cc1101_defs.h"
#include "helpers.h"  // MAX_RF_FRAME_SIZE - the component's own frame ceiling
#include "radio_hal.h"

#include "esphome/core/gpio.h"

namespace esphome::nartis_rf_meter {

class Cc1101Hal : public RadioHal {
 public:
  const char *radio_name() const override { return "CC1101"; }

  /// Which MDMCFG2.MOD_FORMAT the chip runs. The two are equivalent on receive -
  /// MOD_FORMAT only shapes the transmitted waveform, and a GFSK signal demodulates
  /// in a 2-FSK receiver and vice versa - so this really is a transmit-side choice.
  enum class Modulation : uint8_t {
    FSK2,  // MOD_FORMAT = 000, un-shaped 2-FSK
    GFSK,  // MOD_FORMAT = 001, Gaussian-shaped, BT = 0.5
  };

  /// Set GPIO pins. Must be called before init(). Not part of RadioHal: the
  /// bit-banged 4-wire bus is this chip's own business. `gdo0` is optional and may
  /// be nullptr - it only buys a faster, exact TX-done edge; without it the driver
  /// polls MARCSTATE and TXBYTES instead and behaves identically.
  void set_pins(esphome::InternalGPIOPin *sclk, esphome::InternalGPIOPin *miso, esphome::InternalGPIOPin *mosi,
                esphome::InternalGPIOPin *cs, esphome::InternalGPIOPin *gdo0);

  /// Where begin_rx() centres the receiver, in Hz relative to the channel's RX
  /// carrier. Defaults to 0: the chip's frequency offset compensation reaches
  /// +-BW/4 = +-25 kHz at the default bandwidth, which covers the +-17.4 kHz a
  /// worst-case 20 ppm + 20 ppm crystal pair can produce at 433.8 MHz, so there is
  /// normally nothing to correct by hand.
  void set_rx_offset_hz(int32_t hz) { this->rx_offset_hz_ = hz; }

  /// Requested PA output power in dBm, snapped to the nearest 433 MHz PATABLE entry
  /// (-30, -20, -15, -10, 0, +5, +7, +10). Defaults to +10, the chip's maximum.
  ///
  /// A real D101 CIU transmits +14 dBm, so even at +10 the uplink is ~4 dB short of
  /// the reference link budget. Lowering this value makes that strictly worse; if
  /// the meter cannot hear us, the answer is a module with an external PA.
  void set_output_power_dbm(int8_t dbm) { this->output_power_dbm_ = dbm; }

  /// RX channel-filter bandwidth in Hz, snapped to the nearest (CHANBW_E, CHANBW_M).
  /// Defaults to 101562 Hz, which is sized by the crystal-offset budget rather than
  /// by the signal: the reply occupies only ~17 kHz (Carson, +-6 kHz at 2400 bps),
  /// but frequency offset compensation only reaches +-BW/4. The chip's range is
  /// 58035 Hz to 812500 Hz. See CC1101_DEFAULT_RX_BANDWIDTH_HZ.
  void set_rx_bandwidth_hz(uint32_t hz) { this->rx_bandwidth_hz_ = hz; }

  /// FSK deviation in Hz, snapped to the nearest (DEV_E, DEV_M). Defaults to 6000,
  /// which the chip rounds to +-5950 Hz (DEVIATN = 0x17). Used for TX and RX alike.
  ///
  /// NOTE for anyone comparing with fw/RF_PHY_SPEC.md: its "DEVIATN ~= 0x35" hint is
  /// wrong (0x35 is 20629 Hz). The solver below finds 0x17 from 6000 Hz on its own.
  /// See CC1101_DEFAULT_DEVIATION_HZ for the full arithmetic.
  void set_deviation_hz(uint32_t hz) { this->deviation_hz_ = hz; }

  /// Transmit waveform shaping. Defaults to plain 2-FSK, which RF_PHY_SPEC.md sec. 5
  /// recommends and which the SDR capture confirms the on-air signal demodulates as.
  ///
  /// The real CIU nevertheless shapes its transmission: a.exp records
  /// "Demodulation = GFSK" with "Gaussian BT = 0.5", and the CC1101's GFSK is
  /// BT = 0.5 as well, so `gfsk` reproduces the reference transmitter exactly. It is
  /// exposed because a TX-range problem is the one place the difference could matter
  /// - GFSK puts the same power into a narrower spectrum, which can help a meter
  /// whose receive filter is tighter than ours. It cannot help RECEIVE: MOD_FORMAT
  /// shapes the transmitted waveform only, and the two demodulate identically.
  void set_modulation(Modulation mod) { this->modulation_ = mod; }

  /// FOCCFG.FOC_BS_CS_GATE - a HARDWARE BRING-UP KNOB, not a setting with a known
  /// right answer. It decides whether the frequency-offset and bit-synchronisation
  /// loops are allowed to run before carrier sense asserts:
  ///
  ///   false (default) -> FOCCFG = 0x16, ungated. Both loops track from the instant
  ///     RX is entered. Nothing of a weak preamble is wasted waiting for CS, which is
  ///     why it is the default - but the loops also spend the whole reply wait
  ///     tracking receiver noise, and can walk out to their limits (+-BW/4 for FOC,
  ///     i.e. +-25 kHz at the default bandwidth) before the real preamble arrives.
  ///   true -> FOCCFG = 0x36, gated. Both loops are frozen at their initial values
  ///     until carrier sense asserts, so they start from zero offset on the real
  ///     signal - at the cost of the preamble bytes burned while CS makes up its
  ///     mind. The meter sends 10 preamble bytes (33 ms at 2400 baud), so there is
  ///     more to spend here than on the sibling RF433-2 link, but not unlimited.
  ///
  /// Which one wins is not answerable from the datasheet, and the receive waits on
  /// this protocol are long (seconds between request and reply during pairing), which
  /// is exactly when an ungated loop has time to wander. Decide it by measurement:
  /// count CRC-valid replies with each value over many cycles. Bytes landing in the
  /// FIFO are not the test - a slipped loop fills the FIFO too, with garbage.
  /// Everything else in FOCCFG is identical either way; see CC1101_FOCCFG_BASE.
  void set_foc_cs_gate(bool enabled) { this->foc_cs_gate_ = enabled; }

  /// The 16 bits the correlator matches, high byte into SYNC1 and low byte into
  /// SYNC0, i.e. 0xF672 puts F6 on the air first and 72 second.
  ///
  /// This is settable - and the YAML key `sync_word` exists - for one reason: the
  /// on-air byte order of this PHY's sync word is DERIVED, not observed. It follows
  /// from the CMOSTEK AN143 SYNC_VALUE fill rule applied to the real CIU's register
  /// capture, and is cross-validated against the hardware-proven RF433-2 driver in
  /// the sibling projects, but no RF433-1 SDR decode has confirmed it and
  /// fw/RF_PHY_SPEC.md sec. 11 still lists it as an open item. It also CONTRADICTS
  /// that spec's sec. 4.4 and fw/FRAME_HEADER_SPEC.md:823, which print "72 F6 55 55"
  /// - the register listing read backwards. The full argument is the sync-word note
  /// in cc1101_defs.h.
  ///
  /// So: if the meter never answers and nothing is ever received, `sync_word: 0x72F6`
  /// is the one alternative worth trying, and it costs a line of YAML rather than a
  /// rebuild. Nothing else about the driver changes with it - the two bytes are
  /// programmed into the correlator and nowhere else.
  void set_sync_word(uint16_t sync_word) { this->sync_word_ = sync_word; }

  /// TRANSMIT carrier, Hz. Runtime-settable: the component re-tunes between
  /// exchanges when it hops channels. Programmed into FREQ2/1/0 by transmit(), which
  /// has already parked the chip in IDLE - the synthesiser registers cannot be
  /// changed under way.
  ///
  /// This is the CARRIER. The CC1101 is a true-frequency radio and no IF offset of
  /// any kind is applied; CHANNEL_LOW_IF_HZ in radio_hal.h describes the CMT2300A's
  /// low-IF receiver and has nothing to do with this part.
  void set_frequency(uint32_t freq_hz) override;

  /// RECEIVE carrier, Hz. Independent of the transmit carrier, because RF433-1's
  /// alternative channel mode holds TX on 433.82 MHz (the meter's wake frequency)
  /// while the meter replies on the advertised channel's carrier. Programmed by
  /// begin_rx(), with set_rx_offset_hz() added.
  void set_rx_frequency(uint32_t freq_hz) override;

  /// Power-up reset, probe PARTNUM/VERSION, then write the whole register profile
  /// and PATABLE. False means the chip is unusable; the reason is logged.
  bool init() override;

  /// Transmit one logical RF433-1 frame, which per radio_hal.h starts at its LEN
  /// byte and carries no preamble or sync of its own. The frame is the only thing
  /// that goes through the FIFO - preamble and the whole sync word come from the
  /// packet engine. Blocks until the packet is out or the frame's own airtime budget
  /// elapses, and refills the FIFO for frames longer than its 64 bytes. Returns true
  /// only with positive evidence that the chip really entered TX. Parks the chip in
  /// IDLE on every exit path.
  bool transmit(const uint8_t *frame, size_t len) override;

  /// Start a verified receive epoch: SIDLE aborts whatever was in flight (and clears
  /// a wedged RXFIFO_OVERFLOW, which is where this chip latches that fault), then the
  /// RX carrier (channel + rx_offset_hz) and infinite-length mode are programmed, the
  /// RX FIFO is flushed and SRX is strobed and confirmed in MARCSTATE.
  bool begin_rx() override;

  /// Drain whatever the RX FIFO holds. Never takes the final byte while the chip is
  /// still receiving - a documented CC1101 FIFO hazard. Drops this epoch's sync-tail
  /// bytes before anything reaches the caller, so the first byte the component sees
  /// is the frame's LEN byte. Returns bytes appended and sets `status`; this backend
  /// can report an overflow and a chip that has left RX, both of which end the epoch.
  size_t drain_rx(uint8_t *buf, size_t buf_size, RxStatus &status) override;

  /// Read up to `len` bytes straight out of the RX FIFO with no hold-back.
  ///
  /// With rx_chunk_size() == 1 the component's sub-threshold-tail branch can never
  /// fire, so the only caller left is its drain-stall fallback - and that one fires
  /// precisely when drain_rx() is holding a byte back under the errata rule. Holding
  /// it back here too would strand the last byte of the frame forever, so this one
  /// reads exactly what is asked for. The errata hazard is accepted knowingly: by the
  /// time the component gives up on the drain the frame is over, which is when the
  /// hazard window is closed anyway.
  size_t read_rx_exact(uint8_t *buf, size_t len) override;

  /// 1: RXBYTES is an exact count, so there is no minimum run this backend cannot
  /// hand over. This is what makes the component's sub-threshold-tail branch dead
  /// code on this radio (see poll_rx_()).
  size_t rx_chunk_size() const override { return 1; }

  /// The most one drain_rx() call can append. RXBYTES counts the 64 FIFO bytes plus
  /// the one byte held in the receive chain's prefetch register, so its largest legal
  /// value is 65 (see CC1101_RXBYTES_MAX_VALID); a drain made while the chip is still
  /// receiving leaves one byte behind, which puts the peak at exactly 64. drain_rx()
  /// clamps to this same constant on the one path that could otherwise exceed it - a
  /// 65-byte count read after the chip has already left RX - so the number here is
  /// the true bound and not an estimate. Dropping sync-tail bytes can only make a
  /// particular call return fewer.
  size_t max_drain_chunk() const override { return CC1101_FIFO_SIZE; }

  int8_t get_rssi_dbm() override;

  bool go_standby() override;

  /// Wiring and the PHY parameters actually programmed, for the component's
  /// dump_config(). Printed from solved member state rather than read back off the
  /// chip: ESPHome dumps a component even after mark_failed(), so this may run with
  /// nothing answering on the bus.
  void dump_config() override;

  /* go_sleep() is deliberately NOT overridden, so RadioHal's default forwards it to
   * go_standby() = IDLE. The CC1101 does have a deeper state (SPWD), and this
   * component idles for minutes between poll cycles, so the saving would be real -
   * but SPWD stops the crystal, needs a CS-toggle wake-up handshake, loses PATABLE
   * entries 1-7 and costs a full recalibration on the way out. None of that has been
   * exercised on hardware for this backend, and getting the wake sequence subtly
   * wrong presents as "the meter stopped answering" long after the cycle that broke
   * it. IDLE keeps the whole register profile alive and is always correct. This is a
   * deliberate deferral, not an oversight.
   *
   * self_test() is likewise not overridden. The CMT2300A backend needs one because
   * the GPIO3/INT2 line is its ONLY receive-side signal and a mis-wire there is
   * fatal-but-silent; on the CC1101 every receive-side signal is readable over SPI,
   * GDO0 is optional, and init() already proves both bus directions (PARTNUM/VERSION
   * prove MISO, the SYNC1 read-back proves MOSI). There is nothing left for a
   * self-test to find. */

 private:
  /// Half a bit-bang SPI clock period. 2 us gives ~250 kHz, matching the CMT2300A
  /// backend; the CC1101 tolerates up to 6.5 MHz, so this is pure margin.
  static constexpr uint32_t SPI_HALF_PERIOD_US = 2;
  /// How long to wait for CHIP_RDYn (MISO low) after CS goes low.
  static constexpr uint32_t CHIP_RDY_TIMEOUT_US = 10000;
  /// How long the chip may take to come back from SRES.
  static constexpr uint32_t RESET_TIMEOUT_US = 50000;
  /// Status registers can change mid-read; this caps the re-read loop.
  static constexpr uint8_t STATUS_READ_RETRIES = 6;

  /// Largest logical frame transmit() will accept, tied to the component's own frame
  /// buffer so the two cannot drift apart.
  ///
  /// There is no 8-bit cliff here, which is the whole point of the infinite-to-fixed
  /// transition in transmit(): PKTLEN is eight bits, but a frame does not have to fit
  /// in it. The ceiling that does exist is the component's: tx_buf_ is
  /// MAX_RF_FRAME_SIZE and rx_accum_buf_ is the same, and nothing above the radio can
  /// produce or parse more.
  ///
  /// What the frame builder can actually hand us, for calibration: the largest
  /// RfDataLayer::build_frame() envelope is 15 header + 6 encryption prefix + 255
  /// ciphertext + 12 GCM tag + 4 dual CRC = 292 bytes, which is the "~290 B" figure
  /// in RF_PHY_SPEC.md sec. 4.5/sec. 7. Note that the protocol's OWN length byte
  /// (frame[0] = total - 1, eight bits) caps a WELL-FORMED frame at 256 bytes, so
  /// anything between 257 and 292 would already be malformed before it reached this
  /// backend - the radio is not the thing that would be rejecting it. The limit here
  /// is set to the buffer size rather than to 256 so that this backend never becomes
  /// the narrowest part of the chain.
  static constexpr size_t TX_FRAME_MAX = MAX_RF_FRAME_SIZE;
  /// Above this many FIFO bytes, PKTLEN cannot name the packet on its own and
  /// transmit() uses the infinite-to-fixed transition. 255 is the largest value an
  /// 8-bit PKTLEN expresses as itself.
  static constexpr size_t TX_PLAIN_FIXED_MAX = 255;
  /// Refill the TX FIFO once it has drained to this many bytes. Mirrors the
  /// FIFOTHR = 7 split (33 bytes in TX) programmed in CC1101_CONFIG.
  static constexpr uint8_t TX_REFILL_LOW_WATER = 32;
  /// Time allowed for the chip to report TX entry. Entry costs a calibration
  /// (~720 us) plus settling, and the shortest frame this PHY can send is still 14
  /// byte-times of preamble and sync, i.e. ~47 ms, so 20 ms is far too short for a
  /// packet to have come and gone inside it: failing to observe TX here really does
  /// mean TX was not entered.
  static constexpr uint32_t TX_ENTER_TIMEOUT_MS = 20;
  /// Slack added to a frame's computed airtime to get its TX-done deadline. It has
  /// to cover the IDLE->CALIBRATE->SETTLING->TX walk, every bit-banged FIFO refill,
  /// and whatever else ESPHome's loop is doing; 300 ms is more than an order of
  /// magnitude over the measured cost of all three and still fails fast.
  static constexpr uint32_t TX_DONE_MARGIN_MS = 300;
  /// Time allowed for a state-machine transition that is not on the air path.
  static constexpr uint32_t STATE_TIMEOUT_MS = 20;
  static constexpr uint32_t STATE_POLL_INTERVAL_US = 100;

  /// How long a frame of `fifo_bytes` may take, preamble and sync included, at the
  /// programmed symbol rate, plus TX_DONE_MARGIN_MS. `fifo_bytes` is what actually
  /// goes through the FIFO, which is the frame and nothing else.
  ///
  /// A fixed constant cannot work here: TX_FRAME_MAX is 512 bytes, which with 12
  /// preamble and 2 sync bytes is 526 * 8 / 2398.97 = 1.75 s of airtime, so any
  /// one-size deadline short enough to be useful for a 40-byte beacon would guarantee
  /// a timeout on a long reply - transmit() would be rejecting lengths it had just
  /// accepted. The arithmetic is integer milli-baud and rounds up, so the computed
  /// figure never lands under the true airtime.
  ///
  /// Static, unlike the sibling projects' version of this helper: they fold in a YAML
  /// `rf_tx_timeout` floor at this point, and this component has no such knob - its
  /// rx_timeout/rx_reply_timeout govern receiving only. If one is ever added, this is
  /// the single place the floor belongs.
  static uint32_t tx_done_timeout_ms_(size_t fifo_bytes);

  // --- bit-banged 4-wire SPI (mode 0, MSB-first) ---
  uint8_t xfer_byte_(uint8_t out);
  bool wait_miso_low_(uint32_t timeout_us);
  /// Drive CS low and wait for CHIP_RDYn. Reports a readiness failure by raising
  /// spi_not_ready_, not by a return value - see the note on that member.
  void select_();
  void deselect_();
  uint8_t strobe_(uint8_t cmd);
  void write_reg_(uint8_t addr, uint8_t value);
  void write_burst_(uint8_t addr, const uint8_t *data, size_t len);
  uint8_t read_reg_(uint8_t addr);
  uint8_t read_burst_reg_once_(uint8_t addr);
  uint8_t read_status_reg_(uint8_t addr);
  void read_fifo_burst_(uint8_t *data, size_t len);

  static void pin_high(esphome::ISRInternalGPIOPin &pin) { pin.digital_write(true); }
  static void pin_low(esphome::ISRInternalGPIOPin &pin) { pin.digital_write(false); }
  static bool pin_read(esphome::ISRInternalGPIOPin &pin) { return pin.digital_read(); }

  /// Sticky "the chip was never ready" flag, raised by select_() whenever CHIP_RDYn
  /// (MISO low) did not appear inside CHIP_RDY_TIMEOUT_US. TI requires SO to be low
  /// before the header byte is clocked, because that is the chip saying its SPI
  /// interface and crystal are up; clocking a transaction anyway can return arbitrary
  /// register values, execute a strobe that was never intended, or drive a state
  /// transition that never happened. None of that is detectable downstream, so it
  /// must not be thrown away.
  ///
  /// It is a sticky flag rather than a checked return on every accessor on purpose:
  /// write_reg_() is called forty-odd times from write_config_() alone, and
  /// threading a status out of each one would bury the register profile in error
  /// handling for a fault that is all-or-nothing in practice - if the chip is not
  /// answering CHIP_RDYn, it is not answering it for any of them. Callers that care
  /// clear it before a sequence and test it after.
  bool spi_not_ready_{false};

  /// Whether begin_rx() actually confirmed the chip in RX. Cleared before anything
  /// in begin_rx() can fail and again by go_idle_() (the one funnel every exit from
  /// RX passes through), set only on positive evidence. drain_rx() refuses to hand
  /// over bytes while it is clear: without it, a begin_rx() that failed before
  /// reaching SFRX leaves an unflushed FIFO and a chip possibly still in RX from the
  /// previous epoch, and the stale bytes in it would be drained as the head of the
  /// next reply - the desync the begin_rx() contract in radio_hal.h exists to stop.
  /// The component relies on this: it does not escalate a false return from
  /// begin_rx(), it waits for the next drain to report NOT_IN_RX.
  bool rx_epoch_armed_{false};
  void clear_spi_error_() { this->spi_not_ready_ = false; }
  bool spi_error_() const { return this->spi_not_ready_; }

  uint8_t marcstate_();
  bool wait_marcstate_(uint8_t target, uint32_t timeout_ms);
  bool go_idle_();

  /// Write `count` bytes of `frame`, starting at `offset`, into the TX FIFO, clamped
  /// to what `frame` actually holds. Nothing is prepended and nothing is staged: the
  /// caller's frame goes to the FIFO verbatim, which is what lets a 512-byte frame be
  /// transmitted with no staging buffer at all, and is possible only because RF433-1
  /// is MSB-first and needs no per-byte transformation on the way out.
  void write_tx_chunk_(const uint8_t *frame, size_t frame_len, size_t offset, size_t count);

  // --- configuration ---
  void solve_params_();
  /// FREQ = f * 2^16 / Fxosc and its inverse. Static and shared so program_frequency_()
  /// and dump_config() can never disagree about what was programmed.
  static uint32_t freq_word_for_(uint32_t hz);
  static uint32_t freq_hz_for_word_(uint32_t word);
  void program_frequency_(uint32_t hz);
  void write_config_();
  /// The RX carrier actually tuned: the channel's RX carrier plus the YAML offset,
  /// clamped to a positive frequency so a wild offset cannot wrap the subtraction.
  uint32_t rx_tuned_hz_() const;
  uint8_t mdmcfg2_value_() const;

  esphome::InternalGPIOPin *sclk_pin_{nullptr};
  esphome::InternalGPIOPin *miso_pin_{nullptr};
  esphome::InternalGPIOPin *mosi_pin_{nullptr};
  esphome::InternalGPIOPin *cs_pin_{nullptr};
  esphome::InternalGPIOPin *gdo0_pin_{nullptr};
  esphome::ISRInternalGPIOPin sclk_;
  esphome::ISRInternalGPIOPin miso_;
  esphome::ISRInternalGPIOPin mosi_;
  esphome::ISRInternalGPIOPin cs_;
  esphome::ISRInternalGPIOPin gdo0_;

  // --- requested parameters (YAML) ---
  uint32_t freq_hz_{CC1101_DEFAULT_FREQUENCY_HZ};
  uint32_t rx_freq_hz_{CC1101_DEFAULT_FREQUENCY_HZ};
  int32_t rx_offset_hz_{CC1101_DEFAULT_RX_OFFSET_HZ};
  int8_t output_power_dbm_{CC1101_DEFAULT_OUTPUT_POWER_DBM};
  uint32_t rx_bandwidth_hz_{CC1101_DEFAULT_RX_BANDWIDTH_HZ};
  uint32_t deviation_hz_{CC1101_DEFAULT_DEVIATION_HZ};
  Modulation modulation_{Modulation::FSK2};
  bool foc_cs_gate_{false};
  /// SYNC1:SYNC0, high byte first on air. See set_sync_word().
  uint16_t sync_word_{CC1101_SYNC_WORD_DEFAULT};

  // --- solved register fields and the values they actually produce, so
  //     dump_config() can print what was programmed rather than what was asked for ---
  uint8_t chanbw_e_{0};
  uint8_t chanbw_m_{0};
  uint32_t rx_bandwidth_actual_hz_{0};
  uint8_t dev_e_{0};
  uint8_t dev_m_{0};
  uint32_t deviation_actual_hz_{0};
  uint8_t pa_value_{0};
  int8_t output_power_actual_dbm_{0};
  /// The characterised TEST2/TEST1 pair for rx_bandwidth_actual_hz_. Solved rather
  /// than tabled so dump_config() can print the bytes that were actually written.
  uint8_t test2_{CC1101_TEST2_NARROW_BW};
  uint8_t test1_{CC1101_TEST1_NARROW_BW};

  /// FOCCFG as the gate knob resolves it. Derived on demand rather than cached in a
  /// member: dump_config() runs even when init() never did, and a cached copy would
  /// then print the default while the setter said otherwise.
  uint8_t foccfg_value_() const {
    return (uint8_t) (CC1101_FOCCFG_BASE | (this->foc_cs_gate_ ? CC1101_FOCCFG_BS_CS_GATE : 0u));
  }
};

}  // namespace esphome::nartis_rf_meter
