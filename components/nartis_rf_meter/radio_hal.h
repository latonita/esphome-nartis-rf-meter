/*
 * Radio hardware abstraction for the RF433-1 (Nartis D101) link.
 *
 * The RF433-1 PHY is transceiver-agnostic:
 * 2-FSK in the 433.3 - 434.7 MHz band, 0x55 preamble, 4-byte sync word
 * 55 55 F6 72 on air, fixed-length-in-front framing, no hardware CRC. Everything above
 * the radio - the length-prefixed RF envelope, the software CRC-16/DNP
 * (poly 0x3D65), the AES-128-GCM layer and the DLMS/COSEM payload inside it -
 * is radio independent, so a backend only has to carry opaque frames.
 *
 * A frame crossing this interface is already complete and in the LOGICAL domain,
 * MSB-first: `LEN TYPE ... CRCL CRCH` on transmit, and what comes back from
 * drain_rx()/read_rx_exact() starts at the same LEN byte. Preamble and sync are
 * the backend's to emit and to detect - a hardware packet engine on the
 * CMT2300A, whatever a future backend has - and never reach the caller. Any
 * bit-order flip between the logical domain and the wire is each backend's own
 * business.
 *
 * (On the sync word's byte order: it is "72 F6 55 55" in fw/RF_PHY_SPEC.md sec. 4.4
 * and fw/FRAME_HEADER_SPEC.md:823, which is the CMT2300A PKT10..PKT13 register
 * listing read as if it were an air sequence. It is not. The derivation, the
 * cross-checks and the consequences are in the sync-word note in cc1101_defs.h; those
 * two spec files should be read as carrying a documentation bug here.)
 *
 * Backends: Cmt2300aHal (cmt2300a_hal.h).
 *
 * This interface is deliberately the same shape as the one in the sibling
 * nartis_rf_2_meter component, so a reader can compare the two: the first block
 * of methods below is that interface verbatim. RF433-1 is the older and more
 * demanding protocol of the two, and the six additions after it are the places
 * where it needs more of the radio than RF433-2 does; each one says why.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome::nartis_rf_meter {

/* ================================================================
 * Channel grid
 * ================================================================
 *
 * RF433-1 has a fixed 4-entry channel table. The index is not a local choice:
 * it is advertised to the meter in frame[12] bits 7:6 and COMMANDS which carrier
 * the meter answers on, so the index we advertise and the carrier we listen on
 * must agree (see handle_channel_select_()).
 *
 * The numbers below are the CARRIERS, in Hz. They were verified against the
 * proven register blobs in cmt2300a_defs.h (NARTIS_FREQ_CHANNELS) by running
 * the CMOSTEK AN199 synthesiser formula forward - XTAL 26 MHz, divider 4,
 * N = word>>20, K = word & 0xFFFFF, word = floor(f_LO * 4 / 26e6 * 2^20) - over
 * these four values: it reproduces all 32 bytes of that table exactly. They are
 * therefore a restatement of the measured blobs, not an independent guess.
 *
 * IMPORTANT (low-IF receiver), carried over verbatim from cmt2300a_defs.h
 * because it is the single most load-bearing fact about this table: the
 * CMT2300A RX register set holds the LO, and the received frequency is
 * LO - IF, with IF = XTAL/92 = 282608 Hz (~280 kHz; it is exactly the spacing
 * between every standard channel's RX-half and TX-half register value). So to
 * RECEIVE a channel's carrier, the RX LO must sit one IF ABOVE it. The register
 * blobs encode the LO; this table encodes the carrier. A backend that is not a
 * low-IF receiver must not apply any offset of its own - it is handed the
 * carrier and should tune to the carrier.
 *
 * Two channel modes, which is what channel_tx_hz()'s `alternative` flag selects:
 *
 *   default mode      TX carrier = channel i's carrier, RX carrier = the same.
 *
 *   alternative mode  TX carrier = 433.82 MHz (channel 0) ALWAYS, RX carrier =
 *                     channel i's carrier. This is the MEASURED meter
 *                     reply-frequency behaviour, not a guess: the meter wakes on
 *                     433.82 and the CIU keeps transmitting there, while the
 *                     advertised channel index commands the meter to reply on
 *                     that channel's carrier, where we listen. (The earlier
 *                     "+/-1.7 MHz wander" reported for this link was pure SDR
 *                     ALIASING - e.g. ch1 433.30 showing up at 435.35 when
 *                     sampling 434.4 at 2.048 Msps.)
 *
 * Note what this means for the RX carrier: it is the SAME in both modes. Only
 * the TX carrier differs. That matches the register tables, where every
 * alternative-mode entry's RX-half is byte-identical to the standard entry's.
 */
static constexpr uint8_t NUM_CHANNELS = 4;

/// Carrier of each channel, Hz. Index = the channel number advertised to the meter.
static constexpr uint32_t CHANNEL_CARRIER_HZ[NUM_CHANNELS] = {
    433820000u,  // ch0 - the meter's wake frequency; also the alternative-mode TX carrier
    433300000u,  // ch1
    434260000u,  // ch2 - the default; the most reliable reply carrier observed
    434700000u,  // ch3
};

/// The CMT2300A's low-IF offset, Hz (XTAL/92). Not needed to USE this table -
/// set_rx_frequency() takes the carrier - but kept here with the table because
/// it is what explains why the register blobs look 280 kHz "wrong".
static constexpr uint32_t CHANNEL_LOW_IF_HZ = 282608u;

/// Carrier the meter replies on for channel `ch`, Hz. Same in both channel modes.
/// An out-of-range channel falls back to ch0 rather than reading past the table.
inline uint32_t channel_rx_hz(uint8_t ch) { return CHANNEL_CARRIER_HZ[(ch < NUM_CHANNELS) ? ch : 0]; }

/// Carrier we transmit on for channel `ch`, Hz. In alternative-channel mode that
/// is always ch0/433.82 - the frequency the meter wakes on - whatever channel is
/// advertised; in default mode it is the channel's own carrier.
inline uint32_t channel_tx_hz(uint8_t ch, bool alternative) {
  return alternative ? CHANNEL_CARRIER_HZ[0] : channel_rx_hz(ch);
}

/* ================================================================
 * Receive status
 * ================================================================ */

/* Why drain_rx() reports a status instead of just a count: a bare byte count makes
 * "nothing has arrived yet" and "the receiver is broken" the same answer, 0, and the
 * component then waits out its whole rx_timeout_ms_ and books the exchange as
 * "the meter did not answer". That is a misdiagnosis with a cost - it hides a radio
 * fault behind a link fault, and it burns a timeout per attempt where the attempt
 * was already lost. Every zero-byte return therefore says WHY it is zero. */
enum class RxStatus : uint8_t {
  RX_STATUS_DATA,       // bytes were appended
  RX_STATUS_NO_DATA,    // nothing has arrived yet; keep waiting
  RX_STATUS_OVERFLOW,   // RX FIFO overflowed; the backend recovered, the frame is lost
  RX_STATUS_IO_ERROR,   // the chip did not respond plausibly over SPI
  RX_STATUS_NOT_IN_RX,  // the radio is no longer receiving; the epoch is dead
};

class RadioHal {
 public:
  virtual ~RadioHal() = default;

  /// Chip name for logs, e.g. "CMT2300A". Never null.
  virtual const char *radio_name() const = 0;

  /// TRANSMIT carrier, in Hz.
  ///
  /// Divergence from the sibling component, which has only this one setter: there,
  /// the receive carrier is a fixed configured offset from the transmit carrier, so
  /// one frequency defines both and it only ever has to be set before init(). Here
  /// the two carriers are independent (see set_rx_frequency()), and BOTH may be
  /// changed at runtime between exchanges - channel hopping re-tunes mid-cycle -
  /// not only before init(). A backend must therefore handle being called while it
  /// is already configured and running, parking itself as its own hardware requires.
  virtual void set_frequency(uint32_t freq_hz) = 0;

  /// RECEIVE carrier, in Hz.
  ///
  /// RF433-1 can receive on a different channel than it transmits on. With the
  /// alternative channel table the CIU always transmits on 433.82 MHz - the
  /// meter's wake frequency - while the meter replies on the advertised channel's
  /// carrier. That is not an offset from the TX carrier; it is an independent,
  /// runtime-settable frequency, and it is why this method exists at all.
  ///
  /// Pass the CARRIER, not a local-oscillator setting: a low-IF backend applies
  /// its own IF offset internally (see CHANNEL_LOW_IF_HZ).
  virtual void set_rx_frequency(uint32_t freq_hz) = 0;

  /// Reset, probe and configure the chip. False means the radio is unusable and
  /// the component marks itself failed; the backend logs the reason.
  virtual bool init() = 0;

  /// Transmit one frame, blocking until the chip reports the packet sent or an
  /// internal timeout expires. Parks the chip in standby either way - on every exit
  /// path, including the early ones.
  ///
  /// True means the whole frame went out. A frame the chip truncated is a failure
  /// like any other: the meter heard something whose CRC cannot check out and will
  /// never answer it, so the caller must be free to resend rather than spend a
  /// receive timeout waiting on it.
  virtual bool transmit(const uint8_t *frame, size_t len) = 0;

  /// Start a fresh, VERIFIED receive epoch on this channel. The component calls it
  /// once after each transmit, and everything drained afterwards must belong to that
  /// one reply. "Switch to RX" is not enough: a sibling project in this repo had a
  /// late reply to a request that had already timed out sitting in the FIFO, where
  /// it was drained as the head of the NEXT reply and desynced every request/reply
  /// pair after it. So an implementation must, in this order:
  ///
  ///   - abort any transmit or receive still in progress;
  ///   - flush the hardware RX FIFO and any buffer the backend keeps in software;
  ///   - clear the latched overflow, sync, FIFO-threshold and packet-done interrupt
  ///     flags, so the first one seen afterwards was raised by this epoch;
  ///   - confirm the chip actually reached RX, and return false if it did not.
  ///
  /// False means no epoch was started: drain_rx() will then report RX_STATUS_NOT_IN_RX
  /// rather than a silent stream of zeroes.
  virtual bool begin_rx() = 0;

  /// Append whatever has been received to `buf`, never writing past `buf_size`, and
  /// return the number of bytes appended. `status` always says what the return value
  /// means - in particular WHY it is zero, since "nothing yet" and "this epoch is
  /// dead" demand opposite responses from the caller.
  ///
  /// Must return promptly rather than blocking for the packet to complete: the
  /// caller bounds the frame by its own LEN byte and polls from ESPHome's loop(). A
  /// backend that does know a packet has completed is free to say so through
  /// `status` - it just may not wait for it.
  ///
  /// No default argument on purpose: a default on a virtual binds to the static type
  /// of the expression, so an override could silently supply a different one.
  virtual size_t drain_rx(uint8_t *buf, size_t buf_size, RxStatus &status) = 0;

  /// Read exactly `len` bytes out of the receive FIFO, without waiting for any
  /// threshold signal, and return how many were actually read.
  ///
  /// Not in the sibling component's interface, because RF433-2 never needs the
  /// trailing bytes of a frame on their own. Here it is unavoidable: the CMT2300A's
  /// only receive-side signal on this board is the RX_FIFO_TH level on the
  /// GPIO3/INT2 pin - the SPI FIFO-flag registers read 0xFF here (Control2 bank)
  /// and are unusable - so a run of bytes shorter than the threshold never raises
  /// anything at all. The tail of every frame is such a run, and it can only be
  /// fetched by a caller that already knows the frame length, which the component
  /// does from the LEN byte.
  ///
  /// A backend whose FIFO reports an exact byte count implements this as an
  /// ordinary drain: there is nothing special about a short read on such a chip.
  virtual size_t read_rx_exact(uint8_t *buf, size_t len) = 0;

  /// The atomic granularity of drain_rx(): the SMALLEST run of bytes the backend can
  /// hand over. 15 on the CMT2300A (the FIFO threshold); 1 on a backend that can
  /// report an exact FIFO count.
  ///
  /// The component uses it to decide when a frame's remainder has shrunk below what
  /// drain_rx() will ever deliver, and that it must therefore switch to
  /// read_rx_exact(). This is NOT max_drain_chunk(), which is the opposite bound -
  /// the most one call can return, not the least.
  virtual size_t rx_chunk_size() const = 0;

  /// The largest number of bytes one drain_rx() call can append. The component uses
  /// it to tell "the buffer can still grow" from "the buffer is full", which is its
  /// end-of-reception fallback when a frame's LEN byte is not believable.
  virtual size_t max_drain_chunk() const = 0;

  /// RSSI of the last reception, dBm.
  virtual int8_t get_rssi_dbm() = 0;

  /// Put the chip into (or take it out of) whatever mode it needs for get_rssi_dbm()
  /// to mean anything. Default: do nothing.
  ///
  /// Not in the sibling component's interface. The CMT2300A multiplexes RSSI
  /// reporting and FIFO merging through the same SYS11 low-5-bit field, so
  /// get_rssi_dbm() returns garbage unless RSSI mode was turned on first, and packet
  /// I/O stays broken until it is turned off again - the RSSI survey this component
  /// runs before every cycle therefore has to bracket itself with this call. A chip
  /// with a free-running RSSI register needs nothing here, which is exactly why this
  /// is a no-op default rather than a pure virtual: a backend should not be forced
  /// to write an empty override to say "my RSSI always works".
  virtual void set_rssi_mode(bool enable) {}

  /// Park in the lowest state that keeps the configuration. False = the chip did
  /// not confirm the transition.
  virtual bool go_standby() = 0;

  /// Park deeper than standby, between poll cycles. Default: fall back to standby.
  ///
  /// Not in the sibling component's interface, which polls often enough that the
  /// extra state is not worth it. This component idles for minutes between cycles,
  /// so a deeper park is worth having - but the default is standby because standby
  /// is always correct, just slower to leave and hungrier while parked. A backend
  /// with no deeper state simply does not override this.
  virtual bool go_sleep() { return this->go_standby(); }

  /// One-shot wiring self-test at setup. Default: nothing to test, pass.
  ///
  /// Not in the sibling component's interface. On the CMT2300A this is the
  /// GPIO3/INT2 line check: it temporarily remuxes INT2 to RX_ACTIVE and confirms
  /// the pin tracks the chip's own state (low in standby, high in RX), then puts
  /// INT2 back. That line is the ONLY receive-side signal this board has, so a
  /// mis-wire there is fatal and - without this test - presents as "the meter never
  /// answers", which sends the user looking at antennas and meters instead of at a
  /// jumper. A backend with nothing to test returns true.
  ///
  /// Advisory: the component logs the result and carries on either way, so a
  /// backend may use it for a soft check as well as a hard one.
  virtual bool self_test() { return true; }

  /// Backend-specific lines for the component's dump_config(), already indented
  /// to sit under its own header. ESPHome dumps a component's configuration even
  /// after mark_failed(), so this may well be called when init() returned false and
  /// the chip is unresponsive or absent: print stored configuration only, and do not
  /// talk to the chip from here.
  virtual void dump_config() = 0;

  /// Reverse the bits of a byte - the logical <-> on-air mapping for any backend
  /// that has to flip bit order in software.
  ///
  /// Nothing in this component calls it today: the CMT2300A's packet engine is
  /// configured to put RF433-1 on the air in the same bit order the frame builder
  /// uses, so no software flip is needed. It is kept here, identical to the sibling
  /// component's, because it belongs to the shared shape of this interface and
  /// because a backend without that hardware option will need exactly this.
  static uint8_t reverse8(uint8_t b) {
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
  }
};

}  // namespace esphome::nartis_rf_meter
