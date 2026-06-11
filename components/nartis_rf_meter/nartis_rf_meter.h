/*
 * Nartis RF Meter — ESPHome Component
 *
 * Top-level orchestrator that ties together:
 *   Layer 1: CMT2300A HAL (SPI, registers, state machine)
 *   Layer 2: RF Data (framing, CRC-16/DNP, AES-128-GCM, channels)
 *   Layer 3: DLMS Client (proprietary read request/response)
 *
 * Implements a non-blocking state machine in loop() that performs:
 *   RSSI scan → Channel select → Beacon → GET (×N) + ACK → Publish
 * No separate AARQ/AARE — firmware uses proprietary per-request format.
 */

#pragma once

#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

#include "cmt2300a_hal.h"
#include "rf_data.h"
#include "dlms_client.h"
#include "helpers.h"

#include <array>
#include <vector>
#include <string>

namespace esphome::nartis_rf_meter {

/// Pairing state persisted to NVS so a reboot can resume an established
/// session without re-running the handshake. Plain-old-data, byte-packed so
/// the layout is stable across builds (bump kNvsStateVersion if it changes).
struct NvsPairingState {
  uint32_t version;                 // kNvsStateVersion; mismatch ⇒ ignore blob
  uint8_t  aes_key[AES_KEY_SIZE];   // meter-assigned data key (ASCII(SN)[12] + suffix[4])
  uint8_t  meter_addr[8];           // learned meter RF address (RX filter)
  uint32_t frame_counter;           // last TX GCM counter used (rf_)
  uint32_t last_rx_counter;         // RX replay counter
  uint32_t last_nested_rx_counter;  // nested-RX replay counter
  uint8_t  data_seq;                // mode-1/2 TX sequence
  uint8_t  beacon_seq;              // mode-3/6 TX sequence
  uint8_t  paired;                  // 1 ⇒ a valid session was saved
  uint8_t  _pad;                    // keep the struct an even size
} __attribute__((packed));

/// Registered sensor entry
struct SensorEntry {
  esphome::sensor::Sensor *sensor{nullptr};
  esphome::text_sensor::TextSensor *text_sensor{nullptr};
  ObisCode obis{};
  uint16_t class_id{3};   // Default: Register
  uint8_t attr_id{2};     // Default: value
  DlmsValue last_value{};
};

class NartisRfMeterComponent : public esphome::PollingComponent {
 public:
  NartisRfMeterComponent() = default;

  /* ---- ESPHome lifecycle ---- */
  void setup() override;
  void dump_config() override;
  void update() override;
  void loop() override;
  float get_setup_priority() const override { return esphome::setup_priority::DATA; }

  /* ---- Configuration setters (called by Python codegen) ---- */
  void set_pin_sdio(esphome::InternalGPIOPin *p) { pin_sdio_ = p; }
  void set_pin_sclk(esphome::InternalGPIOPin *p) { pin_sclk_ = p; }
  void set_pin_csb(esphome::InternalGPIOPin *p) { pin_csb_ = p; }
  void set_pin_fcsb(esphome::InternalGPIOPin *p) { pin_fcsb_ = p; }
  void set_pin_gpio3(esphome::InternalGPIOPin *p) { pin_gpio3_ = p; }

  /// Set the 12-digit meter serial (printed on the meter nameplate).
  void set_meter_serial(const std::string &s);
  void set_ciu_serial(const std::string &s) { ciu_serial_ = s; }
  /// Set the full 8-byte CIU RF address as 16 hex chars (e.g. "CD2C0000026B5025").
  /// Overrides serial/MAC derivation — use to impersonate a paired CIU exactly.
  void set_ciu_address(const std::string &hex) { ciu_address_ = hex; }

  /// Pin the active-mode physical channel (0..3), bypassing the RSSI auto-scan.
  /// The firmware transmits/listens on a fixed bank (channel 0) and only encodes
  /// the scanned index as a frame hint, so forcing the channel here is the lever
  /// to match the meter when auto-select tunes to the wrong frequency.
  void set_fix_channel(uint8_t ch) { fix_channel_ = static_cast<int8_t>(ch & 0x3); }

  /// Use the non-standard ("invented") frequency presets (NARTIS_CUSTOM_CHANNELS)
  /// instead of the firmware channel grid. The custom centres sit on the meter's
  /// actually-observed reply frequencies so a 100 kHz + AFC RX can capture them;
  /// the firmware presets are 97..278 kHz off and miss. Default: firmware presets.
  /// Enabling this also enables RX channel-hop acquisition (see channel_hopping_enabled_).
  void set_use_non_standard_channels(bool enable) {
    use_non_standard_channels_ = enable;
    hal_.set_use_custom_channels(enable);
  }

  /// User-defined OBIS attributes read per get-request-with-list (1..10). Larger
  /// batches mean fewer cycles but bigger 0x43 responses. Default 1.
  void set_batch_size(uint8_t n) { user_batch_size_ = (n < 1) ? 1 : n; }

  /// EXPERIMENT: skip the closing ("FIN") beacon after each GET response and go
  /// straight to the next batch. Saves ~4.5 s per batch. Default off.
  void set_skip_fin_beacons(bool enable) { skip_fin_beacons_ = enable; }

  /// General RX wait before declaring "no response" (ms). Governs pairing waits
  /// and frame completion once a reply has started. Default 3000.
  void set_rx_timeout_ms(uint32_t ms) { rx_timeout_ms_ = ms; }

  /// Short RX wait for the steady-state GET reply states, applied only until the
  /// first byte arrives — a request dropped in the meter's post-TX deaf window
  /// re-sends after this instead of stalling the full rx_timeout. Default 900.
  void set_rx_reply_timeout_ms(uint32_t ms) { rx_reply_timeout_ms_ = ms; }

  /* ---- Sensor registration ---- */
  void register_sensor(esphome::sensor::Sensor *s, const ObisCode &obis,
                       uint16_t class_id, uint8_t attr_id);
  void register_text_sensor(esphome::text_sensor::TextSensor *s, const ObisCode &obis,
                            uint16_t class_id, uint8_t attr_id);

  /* ---- State machine (public for logging helper) ---- */
  enum class State : uint8_t {
    NOT_INITIALIZED,
    IDLE,
    // Channel scan
    RSSI_SCAN,
    CHANNEL_SELECT,
    // First-time pairing handshake (only when not yet paired this boot)
    //   probe (plain, meter serial) → 0x06 → ACK (enc) → 0x53 → mode-6 (plain) → keepalive
    PAIR_PROBE_TX,
    PAIR_PROBE_WAIT_TX_DONE,
    PAIR_PROBE_WAIT_RESPONSE,
    PAIR_ACK_TX,
    PAIR_ACK_WAIT_TX_DONE,
    PAIR_ACK_WAIT_RESPONSE,
    PAIR_MODE6_TX,
    PAIR_MODE6_WAIT_TX_DONE,
    PAIR_WAIT_KEEPALIVE,
    // Initial post-pairing beacon (wake up meter)
    BEACON_TX,
    BEACON_WAIT_TX_DONE,
    BEACON_WAIT_RESPONSE,
    // Data-read cycle (6 frames: GET → keepalive → BEACON → data → BEACON → ack).
    // Mirrors the firmware cycle in dump-spi2/decoded_sequence.txt frames 0..5.
    GET_TX,                       // TX 0x44 encrypted get-request (IEC+DLMS payload)
    GET_WAIT_TX_DONE,
    GET_WAIT_KEEPALIVE,           // expect RX 0x5B = "meter received request, hang on"
    GET_REQ_BEACON_TX,            // TX 0x08 BEACON to fetch the data
    GET_REQ_BEACON_WAIT_TX_DONE,
    GET_WAIT_DATA,                // expect RX 0x43 with the actual data response
    GET_FIN_BEACON_TX,            // TX 0x08 BEACON to acknowledge data
    GET_FIN_BEACON_WAIT_TX_DONE,
    GET_WAIT_FINAL_ACK,           // expect RX 0x40 short ack — cycle complete
    // Publish results
    PUBLISH,
    // Error recovery
    ERROR_RECOVERY,
  };

 protected:

  void setup_continue_();
  void set_state_(State new_state);
  void handle_state_();

  /// Kick off one pairing/read cycle. Called from update() whenever IDLE.
  /// Always starts at RSSI scan → channel select; the pair-vs-read branch then
  /// happens in handle_channel_select_() based on paired_.
  void start_cycle_();
  /// Soft-recover from a failed cycle: stop RX, sleep the radio, reset DLMS,
  /// account the failure, and return to IDLE so the next update() retries.
  void abort_to_idle_(const char *reason);
  /// Account one cycle outcome toward the re-pair counter. A read-phase failure
  /// (while paired_) increments the counter; MAX_READ_FAILURES_BEFORE_REPAIR_
  /// consecutive failures drop the pairing via reset_pairing_state_().
  void note_cycle_failure_();
  /// Tear down the current pairing so the next cycle re-pairs from scratch:
  /// clears paired_/session_primed_, restores the bootstrap AES key, and resets
  /// the RF frame counters and TX sequence counters to fresh-pairing values.
  /// Also clears the persisted NVS blob so a reboot doesn't restore it.
  void reset_pairing_state_();

  /// Restore a previously-saved session (key, counters, meter address, paired_)
  /// from NVS. The TX frame counter is skipped ahead by FRAME_COUNTER_MARGIN_ to
  /// cover increments used but not yet saved before a crash. session_primed_ is
  /// intentionally left false so the next cycle re-sends the priming get-request.
  void load_pairing_state_();
  /// Persist the current session to NVS. Called whenever the session advances
  /// (priming done, cycle published) or is torn down.
  void save_pairing_state_();

  // State handlers
  void handle_rssi_scan_();
  void handle_channel_select_();

  /* ---- RX channel-hop acquisition ----
   * The meter's reply carrier is stable WITHIN a session but wanders up to
   * ~650 kHz BETWEEN sessions, landing 97..278 kHz off the firmware RX presets.
   * When enabled, we walk the (custom) RX channels on each unanswered probe and
   * LOCK onto whichever channel first yields a reply, for the rest of the session.
   *
   * Only active with non-standard channels (custom presets share one TX freq of
   * 433.82 MHz, so hopping moves RX only and never changes the probe frequency)
   * and no explicit fix_channel pin.
   */
  bool channel_hopping_enabled_() const {
    // Hop the RX channel to acquire the meter whenever a channel isn't pinned —
    // independent of which channel table is in use. Hops move RX only (TX stays
    // on Ch0/433.82, the meter's wake frequency), so it's safe for both the
    // firmware presets and the custom presets.
    return fix_channel_ < 0;
  }
  /// Advance RX to the next channel before a retry probe. No-op if hopping is
  /// disabled or the channel is already locked. Returns true if it hopped.
  bool hop_channel_();
  /// Pin the current channel for the rest of the session (meter answered on it).
  void lock_channel_();
  /// RX centre frequency (MHz) of the currently-programmed channel, for logging.
  float active_rx_freq_mhz_() const;
  // Pairing handshake handlers
  void handle_pair_probe_tx_();
  void handle_pair_ack_tx_();
  void handle_pair_mode6_tx_();
  size_t build_pair_probe_payload_(uint8_t *out, size_t max);
  void capture_meter_address_();
  void handle_beacon_tx_();
  void handle_wait_tx_done_(State next_state);
  /// Wrap a DLMS APDU in the IEC 62056-47 envelope and transmit it as an
  /// encrypted mode-2 (0x44) frame. Shared by the priming and with-list GETs.
  bool tx_dlms_apdu_(const uint8_t *apdu, size_t apdu_len);
  void handle_get_tx_();
  bool handle_get_response_();   // returns false if the response could not be parsed
  void advance_after_get_();
  void skip_current_batch_();    // force-advance past the current batch (parse give-up)
  void handle_publish_();
  void handle_error_recovery_();

  // TX/RX helpers
  bool transmit_frame_(RfFrameType type, const uint8_t *payload, size_t len);

  // Chunked RX — for packets > 64 bytes
  enum class RxStatus : uint8_t {
    IN_PROGRESS,
    COMPLETE,
    ERROR,
  };

  /// Begin chunked RX: configure INT1=FIFO_TH, prepare SDIO, arm ISR, enter RX.
  void start_rx_();

  /// Poll for incoming data. Drains ISR queue into accumulation buffer.
  /// Returns COMPLETE when expected length reached.
  RxStatus poll_rx_();

  /// Stop RX: disable ISR, go standby, clear state.
  void finish_rx_();

  /* ---- Three layers ---- */
  Cmt2300aHal hal_;
  RfDataLayer rf_;
  DlmsClient dlms_;

  /// Derive RF address from CIU serial (or ESP32 MAC if not set).
  /// Called during setup().
  void derive_rf_address_();

  /// Build 29-byte beacon payload matching firmware beacon_address_builder (0xB5B0).
  size_t build_beacon_payload_(uint8_t *out, size_t max);

  /// Pack current time into 6-byte bit-packed RTC timestamp.
  void build_rtc_timestamp_(uint8_t *out);

  /* ---- Configuration ---- */
  esphome::InternalGPIOPin *pin_sdio_{nullptr};
  esphome::InternalGPIOPin *pin_sclk_{nullptr};
  esphome::InternalGPIOPin *pin_csb_{nullptr};
  esphome::InternalGPIOPin *pin_fcsb_{nullptr};
  esphome::InternalGPIOPin *pin_gpio3_{nullptr};
  std::string meter_serial_;
  std::string ciu_serial_;  // empty = use ESP32 MAC address
  std::string ciu_address_; // optional 16-hex-char full CIU address override
  int8_t fix_channel_{-1};  // active-mode pinned channel (0..3); -1 = RSSI auto-scan
  RfAddress address_{};
  uint8_t aes_key_[AES_KEY_SIZE]{};

  /* Hardcoded protocol constants — same for all Nartis devices */
  static constexpr const char *PASSWORD_ = "123456";  // local pairing PIN, not sent over RF
  static constexpr uint16_t CLIENT_ADDRESS_ = 16;
  static constexpr uint16_t SERVER_ADDRESS_ = 1;
  static constexpr uint32_t RX_TIMEOUT_MS_DEFAULT_ = 3000;
  static constexpr uint8_t MAX_RETRIES_ = 3;
  // Steady-state GET reply wait. When the meter is listening it answers a
  // get-request in ~190 ms; a long wait only burns time when our request lands
  // in the meter's post-TX deaf window (it ignores frames that arrive too soon
  // after it just transmitted). So in the GET reply states we time out fast and
  // re-send, re-probing until the meter reopens its receiver — instead of
  // sitting through one 3 s stall. The short timeout applies only until the
  // first byte arrives; once a reply starts, RX_TIMEOUT_MS_ governs completion
  // so a large frame is never aborted mid-flight. (Pairing keeps the long
  // timeout + PAIR_RETRY_DELAY_MS_ pacing — it's one-time and works.)
  static constexpr uint32_t RX_REPLY_TIMEOUT_MS_DEFAULT_ = 900;
  // ~900 ms RX + ~250 ms TX ≈ 1.15 s per probe; 6 spans a ~7 s deaf window.
  static constexpr uint8_t GET_REPLY_MAX_RETRIES_ = 6;
  // Pairing-step pacing — matches the real CIU's f02 capture timings:
  //   * Retry after no response (probe→probe = 5755 ms, ACK→ACK = 5104 ms):
  //     combine TX (~200 ms) + RX_TIMEOUT_MS_ (3000 ms) + gate = ~5500 ms total.
  //   * First TX after RX 0x06: real CIU fires at +637 ms — fast.
  //   * First TX after RX 0x53 (mode-6 confirm): real CIU waits 5629 ms.
  //     Meter is doing key-install internally; faster than this gets dropped.
  static constexpr uint32_t PAIR_RETRY_DELAY_MS_         = 2300;  // gate during retry-after-timeout → ~5.5 s total
  static constexpr uint32_t PAIR_POST_PROBE_DELAY_MS_    = 500;   // small settle after RX 0x06, before ACK TX
  static constexpr uint32_t PAIR_POST_SESSION_DELAY_MS_  = 5500;  // long gate after RX 0x53, before mode-6 TX
  // GET-cycle pacing — measured from iq3/f01 (real-CIU normal session):
  //
  //   RX 0x5B keepalive  →  TX data-request BEACON   ≈ 4800 ms
  //   RX 0x43 response   →  TX closing BEACON        ≈ 4200 ms
  //
  // The 4-5 second window after RX 0x5B is the meter querying its OBIS
  // registers internally. Sending the BEACON earlier than that makes the
  // meter respond with 0x40 (cycle-close ack, no data) instead of 0x43
  // (data response). Use ~5000 ms to comfortably cover the firmware's gap.
  static constexpr uint32_t GET_REQ_BEACON_DELAY_MS_     = 5000;
  static constexpr uint32_t GET_FIN_BEACON_DELAY_MS_     = 4500;

  // FAST PRIMING (experiment). The priming read is a throwaway engagement GET —
  // its 0x43 data is discarded, so it does NOT need the ~5 s pacing that exists
  // to let the meter prepare *real* data. We only need enough req-beacon polls
  // to engage the meter so the first real batch is answered directly. Using a
  // short delay + few retries cuts priming from ~15 s (3×5 s) to a few seconds.
  // These apply ONLY while !session_primed_; real batches keep the full pacing.
  // Tune up if the first real batch starts coming back sacrificial (0x40 no-data).
  static constexpr uint32_t PRIMING_REQ_BEACON_DELAY_MS_ = 1000;
  static constexpr uint8_t  PRIMING_MAX_REQ_RETRIES_     = 2;

  /* ---- Runtime state ---- */
  State state_{State::NOT_INITIALIZED};
  uint32_t state_entered_ms_{0};
  uint32_t session_start_ms_{0};  // millis() at update()/start_cycle_(); used to report session length at PUBLISH
  uint8_t retry_count_{0};
  uint8_t resp_fail_retries_{0};  // consecutive unparseable 0x43 responses for the current batch
  // Runtime-configurable (YAML); seeded from the *_DEFAULT_ constants above.
  uint8_t user_batch_size_{USER_BATCH_SIZE_DEFAULT_};
  uint32_t rx_timeout_ms_{RX_TIMEOUT_MS_DEFAULT_};
  uint32_t rx_reply_timeout_ms_{RX_REPLY_TIMEOUT_MS_DEFAULT_};
  bool skip_fin_beacons_{false};   // YAML skip_fin_beacons — no closing beacon between batches
  uint8_t current_sensor_idx_{0};
  // The firmware maintains TWO independent TX sequence counters (per
  // FRAME_HEADER_SPEC §4 + verified in iq3/f02 pairing capture):
  //
  //   data_seq_   — used by mode-1 (DATA/probe, 0x46) and mode-2 (ACK, 0x44).
  //                 Real CIU starts at 2 in f02 burst 1 → 02, 03, 04, 05 ...
  //
  //   beacon_seq_ — used by mode-3 (BEACON, 0x08) and mode-6 (PLAIN_DATA, 0x00).
  //                 Real CIU sends mode-6 with seq=01 in f02 burst 7. Independent
  //                 from data_seq_. Starts at 1.
  //
  // Conflating these into a single counter caused the meter to reject our MODE-6
  // (we sent seq=06 where firmware sends seq=01) and stay in a SESSION_SETUP
  // retransmit loop — see pair-ours-11 bursts 6/8/10 (same SESSION_SETUP frame
  // sent 3 times by the meter, because our MODE-6 and BEACON were both seq-wrong).
  uint8_t data_seq_{2};
  uint8_t beacon_seq_{1};
  int8_t rssi_readings_[4]{};
  uint8_t rssi_scan_ch_{0};

  /* ---- Pairing state ---- */
  bool paired_{false};       // set once the handshake completes this boot
  bool session_primed_{false};  // set once the post-pairing normal-get priming completes
  uint8_t pair_retry_{0};
  static constexpr uint8_t MAX_PAIR_RETRIES_ = 5;
  // Consecutive failed read cycles while paired_. When this hits
  // MAX_READ_FAILURES_BEFORE_REPAIR_ we drop the pairing and re-pair from
  // scratch on the next update() (the meter may have dropped our session).
  // Reset to 0 by a cycle that reaches PUBLISH.
  uint8_t consecutive_read_failures_{0};
  static constexpr uint8_t MAX_READ_FAILURES_BEFORE_REPAIR_ = 10;

  /* ---- NVS session persistence ---- */
  esphome::ESPPreferenceObject pref_;
  // Bump when NvsPairingState layout/semantics change — a mismatch makes
  // load_pairing_state_() ignore the stored blob and re-pair.
  static constexpr uint32_t NVS_STATE_VERSION_ = 1;
  // On restore, advance the TX frame counter past any values that may have been
  // used in an in-flight cycle but not persisted before a crash, so the meter
  // never sees a replayed counter. Must exceed the encrypted-TX count of a
  // single cycle (counters are saved at end-of-cycle and on abort).
  static constexpr uint32_t FRAME_COUNTER_MARGIN_ = 256;

  /* ---- RX channel-hop acquisition state ---- */
  bool use_non_standard_channels_{false};  // mirrors HAL flag; gates channel hopping
  uint8_t active_channel_{2};   // advertised+RX channel index; default ch2 = 434.26 MHz reply
  bool channel_locked_{false};  // true once the meter has replied on active_channel_
  uint8_t channel_hop_count_{0};
  // Probe-attempt budget when hopping: enough for a full sweep of the 4 custom
  // channels plus margin for meter wake-up (~2 passes of the 2 likeliest).
  static constexpr uint8_t MAX_CHANNEL_HOPS_ = 6;

  /* ---- GET-batching state ----
   * After pairing the meter requires one priming get-request-normal (single
   * OBIS) before it engages the with-list flow; after that we read the user-
   * configured sensors in get-request-with-list batches (up to 10 per frame).
   */
  uint8_t batch_start_idx_{0};                 // index into sensors_ of current batch's first attr
  uint8_t batch_count_{0};                     // attrs in the currently-pending batch
  static constexpr uint8_t USER_BATCH_SIZE_DEFAULT_ = 1;

  /* ---- Noisy-RX retry-within-window state ----
   * On a parse failure where the frame's type byte isn't what we expect
   * (real meter responses are 0x06/0x40/0x43/0x53/0x5B; anything else is
   * noise), the WAIT_RESPONSE handlers re-arm the radio and listen for
   * another frame, up to MAX_RX_PARSE_RETRIES_ times, while still within
   * the RX_TIMEOUT_MS_ window of the current state.
   */
  uint8_t rx_parse_retries_{0};
  static constexpr uint8_t MAX_RX_PARSE_RETRIES_ = 3;

  /* ---- Buffers ---- */
  std::array<uint8_t, MAX_RF_FRAME_SIZE> tx_buf_{};
  std::array<uint8_t, MAX_RF_FRAME_SIZE> rx_buf_{};

  /* ---- Chunked RX accumulation ---- */
  uint8_t rx_accum_buf_[MAX_RF_FRAME_SIZE]{};
  size_t rx_accum_len_{0};
  size_t rx_expected_len_{0};
  bool rx_active_{false};
  uint32_t rx_tail_wait_ms_{0};  // 0 = not waiting; else millis() when tail-wait started
  uint32_t rx_drain_stall_ms_{0};  // millis() of last chunk-drain progress; 0 = none yet
  // Tail-byte wait: at 2400 bps each byte takes ~3.3 ms, so up to
  // (FIFO_TH_VALUE - 1) = 14 trailing bytes take ~47 ms to arrive. Must wait
  // long enough that all of them are in the FIFO before we read by length.
  // Pair-ours-11 burst 6 proved this: SDR captured CRC `44 A8` on air, but
  // our chip's last two bytes read as `00 00` with the old 30 ms wait —
  // i.e. we read the FIFO before the last 2 bytes had arrived. 70 ms
  // covers the 14-byte worst case with margin.
  static constexpr uint32_t RX_TAIL_WAIT_MS_ = 70;
  // Drain-stall recovery: once the packet is fully received the chip stops
  // asserting RX_FIFO_TH even with a full chunk (>= FIFO_TH_VALUE) still unread
  // in the FIFO — the chunk-drain then halts at a multiple of FIFO_TH_VALUE
  // (observed 45/66, 75/93). If the drain has found nothing for this long while
  // bytes are still owed, reception is over and the remainder is sitting in the
  // 64 B FIFO; read it directly by length. Must exceed the in-flight inter-chunk
  // gap (~60 ms at this bitrate) so it never misfires mid-stream.
  static constexpr uint32_t RX_DRAIN_STALL_MS_ = 100;

  /* ---- Sensors ---- */
  std::vector<SensorEntry> sensors_;
};

}  // namespace esphome::nartis_rf_meter
