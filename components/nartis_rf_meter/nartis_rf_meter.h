/*
 * Nartis RF Meter — ESPHome Component
 *
 * Top-level orchestrator that ties together:
 *   Layer 1: CMT2300A HAL (SPI, registers, state machine)
 *   Layer 2: RF Data (framing, CRC-16/DNP, AES-128-CCM, channels)
 *   Layer 3: DLMS Client (proprietary read request/response)
 *
 * Implements a non-blocking state machine in loop() that performs:
 *   RSSI scan → Channel select → Beacon → GET (×N) + ACK → Publish
 * No separate AARQ/AARE — firmware uses proprietary per-request format.
 */

#pragma once

#include "esphome/core/component.h"
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

  /// Enable passive sniff mode: the component never transmits. It parks the
  /// radio in RX and dumps every received frame as raw hex. No address filter,
  /// no CRC/AES — pure on-air bytes.
  void set_sniff_mode(bool enable) { sniff_mode_ = enable; }
  /// Frequency bank (0..3) to camp on while sniffing. Per the CIU firmware the
  /// live link runs on channel 0 (RX freq 434.1026 MHz), so 0 is the default.
  void set_sniff_channel(uint8_t ch) { sniff_channel_ = ch & 0x3; }

  /// Pin the active-mode physical channel (0..3), bypassing the RSSI auto-scan.
  /// The firmware transmits/listens on a fixed bank (channel 0) and only encodes
  /// the scanned index as a frame hint, so forcing the channel here is the lever
  /// to match the meter when auto-select tunes to the wrong frequency.
  void set_fix_channel(uint8_t ch) { fix_channel_ = static_cast<int8_t>(ch & 0x3); }

  /// TX test mode: continuously transmit the probe frame back-to-back (no RX,
  /// no state machine) until the device is reflashed. Use with an SDR to confirm
  /// the radio is actually emitting on the expected frequency (Ch0 ≈ 431.23 MHz).
  void set_tx_test_mode(bool enable) { tx_test_mode_ = enable; }

  /// Delay (ms) from setup completion to the single pairing attempt. A 3-2-1
  /// countdown is logged before it fires so you can start an air capture in time.
  void set_pairing_delay_ms(uint32_t ms) { pairing_delay_ms_ = ms; }

  /* ---- Sensor registration ---- */
  void register_sensor(esphome::sensor::Sensor *s, const ObisCode &obis,
                       uint16_t class_id, uint8_t attr_id);
  void register_text_sensor(esphome::text_sensor::TextSensor *s, const ObisCode &obis,
                            uint16_t class_id, uint8_t attr_id);

  /* ---- State machine (public for logging helper) ---- */
  enum class State : uint8_t {
    NOT_INITIALIZED,
    IDLE,
    // Passive sniff — never transmits; dumps every received frame as raw hex
    SNIFF_LISTEN,
    // TX test — transmits the probe frame back-to-back forever (RF presence check)
    TX_TEST,
    // Terminal stop — pairing failed; radio asleep, no further TX (reboot to retry)
    STOPPED,
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
    // Beacon (wake up meter)
    BEACON_TX,
    BEACON_WAIT_TX_DONE,
    BEACON_WAIT_RESPONSE,
    // Data requests (proprietary format — no separate AARQ/AARE)
    GET_TX,
    GET_WAIT_TX_DONE,
    GET_WAIT_RESPONSE,
    // ACK after GET response
    ACK_TX,
    ACK_WAIT_TX_DONE,
    // Publish results
    PUBLISH,
    // Error recovery
    ERROR_RECOVERY,
  };

 protected:

  void setup_continue_();
  void set_state_(State new_state);
  void handle_state_();

  /// Kick off a single pairing/read cycle (called once ~1s after setup).
  void start_cycle_();
  /// Enter the terminal STOPPED state: sleep the radio, cease all TX. Reboot to retry.
  void stop_(const char *reason);

  // State handlers
  void handle_rssi_scan_();
  void handle_channel_select_();
  // Pairing handshake handlers
  void handle_pair_probe_tx_();
  void handle_pair_ack_tx_();
  void handle_pair_mode6_tx_();
  size_t build_pair_probe_payload_(uint8_t *out, size_t max);
  void capture_meter_address_();
  void handle_beacon_tx_();
  void handle_wait_tx_done_(State next_state);
  void handle_get_tx_();
  void handle_get_response_();
  void handle_ack_tx_();
  void advance_after_get_();
  void handle_publish_();
  void handle_error_recovery_();

  // TX/RX helpers
  bool transmit_frame_(RfFrameType type, const uint8_t *payload, size_t len);
  int receive_frame_(uint8_t *payload_out, size_t max, RfFrameType *type_out);

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
  bool sniff_mode_{false};  // passive listen-only mode (no TX, raw hex dump)
  uint8_t sniff_channel_{0};// frequency bank to camp on while sniffing
  int8_t fix_channel_{-1};  // active-mode pinned channel (0..3); -1 = RSSI auto-scan
  bool tx_test_mode_{false};// continuous probe TX for SDR/RF verification
  uint32_t pairing_delay_ms_{15000};  // delay from setup to the one-shot pairing attempt
  RfAddress address_{};
  uint8_t aes_key_[AES_KEY_SIZE]{};

  /* Hardcoded protocol constants — same for all Nartis devices */
  static constexpr const char *PASSWORD_ = "123456";  // local pairing PIN, not sent over RF
  static constexpr uint16_t CLIENT_ADDRESS_ = 16;
  static constexpr uint16_t SERVER_ADDRESS_ = 1;
  static constexpr uint32_t RX_TIMEOUT_MS_ = 3000;
  static constexpr uint8_t MAX_RETRIES_ = 3;
  // Pairing-step pacing — the real CIU waits ~5–6 s between pairing TX steps,
  // both for retries and after receiving the meter's response. Mirroring this
  // avoids stomping the meter while it processes the previous frame.
  static constexpr uint32_t PAIR_STEP_DELAY_MS_ = 5500;

  /* ---- Runtime state ---- */
  State state_{State::NOT_INITIALIZED};
  uint32_t state_entered_ms_{0};
  uint8_t retry_count_{0};
  uint8_t current_sensor_idx_{0};
  uint8_t sequence_nr_{0};
  int8_t rssi_readings_[4]{};
  uint8_t rssi_scan_ch_{0};

  /* ---- Pairing state ---- */
  bool paired_{false};       // set once the handshake completes this boot
  uint8_t pair_retry_{0};
  static constexpr uint8_t MAX_PAIR_RETRIES_ = 4;

  /* ---- Buffers ---- */
  std::array<uint8_t, MAX_RF_FRAME_SIZE> tx_buf_{};
  std::array<uint8_t, MAX_RF_FRAME_SIZE> rx_buf_{};

  /* ---- Chunked RX accumulation ---- */
  uint8_t rx_accum_buf_[MAX_RF_FRAME_SIZE]{};
  size_t rx_accum_len_{0};
  size_t rx_expected_len_{0};
  bool rx_active_{false};
  uint32_t rx_tail_wait_ms_{0};  // 0 = not waiting; else millis() when tail-wait started
  static constexpr uint32_t RX_TAIL_WAIT_MS_ = 30;  // let sub-threshold tail bytes arrive

  /* ---- Sensors ---- */
  std::vector<SensorEntry> sensors_;
};

}  // namespace esphome::nartis_rf_meter
