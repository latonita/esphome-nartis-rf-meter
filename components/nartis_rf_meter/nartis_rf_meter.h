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
  void set_pin_gpio1(esphome::InternalGPIOPin *p) { pin_gpio1_ = p; }

  void set_meter_serial(const std::string &s) { meter_serial_ = s; }
  void set_ciu_serial(const std::string &s) { ciu_serial_ = s; }
  void set_aes_key(const std::string &key);

  /* ---- Sensor registration ---- */
  void register_sensor(esphome::sensor::Sensor *s, const ObisCode &obis,
                       uint16_t class_id, uint8_t attr_id);
  void register_text_sensor(esphome::text_sensor::TextSensor *s, const ObisCode &obis,
                            uint16_t class_id, uint8_t attr_id);

 protected:
  /* ---- State machine ---- */
  enum class State : uint8_t {
    NOT_INITIALIZED,
    IDLE,
    // Channel scan
    RSSI_SCAN,
    CHANNEL_SELECT,
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

  void set_state_(State new_state);
  void handle_state_();

  // State handlers
  void handle_rssi_scan_();
  void handle_channel_select_();
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
  esphome::InternalGPIOPin *pin_gpio1_{nullptr};
  std::string meter_serial_;
  std::string ciu_serial_;  // empty = use ESP32 MAC address
  RfAddress address_{};
  uint8_t aes_key_[AES_KEY_SIZE]{};

  /* Hardcoded protocol constants — same for all Nartis devices */
  static constexpr const char *PASSWORD_ = "123456";  // local pairing PIN, not sent over RF
  static constexpr uint16_t CLIENT_ADDRESS_ = 16;
  static constexpr uint16_t SERVER_ADDRESS_ = 1;
  static constexpr uint32_t RX_TIMEOUT_MS_ = 2000;
  static constexpr uint8_t MAX_RETRIES_ = 3;

  /* ---- Runtime state ---- */
  State state_{State::NOT_INITIALIZED};
  uint32_t state_entered_ms_{0};
  uint8_t retry_count_{0};
  uint8_t current_sensor_idx_{0};
  uint8_t sequence_nr_{0};
  int8_t rssi_readings_[4]{};
  uint8_t rssi_scan_ch_{0};

  /* ---- Buffers ---- */
  std::array<uint8_t, MAX_RF_FRAME_SIZE> tx_buf_{};
  std::array<uint8_t, MAX_RF_FRAME_SIZE> rx_buf_{};

  /* ---- Chunked RX accumulation ---- */
  uint8_t rx_accum_buf_[MAX_RF_FRAME_SIZE]{};
  size_t rx_accum_len_{0};
  size_t rx_expected_len_{0};
  bool rx_active_{false};

  /* ---- Sensors ---- */
  std::vector<SensorEntry> sensors_;
};

}  // namespace esphome::nartis_rf_meter
