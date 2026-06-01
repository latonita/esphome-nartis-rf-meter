/*
 * Nartis RF Meter — ESPHome Component Implementation
 *
 * Non-blocking state machine orchestrating CMT2300A HAL, RF framing, and DLMS client.
 */

#include "nartis_rf_meter.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/time.h"

#include <cstring>
#include <ctime>

namespace esphome::nartis_rf_meter {

static const char *const TAG = "nartis_rf_meter";

/* ================================================================
 * ESPHome Lifecycle
 * ================================================================ */

void NartisRfMeterComponent::setup() {
  ESP_LOGI(TAG, "Nartis RF Meter: deferring init 10s for network log viewers...");
  this->set_timeout(10000, [this]() { this->setup_continue_(); });
}

void NartisRfMeterComponent::setup_continue_() {
  ESP_LOGI(TAG, "Setting up Nartis RF Meter...");

  // Initialize HAL
  hal_.set_pins(pin_sdio_, pin_sclk_, pin_csb_, pin_fcsb_, pin_gpio1_);
  if (!hal_.init()) {
    ESP_LOGE(TAG, "CMT2300A initialization failed!");
    this->mark_failed();
    return;
  }

  // One-time GPIO1/NIRQ wiring self-test (decisive: RX is impossible if the
  // chip's interrupt line isn't reaching the configured pin_gpio1).
  hal_.test_gpio1_wiring();

  // Derive RF address from CIU serial (or ESP32 MAC)
  derive_rf_address_();

  // Configure RF data layer
  rf_.set_address(address_);
  rf_.set_aes_key(aes_key_);

  // Configure DLMS client
  dlms_.set_credentials(PASSWORD_, CLIENT_ADDRESS_, SERVER_ADDRESS_);

  // Start in IDLE
  set_state_(State::IDLE);

  ESP_LOGI(TAG, "Nartis RF Meter ready. %d sensor(s) registered.", (int) sensors_.size());
  ESP_LOGI(TAG, "AES key: %s", format_hex_pretty(aes_key_, AES_KEY_SIZE).c_str());
  uint8_t addr_bytes[8];
  address_.to_bytes(addr_bytes);
  ESP_LOGI(TAG, "RF address (8 bytes): %s", format_hex_pretty(addr_bytes, 8).c_str());
}

void NartisRfMeterComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Nartis RF Meter:");
  ESP_LOGCONFIG(TAG, "  Pins: SDIO=%d, SCLK=%d, CSB=%d, FCSB=%d, GPIO1=%d",
                pin_sdio_->get_pin(), pin_sclk_->get_pin(), pin_csb_->get_pin(),
                pin_fcsb_->get_pin(), pin_gpio1_->get_pin());
  ESP_LOGCONFIG(TAG, "  Address: device_id=0x%04X, serial_hash=0x%08X, group=%d, type=%d",
                address_.device_id, (unsigned) address_.serial_hash,
                address_.group_id, address_.device_type);
  ESP_LOGCONFIG(TAG, "  DLMS: client=%d, server=%d", CLIENT_ADDRESS_, SERVER_ADDRESS_);
  ESP_LOGCONFIG(TAG, "  Sensors: %d registered", (int) sensors_.size());

  for (size_t i = 0; i < sensors_.size(); i++) {
    const auto &entry = sensors_[i];
    const char *name = entry.sensor ? entry.sensor->get_name().c_str()
                                    : (entry.text_sensor ? entry.text_sensor->get_name().c_str() : "?");
    ESP_LOGCONFIG(TAG, "    [%d] %s — OBIS %d.%d.%d.%d.%d.%d class=%d attr=%d",
                  (int) i, name,
                  entry.obis.bytes[0], entry.obis.bytes[1], entry.obis.bytes[2],
                  entry.obis.bytes[3], entry.obis.bytes[4], entry.obis.bytes[5],
                  entry.class_id, entry.attr_id);
  }
}

void NartisRfMeterComponent::update() {
  if (state_ == State::IDLE) {
    if (sensors_.empty()) {
      ESP_LOGW(TAG, "No sensors registered — skipping update");
      return;
    }
    ESP_LOGI(TAG, "Starting meter read cycle...");
    current_sensor_idx_ = 0;
    retry_count_ = 0;
    set_state_(State::RSSI_SCAN);
  } else {
    ESP_LOGD(TAG, "Update skipped — state machine busy (state=%d)", static_cast<int>(state_));
  }
}

void NartisRfMeterComponent::loop() {
  if (state_ == State::NOT_INITIALIZED || state_ == State::IDLE) {
    return;
  }
  handle_state_();
}

/* ================================================================
 * Configuration
 * ================================================================ */

static constexpr uint8_t NARTIS_PAIRING_SALT[4] = {0xBD, 0x02, 0x9B, 0xBE};

void NartisRfMeterComponent::set_meter_serial(const std::string &s) {
  meter_serial_ = s;

  // Derive the AES-128-GCM key:
  //   key[0..11]  = ASCII bytes of the 12-digit meter serial
  //   key[12..15] = Nartis-wide constant BD 02 9B BE
  memset(aes_key_, 0, AES_KEY_SIZE);
  size_t copy_len = s.size();
  if (copy_len > 12) copy_len = 12;
  memcpy(aes_key_, s.c_str(), copy_len);
  memcpy(aes_key_ + 12, NARTIS_PAIRING_SALT, 4);
}

void NartisRfMeterComponent::register_sensor(esphome::sensor::Sensor *s,
                                             const ObisCode &obis,
                                             uint16_t class_id, uint8_t attr_id) {
  SensorEntry entry;
  entry.sensor = s;
  entry.obis = obis;
  entry.class_id = class_id;
  entry.attr_id = attr_id;
  sensors_.push_back(entry);
}

void NartisRfMeterComponent::register_text_sensor(esphome::text_sensor::TextSensor *s,
                                                  const ObisCode &obis,
                                                  uint16_t class_id, uint8_t attr_id) {
  SensorEntry entry;
  entry.text_sensor = s;
  entry.obis = obis;
  entry.class_id = class_id;
  entry.attr_id = attr_id;
  sensors_.push_back(entry);
}

/* ================================================================
 * State Machine
 * ================================================================ */

static const LogString *state_to_str(NartisRfMeterComponent::State s) {
  switch (s) {
    case NartisRfMeterComponent::State::NOT_INITIALIZED: return LOG_STR("NOT_INITIALIZED");
    case NartisRfMeterComponent::State::IDLE: return LOG_STR("IDLE");
    case NartisRfMeterComponent::State::RSSI_SCAN: return LOG_STR("RSSI_SCAN");
    case NartisRfMeterComponent::State::CHANNEL_SELECT: return LOG_STR("CHANNEL_SELECT");
    case NartisRfMeterComponent::State::PAIR_PROBE_TX: return LOG_STR("PAIR_PROBE_TX");
    case NartisRfMeterComponent::State::PAIR_PROBE_WAIT_TX_DONE: return LOG_STR("PAIR_PROBE_WAIT_TX_DONE");
    case NartisRfMeterComponent::State::PAIR_PROBE_WAIT_RESPONSE: return LOG_STR("PAIR_PROBE_WAIT_RESPONSE");
    case NartisRfMeterComponent::State::PAIR_ACK_TX: return LOG_STR("PAIR_ACK_TX");
    case NartisRfMeterComponent::State::PAIR_ACK_WAIT_TX_DONE: return LOG_STR("PAIR_ACK_WAIT_TX_DONE");
    case NartisRfMeterComponent::State::PAIR_ACK_WAIT_RESPONSE: return LOG_STR("PAIR_ACK_WAIT_RESPONSE");
    case NartisRfMeterComponent::State::PAIR_MODE6_TX: return LOG_STR("PAIR_MODE6_TX");
    case NartisRfMeterComponent::State::PAIR_MODE6_WAIT_TX_DONE: return LOG_STR("PAIR_MODE6_WAIT_TX_DONE");
    case NartisRfMeterComponent::State::PAIR_WAIT_KEEPALIVE: return LOG_STR("PAIR_WAIT_KEEPALIVE");
    case NartisRfMeterComponent::State::BEACON_TX: return LOG_STR("BEACON_TX");
    case NartisRfMeterComponent::State::BEACON_WAIT_TX_DONE: return LOG_STR("BEACON_WAIT_TX_DONE");
    case NartisRfMeterComponent::State::BEACON_WAIT_RESPONSE: return LOG_STR("BEACON_WAIT_RESPONSE");
    case NartisRfMeterComponent::State::GET_TX: return LOG_STR("GET_TX");
    case NartisRfMeterComponent::State::GET_WAIT_TX_DONE: return LOG_STR("GET_WAIT_TX_DONE");
    case NartisRfMeterComponent::State::GET_WAIT_RESPONSE: return LOG_STR("GET_WAIT_RESPONSE");
    case NartisRfMeterComponent::State::ACK_TX: return LOG_STR("ACK_TX");
    case NartisRfMeterComponent::State::ACK_WAIT_TX_DONE: return LOG_STR("ACK_WAIT_TX_DONE");
    case NartisRfMeterComponent::State::PUBLISH: return LOG_STR("PUBLISH");
    case NartisRfMeterComponent::State::ERROR_RECOVERY: return LOG_STR("ERROR_RECOVERY");
    default: return LOG_STR("UNKNOWN");
  }
}

void NartisRfMeterComponent::set_state_(State new_state) {
  ESP_LOGD(TAG, "State: %s -> %s", LOG_STR_ARG(state_to_str(state_)), LOG_STR_ARG(state_to_str(new_state)));
  state_ = new_state;
  state_entered_ms_ = esphome::millis();
}

void NartisRfMeterComponent::handle_state_() {
  uint32_t elapsed = esphome::millis() - state_entered_ms_;

  switch (state_) {
    case State::RSSI_SCAN:
      handle_rssi_scan_();
      break;

    case State::CHANNEL_SELECT:
      handle_channel_select_();
      break;

    /* ---- Pairing handshake ---- */
    case State::PAIR_PROBE_TX:
      handle_pair_probe_tx_();
      break;

    case State::PAIR_PROBE_WAIT_TX_DONE:
      handle_wait_tx_done_(State::PAIR_PROBE_WAIT_RESPONSE);
      break;

    case State::PAIR_PROBE_WAIT_RESPONSE: {
      RxStatus status = poll_rx_();
      if (status == RxStatus::COMPLETE) {
        uint8_t payload[MAX_DLMS_APDU_SIZE];
        RfFrameType rx_type;
        int n = rf_.parse_frame(rx_accum_buf_, rx_accum_len_, payload, sizeof(payload), &rx_type);
        if (n >= 0 && static_cast<uint8_t>(rx_type) == 0x06) {
          ESP_LOGI(TAG, "Pairing: meter answered probe (0x06)");
          capture_meter_address_();
          pair_retry_ = 0;  // reset for the ACK phase
          set_state_(State::PAIR_ACK_TX);
        } else {
          ESP_LOGW(TAG, "Pairing: unexpected probe response (type=0x%02X, n=%d)",
                   n >= 0 ? static_cast<uint8_t>(rx_type) : 0, n);
          set_state_(State::PAIR_PROBE_TX);  // retry
        }
      } else if (status == RxStatus::ERROR) {
        ESP_LOGW(TAG, "Pairing: no probe response");
        set_state_(State::PAIR_PROBE_TX);  // retry (bounded in handler)
      }
      break;
    }

    case State::PAIR_ACK_TX:
      handle_pair_ack_tx_();
      break;

    case State::PAIR_ACK_WAIT_TX_DONE:
      handle_wait_tx_done_(State::PAIR_ACK_WAIT_RESPONSE);
      break;

    case State::PAIR_ACK_WAIT_RESPONSE: {
      RxStatus status = poll_rx_();
      if (status == RxStatus::COMPLETE) {
        uint8_t payload[MAX_DLMS_APDU_SIZE];
        RfFrameType rx_type;
        int n = rf_.parse_frame(rx_accum_buf_, rx_accum_len_, payload, sizeof(payload), &rx_type);
        uint8_t t = (n >= 0) ? static_cast<uint8_t>(rx_type) : 0;
        if (n >= 0 && t == 0x53) {
          // SESSION_SETUP — the meter ships a key blob encrypted with its
          // per-CIU factory pairing key (which we don't have). We don't need
          // it: normal traffic uses the meter-serial-derived key we already
          // hold. Acknowledge with the mode-6 plain reply and move on.
          ESP_LOGI(TAG, "Pairing: SESSION_SETUP received (0x53) — proceeding");
          set_state_(State::PAIR_MODE6_TX);
        } else if (n >= 0 && (t == 0x5B || t == 0x40)) {
          // Some meters skip straight to keepalive/ack — treat as paired.
          ESP_LOGI(TAG, "Pairing: meter ready (0x%02X) — paired", t);
          paired_ = true;
          set_state_(State::BEACON_TX);
        } else {
          ESP_LOGW(TAG, "Pairing: unexpected ACK response (type=0x%02X, n=%d)", t, n);
          set_state_(State::PAIR_ACK_TX);  // retry
        }
      } else if (status == RxStatus::ERROR) {
        ESP_LOGW(TAG, "Pairing: no SESSION_SETUP response");
        set_state_(State::PAIR_ACK_TX);  // retry (bounded in handler)
      }
      break;
    }

    case State::PAIR_MODE6_TX:
      handle_pair_mode6_tx_();
      break;

    case State::PAIR_MODE6_WAIT_TX_DONE:
      handle_wait_tx_done_(State::PAIR_WAIT_KEEPALIVE);
      break;

    case State::PAIR_WAIT_KEEPALIVE: {
      RxStatus status = poll_rx_();
      if (status == RxStatus::COMPLETE) {
        uint8_t payload[MAX_DLMS_APDU_SIZE];
        RfFrameType rx_type;
        int n = rf_.parse_frame(rx_accum_buf_, rx_accum_len_, payload, sizeof(payload), &rx_type);
        ESP_LOGI(TAG, "Pairing complete (meter replied 0x%02X). Now paired.",
                 n >= 0 ? static_cast<uint8_t>(rx_type) : 0);
        paired_ = true;
        set_state_(State::BEACON_TX);
      } else if (status == RxStatus::ERROR) {
        // Keepalive is the meter's "I'm ready" nudge; if we miss it, still
        // consider pairing done and try the beacon poll (it has its own retry).
        ESP_LOGW(TAG, "Pairing: no keepalive — proceeding to beacon anyway");
        paired_ = true;
        set_state_(State::BEACON_TX);
      }
      break;
    }

    case State::BEACON_TX:
      handle_beacon_tx_();
      break;

    case State::BEACON_WAIT_TX_DONE:
      handle_wait_tx_done_(State::BEACON_WAIT_RESPONSE);
      break;

    case State::BEACON_WAIT_RESPONSE: {
      RxStatus status = poll_rx_();
      if (status == RxStatus::COMPLETE) {
        ESP_LOGI(TAG, "Beacon response received (%d bytes)", (int) rx_accum_len_);
        // Parse and validate beacon response (ACK frame)
        uint8_t payload[MAX_DLMS_APDU_SIZE];
        RfFrameType rx_type;
        int payload_len = rf_.parse_frame(rx_accum_buf_, rx_accum_len_, payload, sizeof(payload), &rx_type);
        if (payload_len >= 0) {
          ESP_LOGI(TAG, "Beacon response OK (type=0x%02X, payload=%d bytes)", static_cast<uint8_t>(rx_type), payload_len);
          if (payload_len > 0) {
            ESP_LOGD(TAG, "Beacon payload: %s", format_hex_pretty(payload, payload_len).c_str());
          }
          // No separate AARQ — go directly to data requests
          set_state_(State::GET_TX);
        } else {
          ESP_LOGW(TAG, "Failed to parse beacon response");
          finish_rx_();
          set_state_(State::ERROR_RECOVERY);
        }
      } else if (status == RxStatus::ERROR) {
        ESP_LOGW(TAG, "Beacon RX error/timeout");
        set_state_(State::ERROR_RECOVERY);
      }
      break;
    }

    case State::GET_TX:
      handle_get_tx_();
      break;

    case State::GET_WAIT_TX_DONE:
      handle_wait_tx_done_(State::GET_WAIT_RESPONSE);
      break;

    case State::GET_WAIT_RESPONSE: {
      RxStatus status = poll_rx_();
      if (status == RxStatus::COMPLETE) {
        handle_get_response_();
      } else if (status == RxStatus::ERROR) {
        ESP_LOGW(TAG, "Response timeout/error for sensor %d", current_sensor_idx_);
        retry_count_++;
        if (retry_count_ >= MAX_RETRIES_) {
          ESP_LOGW(TAG, "Max retries for sensor %d, skipping", current_sensor_idx_);
          current_sensor_idx_++;
          retry_count_ = 0;
          if (current_sensor_idx_ >= sensors_.size()) {
            set_state_(State::PUBLISH);
          } else {
            set_state_(State::GET_TX);
          }
        } else {
          set_state_(State::GET_TX);  // Retry
        }
      }
      break;
    }

    case State::ACK_TX:
      handle_ack_tx_();
      break;

    case State::ACK_WAIT_TX_DONE: {
      uint32_t ack_elapsed = esphome::millis() - state_entered_ms_;
      if (hal_.is_tx_done()) {
        ESP_LOGD(TAG, "ACK TX complete");
        hal_.clear_interrupt_flags();
        advance_after_get_();
      } else if (ack_elapsed > 1000) {
        ESP_LOGW(TAG, "ACK TX timeout");
        advance_after_get_();
      }
      break;
    }

    case State::PUBLISH:
      handle_publish_();
      break;

    case State::ERROR_RECOVERY:
      handle_error_recovery_();
      break;

    default:
      break;
  }
}

/* ================================================================
 * State Handlers
 * ================================================================ */

void NartisRfMeterComponent::handle_rssi_scan_() {
  // Firmware (rssi_channel_select / 0x134a4) takes 7 RSSI readings per channel,
  // removes min and max outliers, averages the remaining 4.
  if (rssi_scan_ch_ == 0) {
    ESP_LOGD(TAG, "Scanning RSSI on 4 channels...");
  }

  hal_.set_frequency_channel(rssi_scan_ch_);
  hal_.go_standby();
  hal_.set_rssi_mode(true);  // RSSI-valid mode so get_rssi_dbm() reads real values
  hal_.go_rx();

  // Take 7 readings with 2ms spacing (matching firmware)
  int8_t readings[7];
  for (int i = 0; i < 7; i++) {
    esphome::delay(2);
    readings[i] = hal_.get_rssi_dbm();
  }
  hal_.go_standby();

  // Remove min and max, average remaining 4
  int8_t min_r = readings[0], max_r = readings[0];
  int sum = 0;
  for (int i = 0; i < 7; i++) {
    sum += readings[i];
    if (readings[i] < min_r) min_r = readings[i];
    if (readings[i] > max_r) max_r = readings[i];
  }
  rssi_readings_[rssi_scan_ch_] = static_cast<int8_t>((sum - min_r - max_r) / 5);

  ESP_LOGD(TAG, "  CH%d RSSI: %d dBm (trimmed mean of 7)", rssi_scan_ch_, rssi_readings_[rssi_scan_ch_]);

  rssi_scan_ch_++;
  if (rssi_scan_ch_ >= NUM_CHANNELS) {
    hal_.set_rssi_mode(false);  // restore merged-FIFO mode for TX/RX
    rssi_scan_ch_ = 0;
    set_state_(State::CHANNEL_SELECT);
  }
}

void NartisRfMeterComponent::handle_channel_select_() {
  uint8_t best = RfDataLayer::select_best_channel(rssi_readings_);
  rf_.set_channel(best);
  hal_.set_frequency_channel(best);
  ESP_LOGI(TAG, "Selected channel %d (RSSI: %d dBm)", best, rssi_readings_[best]);
  // First contact requires the pairing handshake; once paired this boot we
  // go straight to the beacon/poll cycle.
  if (paired_) {
    set_state_(State::BEACON_TX);
  } else {
    pair_retry_ = 0;
    set_state_(State::PAIR_PROBE_TX);
  }
}

/* ================================================================
 * Pairing handshake
 *
 * First-contact sequence reverse-engineered from the dump-spi4 capture
 * of a real CIU pairing with an I100 meter:
 *
 *   CIU → meter : plain mode-1 probe, payload = "0" + 12-digit meter serial
 *   meter → CIU : 0x06 (presence ack)
 *   CIU → meter : encrypted mode-2 ACK, payload = beacon template
 *   meter → CIU : 0x53 SESSION_SETUP (key blob — not needed by us)
 *   CIU → meter : plain mode-6, payload = 12-digit meter serial
 *   meter → CIU : 0x5B keepalive
 *   → normal beacon/GET poll begins (AES key = meter_serial || salt)
 * ================================================================ */

size_t NartisRfMeterComponent::build_pair_probe_payload_(uint8_t *out, size_t max) {
  // 13 ASCII bytes: '0' prefix + 12-digit meter serial (verified in
  // dump-spi4 "0021245003137" and dump-spi3 "0000000000000").
  if (max < 1 + meter_serial_.size()) return 0;
  out[0] = '0';
  memcpy(out + 1, meter_serial_.c_str(), meter_serial_.size());
  return 1 + meter_serial_.size();
}

void NartisRfMeterComponent::capture_meter_address_() {
  // The meter's 8-byte RF address sits at rx[2..9] of any valid response.
  // Capture it once so the RF layer can filter subsequent traffic to this
  // meter (also helps reject the ambient-noise frames seen on RX).
  if (rx_accum_len_ < RF_RX_ADDR + 8) return;
  RfAddress meter = RfAddress::from_bytes(rx_accum_buf_ + RF_RX_ADDR);
  rf_.set_meter_address(meter);
  uint8_t b[8];
  meter.to_bytes(b);
  ESP_LOGI(TAG, "Meter address learned: %s", format_hex_pretty(b, 8).c_str());
}

void NartisRfMeterComponent::handle_pair_probe_tx_() {
  if (pair_retry_ >= MAX_PAIR_RETRIES_) {
    // No probe answer. The meter may already be paired (some only answer the
    // beacon once paired) — fall back to the beacon/poll path instead of
    // failing outright. If that also fails it lands in ERROR_RECOVERY.
    ESP_LOGW(TAG, "Pairing: no probe answer after %d tries — trying beacon poll "
                  "(meter may already be paired, or not in pairing mode)", pair_retry_);
    paired_ = true;
    set_state_(State::BEACON_TX);
    return;
  }
  pair_retry_++;
  ESP_LOGI(TAG, "Pairing: sending probe %d/%d for meter '%s'...",
           pair_retry_, MAX_PAIR_RETRIES_, meter_serial_.c_str());

  uint8_t payload[16];
  size_t payload_len = build_pair_probe_payload_(payload, sizeof(payload));
  if (payload_len == 0) {
    set_state_(State::ERROR_RECOVERY);
    return;
  }
  // Plain mode-1 DATA frame (flags 0x46, marker 0x7A, enc_flag 0).
  size_t frame_len = rf_.build_frame(tx_buf_.data(), tx_buf_.size(),
                                     RfFrameType::DATA, sequence_nr_++,
                                     payload, payload_len);
  if (frame_len == 0 || !transmit_frame_(RfFrameType::DATA, tx_buf_.data(), frame_len)) {
    set_state_(State::ERROR_RECOVERY);
    return;
  }
  set_state_(State::PAIR_PROBE_WAIT_TX_DONE);
}

void NartisRfMeterComponent::handle_pair_ack_tx_() {
  if (pair_retry_ >= MAX_PAIR_RETRIES_) {
    ESP_LOGW(TAG, "Pairing failed: no SESSION_SETUP after %d ACKs", pair_retry_);
    set_state_(State::ERROR_RECOVERY);
    return;
  }
  pair_retry_++;
  ESP_LOGI(TAG, "Pairing: sending encrypted ACK %d/%d...", pair_retry_, MAX_PAIR_RETRIES_);
  uint8_t ack_payload[29];
  size_t payload_len = build_beacon_payload_(ack_payload, sizeof(ack_payload));
  if (payload_len == 0) {
    set_state_(State::ERROR_RECOVERY);
    return;
  }
  // Encrypted mode-2 ACK frame (flags 0x44).
  size_t frame_len = rf_.build_frame(tx_buf_.data(), tx_buf_.size(),
                                     RfFrameType::ACK, sequence_nr_++,
                                     ack_payload, payload_len);
  if (frame_len == 0 || !transmit_frame_(RfFrameType::ACK, tx_buf_.data(), frame_len)) {
    set_state_(State::ERROR_RECOVERY);
    return;
  }
  set_state_(State::PAIR_ACK_WAIT_TX_DONE);
}

void NartisRfMeterComponent::handle_pair_mode6_tx_() {
  ESP_LOGI(TAG, "Pairing: sending mode-6 confirmation...");
  // Plain mode-6 frame (flags 0x00, marker 0x8A), payload = 12-digit serial.
  size_t frame_len = rf_.build_frame(
      tx_buf_.data(), tx_buf_.size(), RfFrameType::PLAIN_DATA, sequence_nr_++,
      reinterpret_cast<const uint8_t *>(meter_serial_.c_str()), meter_serial_.size());
  if (frame_len == 0 || !transmit_frame_(RfFrameType::PLAIN_DATA, tx_buf_.data(), frame_len)) {
    set_state_(State::ERROR_RECOVERY);
    return;
  }
  set_state_(State::PAIR_MODE6_WAIT_TX_DONE);
}

void NartisRfMeterComponent::handle_beacon_tx_() {
  ESP_LOGI(TAG, "Sending beacon (frame_counter=%u, seq=%d)...", (unsigned) rf_.get_frame_counter(), sequence_nr_);

  // Build 29-byte beacon payload matching firmware beacon_address_builder (0xB5B0)
  uint8_t beacon_payload[29];
  size_t payload_len = build_beacon_payload_(beacon_payload, sizeof(beacon_payload));
  if (payload_len == 0) {
    ESP_LOGE(TAG, "Failed to build beacon payload");
    set_state_(State::ERROR_RECOVERY);
    return;
  }

  ESP_LOGD(TAG, "Beacon payload (%d bytes): %s", (int) payload_len,
           format_hex_pretty(beacon_payload, payload_len).c_str());

  size_t frame_len = rf_.build_frame(tx_buf_.data(), tx_buf_.size(),
                                     RfFrameType::BEACON, 0x00, sequence_nr_++,
                                     beacon_payload, payload_len);
  if (frame_len == 0) {
    ESP_LOGE(TAG, "Failed to build beacon frame");
    set_state_(State::ERROR_RECOVERY);
    return;
  }

  if (!transmit_frame_(RfFrameType::BEACON, tx_buf_.data(), frame_len)) {
    set_state_(State::ERROR_RECOVERY);
    return;
  }

  set_state_(State::BEACON_WAIT_TX_DONE);
}

void NartisRfMeterComponent::handle_wait_tx_done_(State next_state) {
  uint32_t elapsed = esphome::millis() - state_entered_ms_;

  if (hal_.is_tx_done()) {
    ESP_LOGD(TAG, "TX complete");
    hal_.clear_interrupt_flags();

    // Switch to chunked RX mode for response
    start_rx_();

    set_state_(next_state);
  } else if (elapsed > 1000) {
    ESP_LOGW(TAG, "TX timeout");
    set_state_(State::ERROR_RECOVERY);
  }
}


void NartisRfMeterComponent::handle_get_tx_() {
  if (current_sensor_idx_ >= sensors_.size()) {
    set_state_(State::PUBLISH);
    return;
  }

  const auto &entry = sensors_[current_sensor_idx_];
  ESP_LOGI(TAG, "Read request for sensor %d/%d — OBIS %d.%d.%d.%d.%d.%d class=%d attr=%d",
           current_sensor_idx_ + 1, (int) sensors_.size(),
           entry.obis.bytes[0], entry.obis.bytes[1], entry.obis.bytes[2],
           entry.obis.bytes[3], entry.obis.bytes[4], entry.obis.bytes[5],
           entry.class_id, entry.attr_id);

  // Build proprietary read request (C0 01 C1 00 [class] [obis] [attr] 00)
  uint8_t apdu[16];
  size_t apdu_len = dlms_.build_read_request(apdu, sizeof(apdu),
                                             entry.obis, entry.class_id, entry.attr_id);
  if (apdu_len == 0) {
    ESP_LOGE(TAG, "Failed to build read request");
    current_sensor_idx_++;
    return;
  }

  // Wrap in RF data frame (plaintext — no RF-level encryption for DATA)
  size_t frame_len = rf_.build_frame(tx_buf_.data(), tx_buf_.size(),
                                     RfFrameType::DATA, 0x00, sequence_nr_++,
                                     apdu, apdu_len);
  if (frame_len == 0) {
    current_sensor_idx_++;
    return;
  }

  if (!transmit_frame_(RfFrameType::DATA, tx_buf_.data(), frame_len)) {
    set_state_(State::ERROR_RECOVERY);
    return;
  }

  set_state_(State::GET_WAIT_TX_DONE);
}

void NartisRfMeterComponent::handle_get_response_() {
  ESP_LOGI(TAG, "GET response received (%d bytes) for sensor %d", (int) rx_accum_len_, current_sensor_idx_);

  uint8_t payload[MAX_DLMS_APDU_SIZE];
  RfFrameType rx_type;

  int payload_len = rf_.parse_frame(rx_accum_buf_, rx_accum_len_, payload, sizeof(payload), &rx_type);
  if (payload_len < 0) {
    ESP_LOGW(TAG, "Failed to parse response frame (type=0x%02X)", rx_accum_len_ > 1 ? rx_accum_buf_[1] : 0);
    retry_count_++;
    if (retry_count_ >= MAX_RETRIES_) {
      current_sensor_idx_++;
      retry_count_ = 0;
    }
    // Still send ACK even on parse failure — meter expects it
    set_state_(State::ACK_TX);
    return;
  }

  ESP_LOGD(TAG, "GET payload (%d bytes, type=0x%02X): %s",
           payload_len, static_cast<uint8_t>(rx_type),
           format_hex_pretty(payload, payload_len).c_str());

  DlmsValue value;
  if (dlms_.parse_read_response(payload, payload_len, &value)) {
    sensors_[current_sensor_idx_].last_value = value;
    ESP_LOGI(TAG, "Sensor %d value OK (type=%d)", current_sensor_idx_, value.type);
  } else {
    ESP_LOGW(TAG, "Failed to parse read response data");
  }

  current_sensor_idx_++;
  retry_count_ = 0;

  // Send ACK before next request (firmware sends ACK after every response)
  set_state_(State::ACK_TX);
}

void NartisRfMeterComponent::handle_ack_tx_() {
  ESP_LOGD(TAG, "Sending ACK for GET response...");

  // ACK frame: Mode 2 (header 0x44), RF AES-CCM encrypted, empty payload
  size_t frame_len = rf_.build_frame(tx_buf_.data(), tx_buf_.size(),
                                     RfFrameType::ACK, 0x00, sequence_nr_++,
                                     nullptr, 0);
  if (frame_len == 0) {
    ESP_LOGW(TAG, "Failed to build ACK frame");
    advance_after_get_();
    return;
  }

  finish_rx_();  // Stop RX before switching to TX

  if (!transmit_frame_(RfFrameType::ACK, tx_buf_.data(), frame_len)) {
    advance_after_get_();
    return;
  }

  set_state_(State::ACK_WAIT_TX_DONE);
}

void NartisRfMeterComponent::advance_after_get_() {
  if (current_sensor_idx_ >= sensors_.size()) {
    set_state_(State::PUBLISH);
  } else {
    set_state_(State::GET_TX);
  }
}

void NartisRfMeterComponent::handle_publish_() {
  ESP_LOGI(TAG, "Publishing sensor values...");

  for (auto &entry : sensors_) {
    if (!entry.last_value.has_value()) continue;

    if (entry.sensor) {
      float val = 0.0f;
      switch (entry.last_value.type) {
        case DlmsValue::FLOAT_VAL:
          val = entry.last_value.float_val;
          break;
        case DlmsValue::INT_VAL:
          val = static_cast<float>(entry.last_value.int_val);
          break;
        case DlmsValue::UINT_VAL:
          val = static_cast<float>(entry.last_value.uint_val);
          break;
        default:
          continue;
      }
      entry.sensor->publish_state(val);
    }

    if (entry.text_sensor) {
      if (entry.last_value.type == DlmsValue::STRING_VAL) {
        entry.text_sensor->publish_state(entry.last_value.str_val);
      } else {
        // Convert numeric to string
        char buf[32];
        if (entry.last_value.type == DlmsValue::FLOAT_VAL)
          snprintf(buf, sizeof(buf), "%.3f", entry.last_value.float_val);
        else if (entry.last_value.type == DlmsValue::INT_VAL)
          snprintf(buf, sizeof(buf), "%d", (int) entry.last_value.int_val);
        else if (entry.last_value.type == DlmsValue::UINT_VAL)
          snprintf(buf, sizeof(buf), "%u", (unsigned) entry.last_value.uint_val);
        else
          continue;
        entry.text_sensor->publish_state(buf);
      }
    }
  }

  // Put radio to sleep between readings
  hal_.go_sleep();
  dlms_.reset();
  set_state_(State::IDLE);
  ESP_LOGI(TAG, "Read cycle complete");
}

void NartisRfMeterComponent::handle_error_recovery_() {
  ESP_LOGW(TAG, "Error recovery — resetting radio");
  finish_rx_();
  hal_.go_standby();
  hal_.clear_interrupt_flags();
  hal_.clear_fifo();
  dlms_.reset();

  // Put radio to sleep
  hal_.go_sleep();
  set_state_(State::IDLE);
}

/* ================================================================
 * TX/RX Helpers
 * ================================================================ */

bool NartisRfMeterComponent::transmit_frame_(RfFrameType type,
                                             const uint8_t *frame, size_t len) {
  ESP_LOGI(TAG, "TX frame (%d bytes, type=0x%02X):", (int) len, static_cast<uint8_t>(type));
  ESP_LOGD(TAG, "TX: %s", format_hex_pretty(frame, len).c_str());
  // Use chunked TX for all frames — handles packets > 64 bytes
  // (AARQ can be ~80-100B after CRC framing, beacons ~60-70B)
  return hal_.transmit_chunked(frame, len);
}

int NartisRfMeterComponent::receive_frame_(uint8_t *payload_out, size_t max,
                                           RfFrameType *type_out) {
  // Use accumulated RX buffer from chunked reception
  return rf_.parse_frame(rx_accum_buf_, rx_accum_len_, payload_out, max, type_out);
}

/* ================================================================
 * RF Address Derivation
 * ================================================================ */

void NartisRfMeterComponent::derive_rf_address_() {
  uint8_t mac[6];
  // ESPHome's get_mac_address_raw() works on ESP32, ESP8266, RP2040, etc.
  esphome::get_mac_address_raw(mac);
  uint32_t seed = (static_cast<uint32_t>(mac[2]) << 24) |
                  (static_cast<uint32_t>(mac[3]) << 16) |
                  (static_cast<uint32_t>(mac[4]) << 8) |
                  static_cast<uint32_t>(mac[5]);

  if (!ciu_serial_.empty()) {
    address_ = RfAddress::derive(ciu_serial_.c_str(), seed);
  } else {
    // No CIU serial provided: generate pseudo-serial from ESP32 MAC
    char mac_serial[16];
    snprintf(mac_serial, sizeof(mac_serial), "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    address_ = RfAddress::derive(mac_serial, seed);
  }

  ESP_LOGI(TAG, "RF address derived: hash=0x%08X (ciu_serial=%s)",
           (unsigned) address_.serial_hash,
           ciu_serial_.empty() ? "<from MAC>" : ciu_serial_.c_str());
}

/* ================================================================
 * Beacon Payload — 29-byte struct matching firmware beacon_address_builder (0xB5B0)
 * ================================================================ */

size_t NartisRfMeterComponent::build_beacon_payload_(uint8_t *out, size_t max) {
  if (max < 29) return 0;

  // 29-byte beacon payload — a fixed Nartis TLV template with a 6-byte RTC
  // timestamp patched in. Reconstructed byte-for-byte from real on-air
  // beacons decrypted out of the SPI captures (fw/dump-spi2), which is the
  // authoritative source: those beacons are the ones the meter actually
  // answered. The firmware builds this from flash templates at 0xCA1C/
  // 0xCA20/0xCA24/0xCA28 plus the "1234" constant at 0xB620 — NOT from the
  // RF address (the earlier implementation's mistake).
  //
  //   [0..3]   0d fd 0d 04        TLV: tag 0x0dfd, type 0x0d, len 4
  //   [4..7]   "1234"             4-byte payload (the static pairing PIN)
  //   [8..11]  0d fd 0f 02        TLV: tag 0x0dfd, type 0x0f, len 2
  //   [12..13] 02 05              that TLV's 2-byte value
  //   [14..15] 06 6d              i32 high half (constant 0x066d)
  //   [16..21] RTC timestamp      6-byte bit-packed clock (varies per beacon)
  //   [22..24] 04 fd 17           TLV: tag 0x04fd, len/type 0x17
  //   [25..28] 00 00 00 00        trailing u32 (always 0 in captures)
  static constexpr uint8_t BEACON_HEAD[16] = {
      0x0D, 0xFD, 0x0D, 0x04, 0x31, 0x32, 0x33, 0x34,
      0x0D, 0xFD, 0x0F, 0x02, 0x02, 0x05, 0x06, 0x6D};
  static constexpr uint8_t BEACON_TAIL[7] = {0x04, 0xFD, 0x17, 0x00, 0x00, 0x00, 0x00};

  memcpy(out, BEACON_HEAD, sizeof(BEACON_HEAD));
  build_rtc_timestamp_(out + 16);          // [16..21]
  memcpy(out + 22, BEACON_TAIL, sizeof(BEACON_TAIL));

  return 29;
}

void NartisRfMeterComponent::build_rtc_timestamp_(uint8_t *out) {
  time_t epoch = ::time(nullptr);
  auto now = esphome::ESPTime::from_epoch_local(epoch);
  if (!now.is_valid()) {
    memset(out, 0, 6);
    return;
  }

  uint8_t yr  = now.year % 100;
  uint8_t mo  = now.month;
  uint8_t day = now.day_of_month;
  uint8_t hr  = now.hour;
  uint8_t mn  = now.minute;
  uint8_t sec = now.second;
  uint8_t dow = now.day_of_week;
  if (dow == 0) dow = 7;  // firmware uses 1-7, Sunday=7

  // Bit-packed into 6 bytes matching firmware rtc_timestamp_pack (0x102FA)
  out[0] = yr & 0x3F;
  out[1] = mo & 0x3F;
  out[2] = (day & 0x1F) | (dow << 5);
  out[3] = (hr & 0x1F) | (sec << 5);
  out[4] = ((sec >> 3) << 4) | (mn & 0x0F);
  out[5] = (mn >> 4) & 0x0F;
}

/* ================================================================
 * Chunked RX — ISR reads 12-byte FIFO chunks, main loop accumulates
 * ================================================================ */

void NartisRfMeterComponent::start_rx_() {
  // Reset accumulation state
  rx_accum_len_ = 0;
  rx_expected_len_ = 0;
  rx_active_ = true;

  hal_.go_standby();

  // CRITICAL: the preceding TX left the merged FIFO in TX direction with the
  // 64-byte beacon still "occupying" it. Switch the merged FIFO back to RX
  // direction FIRST, then fully reset it (clear + RESTORE pointers) — otherwise
  // RX starts against a FIFO that already reads full/overflowed (FIFO_FLAG=0xFF)
  // and we drain stale garbage. Order matters: direction → restore → clear.
  hal_.prepare_rx_session();   // RX FIFO direction + SDIO input
  hal_.reset_rx_fifo_full();   // FIFO_RESTORE + clear RX/TX
  hal_.clear_interrupt_flags();

  // Route the INT line → RX_FIFO_TH (the firmware's RX mechanism). The pin
  // goes HIGH while >= FIFO_TH_VALUE bytes await in the RX FIFO; we drain
  // threshold chunks on that, and read the trailing sub-threshold bytes by
  // frame length. (TX left it on TX_FIFO_TH; this restores it for RX.)
  hal_.set_int1_source(INT_SEL_RX_FIFO_TH);
  rx_tail_wait_ms_ = 0;

  // Enter RX mode: STBY → RFS → RX
  hal_.write_reg(REG_MODE_CTL, GO_RFS);
  hal_.wait_for_state(STA_RFS);
  hal_.write_reg(REG_MODE_CTL, GO_RX);
  hal_.wait_for_state(STA_RX);

  ESP_LOGD(TAG, "RX started (GPIO1/PKT_DONE polled, 4800 bps)");
}

NartisRfMeterComponent::RxStatus NartisRfMeterComponent::poll_rx_() {
  if (!rx_active_) return RxStatus::ERROR;

  uint32_t elapsed = esphome::millis() - state_entered_ms_;

  // Drain whatever the chip has in its FIFO right now (SPI poll).
  size_t drained = hal_.poll_rx_drain(rx_accum_buf_ + rx_accum_len_,
                                       MAX_RF_FRAME_SIZE - rx_accum_len_);
  if (drained > 0) {
    ESP_LOGV(TAG, "RX drained +%d bytes (total %d)", (int) drained, (int) (rx_accum_len_ + drained));
  }
  rx_accum_len_ += drained;

  // Extract expected length from first byte once we have it
  if (rx_accum_len_ >= 1 && rx_expected_len_ == 0) {
    rx_expected_len_ = static_cast<size_t>(rx_accum_buf_[0]) + 1;
    if (rx_expected_len_ > MAX_RF_FRAME_SIZE) {
      ESP_LOGW(TAG, "RX frame too large: %d bytes", (int) rx_expected_len_);
      finish_rx_();
      return RxStatus::ERROR;
    }
    ESP_LOGD(TAG, "RX expecting %d bytes total", (int) rx_expected_len_);
  }

  // Tail read: the final < FIFO_TH_VALUE bytes never raise RX_FIFO_TH, so the
  // chunk-drain above can't fetch them. Once we know the length and only the
  // sub-threshold tail remains, read it directly by length — but wait briefly
  // first so those bytes have actually been received (at 4800 bps ~600 B/s, up
  // to 11 trailing bytes take <20 ms to arrive).
  if (rx_expected_len_ > 0 && rx_accum_len_ < rx_expected_len_) {
    size_t remaining = rx_expected_len_ - rx_accum_len_;
    if (remaining < FIFO_TH_VALUE) {
      if (rx_tail_wait_ms_ == 0) {
        rx_tail_wait_ms_ = esphome::millis();
      } else if (esphome::millis() - rx_tail_wait_ms_ >= RX_TAIL_WAIT_MS_) {
        hal_.read_fifo(rx_accum_buf_ + rx_accum_len_, remaining);
        rx_accum_len_ += remaining;
      }
    }
  }

  // Check if we have the complete packet
  if (rx_expected_len_ > 0 && rx_accum_len_ >= rx_expected_len_) {
    // Trim to expected length (last chunk may have extra bytes)
    rx_accum_len_ = rx_expected_len_;
    ESP_LOGI(TAG, "RX complete: %d bytes", (int) rx_accum_len_);
    ESP_LOGD(TAG, "RX: %s", format_hex_pretty(rx_accum_buf_, rx_accum_len_).c_str());
    finish_rx_();
    return RxStatus::COMPLETE;
  }

  // Check for buffer overflow
  if (rx_accum_len_ >= MAX_RF_FRAME_SIZE) {
    ESP_LOGW(TAG, "RX buffer overflow (%d bytes)", (int) rx_accum_len_);
    finish_rx_();
    return RxStatus::ERROR;
  }

  // Check timeout
  if (elapsed > RX_TIMEOUT_MS_) {
    ESP_LOGW(TAG, "RX timeout (%d/%d bytes received in %dms)",
             (int) rx_accum_len_, (int) rx_expected_len_, (int) elapsed);
    if (rx_accum_len_ > 0) {
      ESP_LOGD(TAG, "RX partial: %s", format_hex_pretty(rx_accum_buf_, rx_accum_len_).c_str());
    }
    finish_rx_();
    return RxStatus::ERROR;
  }

  return RxStatus::IN_PROGRESS;
}

void NartisRfMeterComponent::finish_rx_() {
  if (!rx_active_) return;

  // Stop reception
  hal_.go_standby();
  hal_.clear_interrupt_flags();

  rx_active_ = false;
}

}  // namespace esphome::nartis_rf_meter
