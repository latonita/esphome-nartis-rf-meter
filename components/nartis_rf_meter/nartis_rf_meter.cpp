/*
 * Nartis RF Meter — ESPHome Component Implementation
 *
 * Non-blocking state machine orchestrating CMT2300A HAL, RF framing, and DLMS client.
 */

#include "nartis_rf_meter.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/core/time.h"

#include <esp_mac.h>
#include <cstring>
#include <ctime>

namespace esphome::nartis_rf_meter {

static const char *const TAG = "nartis_rf_meter";

/* ================================================================
 * ESPHome Lifecycle
 * ================================================================ */

void NartisRfMeterComponent::setup() {
  ESP_LOGI(TAG, "Setting up Nartis RF Meter...");

  // Initialize HAL
  hal_.set_pins(pin_sdio_, pin_sclk_, pin_csb_, pin_fcsb_, pin_gpio1_);
  if (!hal_.init()) {
    ESP_LOGE(TAG, "CMT2300A initialization failed!");
    this->mark_failed();
    return;
  }

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

void NartisRfMeterComponent::set_aes_key(const std::string &key) {
  memset(aes_key_, 0, AES_KEY_SIZE);
  size_t copy_len = key.size();
  if (copy_len > AES_KEY_SIZE) copy_len = AES_KEY_SIZE;
  memcpy(aes_key_, key.c_str(), copy_len);
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

void NartisRfMeterComponent::set_state_(State new_state) {
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

    case State::BEACON_TX:
      handle_beacon_tx_();
      break;

    case State::BEACON_WAIT_TX_DONE:
      handle_wait_tx_done_(State::BEACON_WAIT_RESPONSE);
      break;

    case State::BEACON_WAIT_RESPONSE: {
      RxStatus status = poll_rx_();
      if (status == RxStatus::COMPLETE) {
        ESP_LOGD(TAG, "Beacon response received (%d bytes)", (int) rx_accum_len_);
        // Parse and validate beacon response (ACK frame)
        uint8_t payload[MAX_DLMS_APDU_SIZE];
        RfFrameType rx_type;
        int payload_len = rf_.parse_frame(rx_accum_buf_, rx_accum_len_, payload, sizeof(payload), &rx_type);
        if (payload_len >= 0) {
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
    rssi_scan_ch_ = 0;
    set_state_(State::CHANNEL_SELECT);
  }
}

void NartisRfMeterComponent::handle_channel_select_() {
  uint8_t best = RfDataLayer::select_best_channel(rssi_readings_);
  rf_.set_channel(best);
  hal_.set_frequency_channel(best);
  ESP_LOGI(TAG, "Selected channel %d (RSSI: %d dBm)", best, rssi_readings_[best]);
  set_state_(State::BEACON_TX);
}

void NartisRfMeterComponent::handle_beacon_tx_() {
  ESP_LOGD(TAG, "Sending beacon...");

  // Build 29-byte beacon payload matching firmware beacon_address_builder (0xB5B0)
  uint8_t beacon_payload[29];
  size_t payload_len = build_beacon_payload_(beacon_payload, sizeof(beacon_payload));
  if (payload_len == 0) {
    ESP_LOGE(TAG, "Failed to build beacon payload");
    set_state_(State::ERROR_RECOVERY);
    return;
  }

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
  ESP_LOGD(TAG, "Read request for sensor %d/%d", current_sensor_idx_ + 1, (int) sensors_.size());

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
  uint8_t payload[MAX_DLMS_APDU_SIZE];
  RfFrameType rx_type;

  int payload_len = rf_.parse_frame(rx_accum_buf_, rx_accum_len_, payload, sizeof(payload), &rx_type);
  if (payload_len < 0) {
    ESP_LOGW(TAG, "Failed to parse response frame");
    retry_count_++;
    if (retry_count_ >= MAX_RETRIES_) {
      current_sensor_idx_++;
      retry_count_ = 0;
    }
    // Still send ACK even on parse failure — meter expects it
    set_state_(State::ACK_TX);
    return;
  }

  DlmsValue value;
  if (dlms_.parse_read_response(payload, payload_len, &value)) {
    sensors_[current_sensor_idx_].last_value = value;
    ESP_LOGD(TAG, "Got value for sensor %d (type=%d)", current_sensor_idx_, value.type);
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
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
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

  uint8_t addr_bytes[8];
  address_.to_bytes(addr_bytes);

  // [0-3] meter address (serial_hash bytes = addr[2..5])
  memcpy(out, addr_bytes + 2, 4);

  // [4-8] static factory bytes "1234\0"
  memcpy(out + 4, "1234", 5);  // includes null terminator

  // [8-11] extended address (device_id LE + group + type = addr[0,1,6,7])
  out[8]  = addr_bytes[0];
  out[9]  = addr_bytes[1];
  out[10] = addr_bytes[6];
  out[11] = addr_bytes[7];

  // [12] frame subtype
  out[12] = 0x07;

  // [13] protocol version
  out[13] = 0x04;

  // [14-15] channel config (2 bytes LE)
  out[14] = rf_.get_channel();
  out[15] = 0x00;

  // [16-21] RTC timestamp (6 bytes, bit-packed)
  build_rtc_timestamp_(out + 16);

  // [22-24] device ID (3 bytes from address)
  out[22] = addr_bytes[0];
  out[23] = addr_bytes[1];
  out[24] = addr_bytes[2];

  // [25-28] mode/sync/seq config bytes
  out[25] = 0x00;           // mode
  out[26] = 0x00;           // sync
  out[27] = sequence_nr_;   // current sequence
  out[28] = 0x00;           // reserved

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

  // Go to standby first
  hal_.go_standby();
  hal_.clear_rx_fifo();
  hal_.clear_interrupt_flags();

  // Set INT1 source to RX_FIFO_TH (auto-clearing, per AN143)
  hal_.set_int1_source(INT_SEL_RX_FIFO_TH);

  // Enter RX mode: STBY → RFS → RX
  // We do manual state transitions instead of go_rx() to control
  // the SDIO direction setup and ISR arming order.
  hal_.write_reg(REG_MODE_CTL, GO_RFS);
  hal_.wait_for_state(STA_RFS);
  hal_.write_reg(REG_MODE_CTL, GO_RX);
  hal_.wait_for_state(STA_RX);

  // Now that we're in RX and won't do any more SPI register writes,
  // set SDIO to input for the ISR (avoids gpio_set_direction spinlock in ISR)
  hal_.prepare_fifo_read();

  // Arm the GPIO interrupt (must be after prepare_fifo_read and after RX mode entered)
  hal_.enable_rx_interrupt();

  ESP_LOGD(TAG, "RX started (chunked mode, FIFO_TH interrupt)");
}

NartisRfMeterComponent::RxStatus NartisRfMeterComponent::poll_rx_() {
  if (!rx_active_) return RxStatus::ERROR;

  uint32_t elapsed = esphome::millis() - state_entered_ms_;

  // Drain any chunks the ISR has queued
  size_t drained = hal_.drain_rx_queue(rx_accum_buf_ + rx_accum_len_,
                                       MAX_RF_FRAME_SIZE - rx_accum_len_);
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

  // Check if we have the complete packet
  if (rx_expected_len_ > 0 && rx_accum_len_ >= rx_expected_len_) {
    ESP_LOGD(TAG, "RX complete: %d/%d bytes", (int) rx_accum_len_, (int) rx_expected_len_);
    finish_rx_();
    // Trim to expected length (last chunk may have extra bytes)
    rx_accum_len_ = rx_expected_len_;
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
    ESP_LOGD(TAG, "RX timeout (%d/%d bytes received)",
             (int) rx_accum_len_, (int) rx_expected_len_);
    finish_rx_();
    return RxStatus::ERROR;
  }

  return RxStatus::IN_PROGRESS;
}

void NartisRfMeterComponent::finish_rx_() {
  if (!rx_active_) return;

  // Disable ISR first
  hal_.disable_rx_interrupt();

  // Stop reception
  hal_.go_standby();
  hal_.clear_interrupt_flags();

  rx_active_ = false;
}

}  // namespace esphome::nartis_rf_meter
