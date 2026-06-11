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

// Defined below (after the lifecycle methods); forward-declared so update() can
// log the current state name.
static const LogString *state_to_str(NartisRfMeterComponent::State s);

/* ================================================================
 * ESPHome Lifecycle
 * ================================================================ */

void NartisRfMeterComponent::setup() {
  ESP_LOGI(TAG, "Nartis RF Meter: deferring init 2s for network log viewers...");
  this->set_timeout(2000, [this]() { this->setup_continue_(); });
}

void NartisRfMeterComponent::setup_continue_() {
  ESP_LOGI(TAG, "Setting up Nartis RF Meter...");

  // Initialize HAL
  hal_.set_pins(pin_sdio_, pin_sclk_, pin_csb_, pin_fcsb_, pin_gpio3_);
  if (!hal_.init()) {
    ESP_LOGE(TAG, "CMT2300A initialization failed!");
    this->mark_failed();
    return;
  }

  // One-time GPIO3/INT2 wiring self-test (decisive: RX is impossible if the
  // chip's interrupt line isn't reaching the configured pin_gpio3).
  hal_.test_gpio3_wiring();

  // Derive RF address from CIU serial (or ESP32 MAC)
  derive_rf_address_();

  // Configure RF data layer
  rf_.set_address(address_);
  rf_.set_aes_key(aes_key_);

  // Configure DLMS client
  dlms_.set_credentials(PASSWORD_, CLIENT_ADDRESS_, SERVER_ADDRESS_);

  // Restore a previously-paired session from NVS (key, counters, meter address).
  // Keyed by meter serial so swapping the configured meter starts fresh. Done
  // AFTER set_aes_key(aes_key_) above so a restored session key overrides the
  // bootstrap key.
  pref_ = esphome::global_preferences->make_preference<NvsPairingState>(
      esphome::fnv1_hash(std::string("nartis_rf_meter_session_") + meter_serial_));
  load_pairing_state_();

  // Ready and idle. Cycles are driven entirely by update() on the polling
  // interval: the first update() tick kicks the first cycle (pairing if not yet
  // paired, otherwise straight to reading); each later tick reuses the session.
  set_state_(State::IDLE);

  ESP_LOGI(TAG, "Nartis RF Meter ready. %d sensor(s) registered.", (int) sensors_.size());
  ESP_LOGI(TAG, "AES key: %s", format_hex_pretty(aes_key_, AES_KEY_SIZE).c_str());
  uint8_t addr_bytes[8];
  address_.to_bytes(addr_bytes);
  ESP_LOGI(TAG, "RF address (8 bytes): %s", format_hex_pretty(addr_bytes, 8).c_str());
}

void NartisRfMeterComponent::start_cycle_() {
  current_sensor_idx_ = 0;
  retry_count_ = 0;
  pair_retry_ = 0;
  rx_parse_retries_ = 0;
  // Reset batch state for this read cycle. session_primed_ persists across
  // *update()* cycles within one boot (the meter only needs the priming opener
  // once per power-on), so it's NOT reset here.
  batch_start_idx_ = 0;
  batch_count_ = 0;
  // With a pinned channel, skip the RSSI scan and go straight to channel setup.  
  set_state_(fix_channel_ >= 0 ? State::CHANNEL_SELECT : State::RSSI_SCAN);
}

void NartisRfMeterComponent::abort_to_idle_(const char *reason) {
  ESP_LOGW(TAG, "%s — returning to IDLE, will retry on next update().", reason);
  finish_rx_();
  hal_.go_sleep();
  dlms_.reset();
  note_cycle_failure_();
  // Persist whatever counter advances this (failed) cycle made so the meter
  // never sees a replayed counter after a reboot. note_cycle_failure_() may have
  // torn the pairing down (reset_pairing_state_ already saved the cleared blob);
  // this save then just refreshes counters for a still-paired session.
  save_pairing_state_();
  set_state_(State::IDLE);
}

void NartisRfMeterComponent::note_cycle_failure_() {
  // Pairing-phase failures self-heal: paired_ is still false, so the next
  // update() re-pairs anyway — don't count those toward the re-pair budget.
  if (!paired_) return;
  consecutive_read_failures_++;
  ESP_LOGW(TAG, "Read cycle failed (%u/%u consecutive)",
           consecutive_read_failures_, MAX_READ_FAILURES_BEFORE_REPAIR_);
  if (consecutive_read_failures_ >= MAX_READ_FAILURES_BEFORE_REPAIR_) {
    ESP_LOGW(TAG, "Too many failed reads — dropping pairing; will re-pair next cycle.");
    reset_pairing_state_();
  }
}

void NartisRfMeterComponent::reset_pairing_state_() {
  paired_ = false;
  session_primed_ = false;
  consecutive_read_failures_ = 0;
  // Restart the TX sequence counters at their fresh-pairing values. aes_key_
  // still holds the bootstrap key (only rf_'s internal key was overwritten by
  // the meter-assigned session key at 0x53) — restore it so the re-pair ACK
  // encrypts under the factory key again.
  data_seq_ = 2;
  beacon_seq_ = 1;
  rf_.set_aes_key(aes_key_);
  rf_.set_frame_counter(0);
  rf_.reset_rx_counter();
  rf_.set_last_nested_rx_counter(0);
  // Persist the cleared state so a reboot before the next pairing doesn't
  // restore the stale session.
  save_pairing_state_();
}

void NartisRfMeterComponent::load_pairing_state_() {
  NvsPairingState s{};
  if (!pref_.load(&s)) {
    ESP_LOGI(TAG, "Pairing restore: NONE — no saved session in NVS, will pair on first cycle");
    return;
  }
  if (s.version != NVS_STATE_VERSION_) {
    ESP_LOGW(TAG, "Pairing restore: FAIL — NVS version %u != expected %u, will re-pair",
             (unsigned) s.version, (unsigned) NVS_STATE_VERSION_);
    return;
  }
  if (!s.paired) {
    ESP_LOGI(TAG, "Pairing restore: NONE — saved session marked unpaired, will pair");
    return;
  }
  // Guard against a key saved for a different meter (config changed): the key's
  // first 12 bytes are the meter serial in ASCII.
  if (meter_serial_.size() >= 12 && memcmp(s.aes_key, meter_serial_.c_str(), 12) != 0) {
    ESP_LOGW(TAG, "Pairing restore: FAIL — saved key is for a different meter serial, will re-pair");
    return;
  }

  rf_.set_aes_key(s.aes_key);
  rf_.set_meter_address(RfAddress::from_bytes(s.meter_addr));
  // Skip the TX counter ahead so any value used in an unsaved in-flight cycle
  // before a crash is never reused (meter rejects rewound/replayed counters).
  rf_.set_frame_counter(s.frame_counter + FRAME_COUNTER_MARGIN_);
  rf_.set_last_rx_counter(s.last_rx_counter);
  rf_.set_last_nested_rx_counter(s.last_nested_rx_counter);
  data_seq_ = s.data_seq;
  beacon_seq_ = s.beacon_seq;
  paired_ = true;
  // Re-prime once after reboot: the freshly-rebooted CIU re-sends the priming
  // get-request-normal in case the meter no longer treats us as primed. Cheap
  // (one frame) and reuses the restored key/counters.
  session_primed_ = false;

  ESP_LOGI(TAG, "Pairing restore: OK — meter=%s, frame_counter=%u (+%u margin), rx=%u, nested_rx=%u",
           format_hex_pretty(s.meter_addr, 8).c_str(),
           (unsigned) s.frame_counter, (unsigned) FRAME_COUNTER_MARGIN_,
           (unsigned) s.last_rx_counter, (unsigned) s.last_nested_rx_counter);
}

void NartisRfMeterComponent::save_pairing_state_() {
  NvsPairingState s{};
  s.version = NVS_STATE_VERSION_;
  rf_.get_aes_key(s.aes_key);
  RfAddress meter = rf_.get_meter_address();
  meter.to_bytes(s.meter_addr);
  s.frame_counter = rf_.get_frame_counter();
  s.last_rx_counter = rf_.get_last_rx_counter();
  s.last_nested_rx_counter = rf_.get_last_nested_rx_counter();
  s.data_seq = data_seq_;
  s.beacon_seq = beacon_seq_;
  s.paired = paired_ ? 1 : 0;
  s._pad = 0;
  if (pref_.save(&s)) {
    ESP_LOGD(TAG, "Session saved to NVS (paired=%u, frame_counter=%u)",
             (unsigned) s.paired, (unsigned) s.frame_counter);
  } else {
    ESP_LOGW(TAG, "Failed to save session to NVS");
  }
}

void NartisRfMeterComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Nartis RF Meter:");
  ESP_LOGCONFIG(TAG, "  Pins: SDIO=%d, SCLK=%d, CSB=%d, FCSB=%d, GPIO3=%d",
                pin_sdio_->get_pin(), pin_sclk_->get_pin(), pin_csb_->get_pin(),
                pin_fcsb_->get_pin(), pin_gpio3_->get_pin());
  ESP_LOGCONFIG(TAG, "  Address: device_id=0x%04X, serial_hash=0x%08X, group=%d, type=%d",
                address_.device_id, (unsigned) address_.serial_hash,
                address_.group_id, address_.device_type);
  ESP_LOGCONFIG(TAG, "  DLMS: client=%d, server=%d", CLIENT_ADDRESS_, SERVER_ADDRESS_);
  if (fix_channel_ >= 0) {
    ESP_LOGCONFIG(TAG, "  Channel: fixed %d (RSSI auto-scan disabled)", fix_channel_);
  } else {
    ESP_LOGCONFIG(TAG, "  Channel: RSSI auto-scan");
  }
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
  // The polling interval is the read cadence. Only start a cycle when idle:
  //   - NOT_INITIALIZED: HAL still in the 2 s setup defer; next tick will catch it.
  //   - any active state: a cycle is still running; skip this tick.
  // start_cycle_() always begins at RSSI scan → channel select; the pair-vs-read
  // decision then happens in handle_channel_select_() based on paired_.
  if (state_ == State::NOT_INITIALIZED) {
    return;
  }
  if (state_ != State::IDLE) {
    ESP_LOGD(TAG, "update(): cycle still running (%s) — skipping this tick",
             LOG_STR_ARG(state_to_str(state_)));
    return;
  }
  start_cycle_();
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

  // Build the INITIAL/bootstrap AES-128-GCM key, used only until pairing:
  //   key[0..11]  = ASCII bytes of the 12-digit meter serial
  //   key[12..15] = placeholder suffix (NARTIS_PAIRING_SALT)
  // The real operational key is meter-assigned: during pairing the meter
  // delivers ASCII(serial) + a fresh per-pairing 4-byte suffix inside the 0x53
  // SESSION_SETUP (GCM-encrypted under the factory key), and that replaces this
  // via set_aes_key() — see the 0x53 handler. The suffix is NOT a fixed
  // constant; BD 02 9B BE was just one observed pair's value.
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
    case NartisRfMeterComponent::State::GET_WAIT_KEEPALIVE: return LOG_STR("GET_WAIT_KEEPALIVE");
    case NartisRfMeterComponent::State::GET_REQ_BEACON_TX: return LOG_STR("GET_REQ_BEACON_TX");
    case NartisRfMeterComponent::State::GET_REQ_BEACON_WAIT_TX_DONE: return LOG_STR("GET_REQ_BEACON_WAIT_TX_DONE");
    case NartisRfMeterComponent::State::GET_WAIT_DATA: return LOG_STR("GET_WAIT_DATA");
    case NartisRfMeterComponent::State::GET_FIN_BEACON_TX: return LOG_STR("GET_FIN_BEACON_TX");
    case NartisRfMeterComponent::State::GET_FIN_BEACON_WAIT_TX_DONE: return LOG_STR("GET_FIN_BEACON_WAIT_TX_DONE");
    case NartisRfMeterComponent::State::GET_WAIT_FINAL_ACK: return LOG_STR("GET_WAIT_FINAL_ACK");
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
        // Fall back to raw frame-type byte on parse failure (CRC/length errors
        // from noise-corrupted frames). For the simple 0x06 presence-ack the
        // routing decision only needs the type byte; we don't read the payload.
        const uint8_t raw_type = (rx_accum_len_ > 1) ? rx_accum_buf_[1] : 0;
        const uint8_t t = (n >= 0) ? static_cast<uint8_t>(rx_type) : raw_type;
        if (t == 0x06 && n >= 0) {
          // Clean frame — safe to capture the meter address.
          ESP_LOGI(TAG, "Pairing: meter answered probe (0x06)");
          lock_channel_();  // meter replied on active_channel_ — pin it for the session
          capture_meter_address_();
          pair_retry_ = 0;  // reset for the ACK phase
          rx_parse_retries_ = 0;
          set_state_(State::PAIR_ACK_TX);
        } else if (t == 0x06 && n < 0 &&
                   rx_parse_retries_ < MAX_RX_PARSE_RETRIES_ &&
                   esphome::millis() - state_entered_ms_ + 300 < rx_timeout_ms_) {
          // CRC fail with matching type byte — the address bytes could be
          // corrupted too. Don't learn a poisoned address; re-arm RX and listen
          // for a clean frame (meter retransmits 0x06 — pair-ours-08 showed 4-5
          // ACKs in a single run).
          rx_parse_retries_++;
          ESP_LOGW(TAG, "Pairing: 0x06 with parse fail (n=%d) — re-arm RX for clean frame "
                        "(try %u/%u)", n, rx_parse_retries_, MAX_RX_PARSE_RETRIES_);
          start_rx_();
        } else if (rx_parse_retries_ < MAX_RX_PARSE_RETRIES_ &&
                   esphome::millis() - state_entered_ms_ + 300 < rx_timeout_ms_) {
          // Noise frame, not the expected 0x06. Re-arm RX and try again within
          // the same wait window — the meter may have retransmitted (it sends
          // an ACK per probe; see iq3/f02 latency 250 ms).
          rx_parse_retries_++;
          ESP_LOGW(TAG, "Pairing: noise frame (type=0x%02X, n=%d) — re-arm RX (try %u/%u)",
                   t, n, rx_parse_retries_, MAX_RX_PARSE_RETRIES_);
          start_rx_();
        } else {
          ESP_LOGW(TAG, "Pairing: unexpected probe response (type=0x%02X, n=%d)", t, n);
          rx_parse_retries_ = 0;
          set_state_(State::PAIR_PROBE_TX);  // retry TX
        }
      } else if (status == RxStatus::ERROR) {
        ESP_LOGW(TAG, "Pairing: no probe response");
        rx_parse_retries_ = 0;
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
        // Determine type from parsed value when available; on parse failure
        // (CRC/length error from a noise-corrupted frame), fall back to raw
        // byte [1]. The transport-layer routing decision only needs the
        // frame-type byte — even a CRC-broken 0x53 should advance us to
        // mode-6 (the meter is already past the session-setup step on its
        // side; sending mode-6 anyway closes the handshake cleanly).
        const uint8_t raw_type = (rx_accum_len_ > 1) ? rx_accum_buf_[1] : 0;
        const uint8_t t = (n >= 0) ? static_cast<uint8_t>(rx_type) : raw_type;
        if (t == 0x53) {
          // SESSION_SETUP — the meter ships the per-pairing data key it assigns
          // this CIU, GCM-encrypted under the factory default key (0x22 x16). 
          // We MUST extract it: the meter encrypts/decrypts all subsequent 
          // traffic with this key, and replies 0x40 forever if we keep using 
          // the bootstrap key. Extract + install below, then acknowledge with 
          // the mode-6 plain reply.
          if (n < 0) {
            ESP_LOGW(TAG, "Pairing: SESSION_SETUP received but parse failed "
                          "(n=%d, raw_type=0x%02X) — proceeding to mode-6 anyway", n, raw_type);
          } else {
            ESP_LOGI(TAG, "Pairing: SESSION_SETUP received (0x53) — proceeding");
            // The 0x53 carries the per-pairing data key the meter assigns this
            // CIU (= ASCII(meter_SN) + meter-chosen 4-byte suffix), GCM-encrypted
            // under the factory default key. Extract it and use it for all data:
            // the meter encrypts/decrypts traffic with THIS key. 
            // Without this the meter can't read our GETs and replies 0x40 forever.
            uint8_t session_key[AES_KEY_SIZE];
            if (n > 0 && rf_.extract_session_key(payload, static_cast<size_t>(n), session_key)) {
              // Sanity: the embedded 12-byte serial must match our configured meter.
              bool sn_ok = (meter_serial_.size() >= 12) &&
                           (memcmp(session_key, meter_serial_.c_str(), 12) == 0);
              if (sn_ok) {
                rf_.set_aes_key(session_key);
                ESP_LOGI(TAG, "Pairing: installed meter-assigned data key (suffix %02X.%02X.%02X.%02X)",
                         session_key[12], session_key[13], session_key[14], session_key[15]);
              } else {
                ESP_LOGW(TAG, "Pairing: 0x53 key serial mismatch — keeping configured key");
              }
            } else {
              ESP_LOGW(TAG, "Pairing: could not extract data key from 0x53 — keeping configured key");
            }
          }

          rf_.set_frame_counter(0);
          rf_.reset_rx_counter();
          rf_.set_last_nested_rx_counter(0);
          rx_parse_retries_ = 0;
          set_state_(State::PAIR_MODE6_TX);
        } else if (t == 0x5B || t == 0x40) {
          // Some meters skip straight to keepalive/ack — treat as paired.
          ESP_LOGI(TAG, "Pairing: meter ready (0x%02X) — paired", t);
          paired_ = true;
          rx_parse_retries_ = 0;
          set_state_(State::BEACON_TX);
        } else if (rx_parse_retries_ < MAX_RX_PARSE_RETRIES_ &&
                   esphome::millis() - state_entered_ms_ + 300 < rx_timeout_ms_) {
          // Noise frame. Re-arm RX and try once more within the same window.
          rx_parse_retries_++;
          ESP_LOGW(TAG, "Pairing: noise frame in ACK-wait (type=0x%02X, n=%d) — "
                        "re-arm RX (try %u/%u)", t, n, rx_parse_retries_, MAX_RX_PARSE_RETRIES_);
          start_rx_();
        } else {
          ESP_LOGW(TAG, "Pairing: unexpected ACK response (type=0x%02X, n=%d)", t, n);
          rx_parse_retries_ = 0;
          set_state_(State::PAIR_ACK_TX);  // retry TX
        }
      } else if (status == RxStatus::ERROR) {
        ESP_LOGW(TAG, "Pairing: no SESSION_SETUP response");
        rx_parse_retries_ = 0;
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
        const uint8_t raw_type = (rx_accum_len_ > 1) ? rx_accum_buf_[1] : 0;
        const uint8_t t = (n >= 0) ? static_cast<uint8_t>(rx_type) : raw_type;
        // Expected: 0x5B keepalive. Accept type-byte match even on CRC fail.
        // (0x40 short-ack also acceptable — some meters skip keepalive after mode-6.)
        if (t == 0x5B || t == 0x40) {
          ESP_LOGI(TAG, "Pairing complete (meter replied 0x%02X). Now paired.", t);
          paired_ = true;
          rx_parse_retries_ = 0;
          set_state_(State::BEACON_TX);
        } else if (rx_parse_retries_ < MAX_RX_PARSE_RETRIES_ &&
                   esphome::millis() - state_entered_ms_ + 300 < rx_timeout_ms_) {
          rx_parse_retries_++;
          ESP_LOGW(TAG, "Pairing keepalive: noise frame (type=0x%02X, n=%d) — re-arm RX (try %u/%u)",
                   t, n, rx_parse_retries_, MAX_RX_PARSE_RETRIES_);
          start_rx_();
        } else {
          ESP_LOGW(TAG, "Pairing: gave up keepalive wait (type=0x%02X) — proceeding anyway", t);
          paired_ = true;
          rx_parse_retries_ = 0;
          set_state_(State::BEACON_TX);
        }
      } else if (status == RxStatus::ERROR) {
        // Keepalive is the meter's "I'm ready" nudge; if we miss it, still
        // consider pairing done and try the beacon poll (it has its own retry).
        ESP_LOGW(TAG, "Pairing: no keepalive — proceeding to beacon anyway");
        paired_ = true;
        rx_parse_retries_ = 0;
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
        uint8_t payload[MAX_DLMS_APDU_SIZE];
        RfFrameType rx_type;
        int payload_len = rf_.parse_frame(rx_accum_buf_, rx_accum_len_, payload, sizeof(payload), &rx_type);
        const uint8_t raw_type = (rx_accum_len_ > 1) ? rx_accum_buf_[1] : 0;
        const uint8_t t = (payload_len >= 0) ? static_cast<uint8_t>(rx_type) : raw_type;

        // Expected: 0x40 short-ack. Accept type-byte match even on CRC fail
        // (noisy RX corrupts the trailing CRC bytes but the type byte is usually
        // intact). On other type bytes, retry within the wait window before
        // giving up.
        if (t == 0x40) {
          lock_channel_();  // first contact in an already-paired session — pin the channel
          if (payload_len < 0) {
            ESP_LOGW(TAG, "Beacon response: 0x40 with parse fail (n=%d) — accepting", payload_len);
          } else {
            ESP_LOGI(TAG, "Beacon response OK (type=0x40, payload=%d bytes)", payload_len);
            if (payload_len > 0) {
              ESP_LOGD(TAG, "Beacon payload: %s", format_hex_pretty(payload, payload_len).c_str());
            }
          }
          rx_parse_retries_ = 0;
          set_state_(State::GET_TX);
        } else if (rx_parse_retries_ < MAX_RX_PARSE_RETRIES_ &&
                   esphome::millis() - state_entered_ms_ + 300 < rx_timeout_ms_) {
          rx_parse_retries_++;
          ESP_LOGW(TAG, "Beacon response: noise frame (type=0x%02X, n=%d) — re-arm RX (try %u/%u)",
                   t, payload_len, rx_parse_retries_, MAX_RX_PARSE_RETRIES_);
          start_rx_();
        } else {
          ESP_LOGW(TAG, "Beacon response: gave up after %u re-arm attempts (last type=0x%02X)",
                   rx_parse_retries_, t);
          rx_parse_retries_ = 0;
          set_state_(State::ERROR_RECOVERY);
        }
      } else if (status == RxStatus::ERROR) {
        ESP_LOGW(TAG, "Beacon RX error/timeout");
        rx_parse_retries_ = 0;
        set_state_(State::ERROR_RECOVERY);
      }
      break;
    }

    case State::GET_TX:
      handle_get_tx_();
      break;

    case State::GET_WAIT_TX_DONE:
      handle_wait_tx_done_(State::GET_WAIT_KEEPALIVE);
      break;

    // Step 1: meter must reply with 0x5B keepalive ("I got your request, hang on").
    // Per dump-spi2/decoded_sequence.txt frame 1 — short 25-byte plain frame.
    case State::GET_WAIT_KEEPALIVE: {
      RxStatus status = poll_rx_();
      if (status == RxStatus::COMPLETE) {
        const uint8_t raw_type = (rx_accum_len_ > 1) ? rx_accum_buf_[1] : 0;
        if (raw_type == 0x5B) {
          ESP_LOGD(TAG, "GET cycle: meter ack'd request (0x5B), requesting data via BEACON");
          retry_count_ = 0;
          set_state_(State::GET_REQ_BEACON_TX);
        } else if (raw_type == 0x43) {
          // Meter answered the GET directly with data (no keepalive/beacon round-trip
          // needed) — matches genuine captures (licon spi4 #10→#12, real01 #1→#4).
          ESP_LOGD(TAG, "GET cycle: meter answered GET directly with data (0x43)");
          retry_count_ = 0;
          if (handle_get_response_()) {
            resp_fail_retries_ = 0;
            set_state_(State::GET_FIN_BEACON_TX);
          } else if (++resp_fail_retries_ >= MAX_RETRIES_) {
            ESP_LOGW(TAG, "GET cycle: response unparseable %ux — skipping batch (start=%u)",
                     resp_fail_retries_, batch_start_idx_);
            resp_fail_retries_ = 0;
            skip_current_batch_();
            advance_after_get_();
          } else {
            set_state_(State::GET_TX);  // retry same batch
          }
        } else {
          ESP_LOGW(TAG, "GET cycle: unexpected response to get-request (type=0x%02X)", raw_type);
          set_state_(State::GET_TX);  // retry whole get-request
        }
      } else if (status == RxStatus::ERROR) {
        ESP_LOGD(TAG, "GET cycle: no reply yet (start=%u, n=%u) — re-sending",
                 batch_start_idx_, batch_count_);
        retry_count_++;
        if (retry_count_ >= GET_REPLY_MAX_RETRIES_) {
          ESP_LOGW(TAG, "GET cycle: max retries — skipping batch (start=%u)", batch_start_idx_);
          if (session_primed_) batch_start_idx_ += batch_count_;  // skip this user batch
          retry_count_ = 0;
          advance_after_get_();
        } else {
          set_state_(State::GET_TX);
        }
      }
      break;
    }

    // Step 2: send BEACON to ask meter to deliver the data.
    // Pace by ~1000 ms after RX of 0x5B — the firmware in dump-spi2 waits
    // ~1.1 s between keepalive and the data-request BEACON. The meter uses
    // this time to query its internal OBIS registers and prep the response.
    // Sending the beacon faster than this makes the meter respond with 0x40
    // (cycle-close ack) instead of 0x43 (data response).
    case State::GET_REQ_BEACON_TX:
      if (esphome::millis() - state_entered_ms_ < GET_REQ_BEACON_DELAY_MS_) {
        break;
      }
      handle_beacon_tx_();
      break;

    case State::GET_REQ_BEACON_WAIT_TX_DONE:
      handle_wait_tx_done_(State::GET_WAIT_DATA);
      break;

    // Step 3: meter sends 0x43 RESPONSE carrying the data (nested-encrypted body).
    case State::GET_WAIT_DATA: {
      RxStatus status = poll_rx_();
      if (status == RxStatus::COMPLETE) {
        const uint8_t raw_type = (rx_accum_len_ > 1) ? rx_accum_buf_[1] : 0;
        if (raw_type == 0x43) {
          if (handle_get_response_()) {
            resp_fail_retries_ = 0;
            set_state_(State::GET_FIN_BEACON_TX);  // closing beacon
          } else if (++resp_fail_retries_ >= MAX_RETRIES_) {
            ESP_LOGW(TAG, "GET cycle: response unparseable %ux — skipping batch (start=%u)",
                     resp_fail_retries_, batch_start_idx_);
            resp_fail_retries_ = 0;
            skip_current_batch_();
            advance_after_get_();
          } else {
            set_state_(State::GET_TX);  // retry same batch
          }
        } else {
          ESP_LOGW(TAG, "GET cycle: unexpected data response (type=0x%02X) — retrying req-beacon",
                   raw_type);
          retry_count_++;
          if (retry_count_ >= MAX_RETRIES_) {
            ESP_LOGW(TAG, "GET cycle: max retries on data response — skipping batch (start=%u)",
                     batch_start_idx_);
            if (session_primed_) batch_start_idx_ += batch_count_;
            retry_count_ = 0;
            advance_after_get_();
          } else {
            set_state_(State::GET_REQ_BEACON_TX);
          }
        }
      } else if (status == RxStatus::ERROR) {
        ESP_LOGD(TAG, "GET cycle: no data response yet (batch start=%u) — re-sending", batch_start_idx_);
        retry_count_++;
        if (retry_count_ >= GET_REPLY_MAX_RETRIES_) {
          if (session_primed_) batch_start_idx_ += batch_count_;
          retry_count_ = 0;
          advance_after_get_();
        } else {
          set_state_(State::GET_REQ_BEACON_TX);
        }
      }
      break;
    }

    // Step 4: send final BEACON acknowledging we got the data.
    // Pace by ~4500 ms after RX of 0x43 — firmware in iq3/f01 burst 4→5 waits
    // ~4200 ms here. Likely protocol pacing for the meter to settle its
    // state-machine before the cycle-close beacon arrives.
    case State::GET_FIN_BEACON_TX:
      if (esphome::millis() - state_entered_ms_ < GET_FIN_BEACON_DELAY_MS_) {
        break;
      }
      handle_beacon_tx_();
      break;

    case State::GET_FIN_BEACON_WAIT_TX_DONE:
      handle_wait_tx_done_(State::GET_WAIT_FINAL_ACK);
      break;

    // Step 5: meter replies with 0x40 short ack — cycle complete.
    case State::GET_WAIT_FINAL_ACK: {
      RxStatus status = poll_rx_();
      if (status == RxStatus::COMPLETE) {
        const uint8_t raw_type = (rx_accum_len_ > 1) ? rx_accum_buf_[1] : 0;
        ESP_LOGD(TAG, "GET cycle: final ack (type=0x%02X) — moving on", raw_type);
        advance_after_get_();
      } else if (status == RxStatus::ERROR) {
        // No final ack — data was already received and parsed; treat as soft success.
        ESP_LOGD(TAG, "GET cycle: no final ack, but data was received — advancing anyway");
        advance_after_get_();
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
  // DISCOVERY (iq3 captures 16/17/18 + real-CIU f01/f02): the advertised channel
  // index in frame[12] bits 7:6 COMMANDS the meter's reply frequency. The meter
  // answers on a per-channel frequency (NARTIS_CUSTOM_CHANNELS) and the CIU
  // listens there; TX always stays on Ch0/433.82 (the meter's wake freq). So the
  // advertised channel and our RX channel MUST be the SAME index — then the meter
  // replies exactly where we listen. We default to ch2 (434.26, proven), and the
  // hop fallback moves advertised+RX together (hop_channel_).
  channel_locked_ = false;
  channel_hop_count_ = 0;
  if (fix_channel_ >= 0) {
    active_channel_ = static_cast<uint8_t>(fix_channel_);
  } else if (active_channel_ >= NUM_CHANNELS) {
    active_channel_ = 2;  // default: ch2 = 434.26 MHz (most reliable reply freq)
  }
  const uint8_t ch = active_channel_;

  // Advertise the channel we listen on, so the meter replies on our RX freq.
  rf_.set_channel_quality(ch, rssi_readings_[ch]);

  // Full bank write (re-locks the PLL). Every custom entry has TX-half=433.82,
  // so this keeps TX on the probe frequency while RX = the channel's reply freq.
  hal_.set_frequency_channel(ch);

  ESP_LOGI(TAG, "Channel: TX 433.82, advertise+RX ch%u (%.3f MHz)%s",
           ch, active_rx_freq_mhz_(),
           (fix_channel_ >= 0) ? " (fix_channel)" : " (default ch2, hop fallback)");

  // First contact requires the pairing handshake; once paired this boot we
  // go straight to the beacon/poll cycle.
  if (paired_) {
    set_state_(State::BEACON_TX);
  } else {
    pair_retry_ = 0;
    set_state_(State::PAIR_PROBE_TX);
  }
}

bool NartisRfMeterComponent::hop_channel_() {
  if (!channel_hopping_enabled_() || channel_locked_) {
    return false;
  }
  active_channel_ = static_cast<uint8_t>((active_channel_ + 1) % NUM_CHANNELS);
  // Move advertised channel AND RX together: advertising channel N makes the
  // meter reply on channel N's frequency, which is where set_frequency_channel
  // tunes RX. Full bank write re-locks the PLL; TX stays on 433.82 (all custom
  // entries share TX-half=433.82).
  hal_.set_frequency_channel(active_channel_);
  rf_.set_channel_quality(active_channel_, rssi_readings_[active_channel_]);
  channel_hop_count_++;
  ESP_LOGI(TAG, "No reply — hopping advertise+RX to channel %u (%.3f MHz), hop %u/%u",
           active_channel_, active_rx_freq_mhz_(), channel_hop_count_, MAX_CHANNEL_HOPS_);
  return true;
}

void NartisRfMeterComponent::lock_channel_() {
  if (!channel_hopping_enabled_() || channel_locked_) {
    return;
  }
  channel_locked_ = true;
  ESP_LOGI(TAG, "Meter replied on channel %u (%.3f MHz) — locking it for this session",
           active_channel_, active_rx_freq_mhz_());
}

float NartisRfMeterComponent::active_rx_freq_mhz_() const {
  // RX-half centre of each channel, mirroring the byte tables in cmt2300a_defs.h.
  static constexpr float kFirmwareRx[NUM_CHANNELS] = {434.10f, 433.58f, 434.54f, 434.98f};
  static constexpr float kCustomRx[NUM_CHANNELS]   = {433.82f, 433.30f, 434.26f, 434.70f};  // meter reply freqs = std TX-halves
  const uint8_t ch = (active_channel_ < NUM_CHANNELS) ? active_channel_ : 0;
  return use_non_standard_channels_ ? kCustomRx[ch] : kFirmwareRx[ch];
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
  // When hopping to acquire the meter's RX channel, allow a larger probe budget
  // (a full sweep of the custom channels + wake-up margin) instead of the plain
  // same-channel retry count.
  const uint8_t probe_budget = channel_hopping_enabled_() ? MAX_CHANNEL_HOPS_ : MAX_PAIR_RETRIES_;
  if (pair_retry_ >= probe_budget) {
    abort_to_idle_("Pairing: no probe answer from meter");
    return;
  }
  // Retry-after-timeout gate (2.3 s) combines with the preceding TX (~200 ms)
  // and RX_TIMEOUT_MS_ (3 s) for a ~5.5 s retry interval, matching firmware.
  // First entry from CHANNEL_SELECT is not gated — the RSSI scan already paced it.
  if (pair_retry_ > 0 &&
      esphome::millis() - state_entered_ms_ < PAIR_RETRY_DELAY_MS_) {
    return;
  }
  // Each unanswered probe hops the RX to the next channel (no-op once locked or
  // when hopping is disabled). TX stays on 433.82 MHz across custom channels.
  if (pair_retry_ > 0) {
    hop_channel_();
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
  // Sequence sourced from data_seq_ (session[0x1A] in firmware terms).
  size_t frame_len = rf_.build_frame(tx_buf_.data(), tx_buf_.size(),
                                     RfFrameType::DATA, data_seq_++,
                                     payload, payload_len);
  if (frame_len == 0 || !transmit_frame_(RfFrameType::DATA, tx_buf_.data(), frame_len)) {
    set_state_(State::ERROR_RECOVERY);
    return;
  }
  set_state_(State::PAIR_PROBE_WAIT_TX_DONE);
}

void NartisRfMeterComponent::handle_pair_ack_tx_() {
  if (pair_retry_ >= MAX_PAIR_RETRIES_) {
    abort_to_idle_("Pairing: no SESSION_SETUP after ACKs");
    return;
  }
  // First attempt (after RX 0x06): real CIU fires 637 ms later — short settle gate.
  // Retries (after RX timeout): same 2.3 s gate as probe → ~5.5 s retry interval.
  uint32_t gate = (pair_retry_ == 0) ? PAIR_POST_PROBE_DELAY_MS_ : PAIR_RETRY_DELAY_MS_;
  if (esphome::millis() - state_entered_ms_ < gate) {
    return;
  }
  pair_retry_++;
  ESP_LOGD(TAG, "Pairing: sending encrypted ACK %d/%d...", pair_retry_, MAX_PAIR_RETRIES_);
  uint8_t ack_payload[29];
  size_t payload_len = build_beacon_payload_(ack_payload, sizeof(ack_payload));
  if (payload_len == 0) {
    set_state_(State::ERROR_RECOVERY);
    return;
  }
  // Encrypted mode-2 ACK frame (flags 0x44). Continues data_seq_ from probe.
  size_t frame_len = rf_.build_frame(tx_buf_.data(), tx_buf_.size(),
                                     RfFrameType::ACK, data_seq_++,
                                     ack_payload, payload_len);
  if (frame_len == 0 || !transmit_frame_(RfFrameType::ACK, tx_buf_.data(), frame_len)) {
    set_state_(State::ERROR_RECOVERY);
    return;
  }
  set_state_(State::PAIR_ACK_WAIT_TX_DONE);
}

void NartisRfMeterComponent::handle_pair_mode6_tx_() {
  // Pace by ~5.5 s after the meter's 0x53 SESSION_SETUP — the real CIU waits
  // 5629 ms before sending mode-6 (meter is doing key-install internally).
  if (esphome::millis() - state_entered_ms_ < PAIR_POST_SESSION_DELAY_MS_) {
    return;
  }
  ESP_LOGI(TAG, "Pairing: sending mode-6 confirmation...");
  // Plain mode-6 frame (flags 0x00, marker 0x8A). Payload = '0' + 12-digit
  // meter serial (13 B), same as the 0x46 probe — verified in f02 air capture.
  uint8_t payload[16];
  size_t payload_len = build_pair_probe_payload_(payload, sizeof(payload));
  if (payload_len == 0) {
    set_state_(State::ERROR_RECOVERY);
    return;
  }
  // Mode-6 frame (flags 0x00, marker 0x8A). ★ Sequence sourced from the
  // BEACON counter (session[0x15] in firmware), NOT the data counter. Real
  // CIU sends seq=01 here even though the data counter is at 5 (iq3/f02
  // burst 7 vs bursts 1-5). Using the wrong counter = meter rejects the
  // handshake-confirm and stays in SESSION_SETUP retransmit loop.
  size_t frame_len = rf_.build_frame(
      tx_buf_.data(), tx_buf_.size(), RfFrameType::PLAIN_DATA, beacon_seq_++,
      payload, payload_len);
  if (frame_len == 0 || !transmit_frame_(RfFrameType::PLAIN_DATA, tx_buf_.data(), frame_len)) {
    set_state_(State::ERROR_RECOVERY);
    return;
  }
  set_state_(State::PAIR_MODE6_WAIT_TX_DONE);
}

void NartisRfMeterComponent::handle_beacon_tx_() {
  ESP_LOGD(TAG, "Sending beacon (frame_counter=%u, beacon_seq=%d)...",
           (unsigned) rf_.get_frame_counter(), beacon_seq_);

  // Build 29-byte beacon payload matching firmware beacon_address_builder (0xB5B0)
  uint8_t beacon_payload[29];
  size_t payload_len = build_beacon_payload_(beacon_payload, sizeof(beacon_payload));
  if (payload_len == 0) {
    ESP_LOGE(TAG, "Failed to build beacon payload");
    set_state_(State::ERROR_RECOVERY);
    return;
  }

  ESP_LOGV(TAG, "Beacon payload (%d bytes): %s", (int) payload_len,
           format_hex_pretty(beacon_payload, payload_len).c_str());

  // Mode-3 BEACON shares the beacon_seq_ counter with mode-6 MODE-6.
  size_t frame_len = rf_.build_frame(tx_buf_.data(), tx_buf_.size(),
                                     RfFrameType::BEACON, 0x00, beacon_seq_++,
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

  // Pick the right "wait TX done" state based on which beacon we're sending:
  //   BEACON_TX            — initial post-pairing wake-up beacon (→ wait 0x40)
  //   GET_REQ_BEACON_TX    — in-cycle beacon requesting data (→ wait 0x43)
  //   GET_FIN_BEACON_TX    — in-cycle beacon closing the read (→ wait 0x40)
  switch (state_) {
    case State::GET_REQ_BEACON_TX:
      set_state_(State::GET_REQ_BEACON_WAIT_TX_DONE);
      break;
    case State::GET_FIN_BEACON_TX:
      set_state_(State::GET_FIN_BEACON_WAIT_TX_DONE);
      break;
    case State::BEACON_TX:
    default:
      set_state_(State::BEACON_WAIT_TX_DONE);
      break;
  }
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
  // If the session is primed AND we've walked past every user sensor, we're
  // done — publish and return to IDLE.
  if (session_primed_ && batch_start_idx_ >= sensors_.size()) {
    set_state_(State::PUBLISH);
    return;
  }

  // ---- Build the current batch ----
  // Two phases:
  //   Phase 0 (!session_primed_): get-request-NORMAL (c0 01, single OBIS).
  //     The freshly-paired meter requires this opener before it engages the
  //     with-list flow — sending a with-list first leaves it silent (matches
  //     real CIU spi4 frame #10). Drives the same keepalive→beacon→data cycle,
  //     so it reuses the GET states below.
  //   Phase 1: user-defined sensors in batches of user_batch_size_ (YAML
  //     `batch_size`). No vendor-init read — we only request configured sensors.
  uint8_t apdu[4 + 10 * DlmsClient::MAX_LIST_ATTRS];
  size_t apdu_len;

  if (!session_primed_) {
    static constexpr ObisCode kVendorObis{{0x00, 0x00, 0x60, 0x80, 0x03, 0xFF}};
    apdu_len = dlms_.build_get_request_normal(apdu, sizeof(apdu), kVendorObis, 1, 2);
    batch_count_ = 0;  // priming read — store nothing
    ESP_LOGI(TAG, "GET priming: get-request-normal (c0 01, OBIS 0-0:96.128.3.255) "
                  "— meter requires this before the with-list flow");
  } else {
    // User-defined sensors, up to user_batch_size_ per batch.
    DlmsClient::AttrSpec specs[DlmsClient::MAX_LIST_ATTRS];
    uint8_t spec_count = 0;
    for (uint8_t i = 0; i < user_batch_size_ && batch_start_idx_ + i < sensors_.size(); i++) {
      const auto &s = sensors_[batch_start_idx_ + i];
      specs[i] = {s.class_id, s.obis, s.attr_id};
      spec_count++;
    }
    if (spec_count == 0) {
      // No more sensors — publish and finish the cycle.
      set_state_(State::PUBLISH);
      return;
    }
    ESP_LOGD(TAG, "GET batch: user sensors [%u..%u] (%u attrs)",
             batch_start_idx_, batch_start_idx_ + spec_count - 1, spec_count);
    for (uint8_t i = 0; i < spec_count; i++) {
      ESP_LOGD(TAG, "  [%u] OBIS %u.%u.%u.%u.%u.%u class=%u attr=%u",
               i,
               specs[i].obis.bytes[0], specs[i].obis.bytes[1], specs[i].obis.bytes[2],
               specs[i].obis.bytes[3], specs[i].obis.bytes[4], specs[i].obis.bytes[5],
               specs[i].class_id, specs[i].attr_id);
    }
    batch_count_ = spec_count;
    apdu_len = dlms_.build_get_request_with_list(apdu, sizeof(apdu), specs, spec_count);
  }

  if (apdu_len == 0) {
    ESP_LOGE(TAG, "Failed to build read request");
    advance_after_get_();
    return;
  }

  // Wrap in IEC envelope + AES-GCM encrypt + transmit as mode-2 (0x44).
  if (!tx_dlms_apdu_(apdu, apdu_len)) {
    set_state_(State::ERROR_RECOVERY);
    return;
  }

  set_state_(State::GET_WAIT_TX_DONE);
}

bool NartisRfMeterComponent::tx_dlms_apdu_(const uint8_t *apdu, size_t apdu_len) {
  // Wrap a DLMS APDU in the IEC 62056-47 transport envelope and send it as an
  // ENCRYPTED mode-2 (0x44) frame. Format observed in spi2 frame #0 / spi4 #10:
  //   [0..1] version  = 00 01
  //   [2..3] src wPort = 00 66 (CIU client = 102)
  //   [4..5] dst wPort = 00 01 (meter server = 1)
  //   [6..7] length   = APDU length BE
  //   [8..]  DLMS APDU
  // The whole 8+APDU plaintext gets AES-GCM-encrypted inside the 0x44 frame.
  // Sending it plain (mode-1 / 0x46) makes the meter silently drop it —
  // post-pairing the meter only accepts encrypted reads. With up to 10 attrs
  // the APDU reaches 4 + 10*10 = 104 bytes, so 128 is safe.
  uint8_t plain[128];
  if (apdu_len + 8 > sizeof(plain)) {
    ESP_LOGE(TAG, "APDU too large for IEC wrapper buffer");
    return false;
  }
  plain[0] = 0x00; plain[1] = 0x01;             // version
  plain[2] = 0x00; plain[3] = 0x66;             // src wPort = 102
  plain[4] = 0x00; plain[5] = 0x01;             // dst wPort = 1
  plain[6] = static_cast<uint8_t>((apdu_len >> 8) & 0xFF);
  plain[7] = static_cast<uint8_t>(apdu_len & 0xFF);
  memcpy(plain + 8, apdu, apdu_len);
  const size_t plain_len = 8 + apdu_len;

  ESP_LOGV(TAG, "DLMS TX plaintext (%d B = IEC[8] + DLMS[%d]): %s",
           (int) plain_len, (int) apdu_len,
           format_hex_pretty(plain, plain_len).c_str());

  // Mode-2 encrypted ACK frame — uses data_seq_ (same counter as probe/ACK).
  size_t frame_len = rf_.build_frame(tx_buf_.data(), tx_buf_.size(),
                                     RfFrameType::ACK, data_seq_++,
                                     plain, plain_len);
  if (frame_len == 0) {
    return false;
  }
  return transmit_frame_(RfFrameType::ACK, tx_buf_.data(), frame_len);
}

bool NartisRfMeterComponent::handle_get_response_() {
  // Caller (GET_WAIT_DATA case) owns the state transition AFTER this returns.
  // We parse the frame, then walk the multi-result get-response-with-list to
  // pull one DlmsValue per attribute in the batch we just sent. Vendor-init
  // results are parsed-and-discarded; user-batch results are stored into
  // sensors_[batch_start_idx_ + i].last_value.
  ESP_LOGD(TAG, "GET response received (%d bytes, batch_count=%u)",
           (int) rx_accum_len_, batch_count_);

  uint8_t payload[MAX_DLMS_APDU_SIZE];
  RfFrameType rx_type;
  int payload_len = rf_.parse_frame(rx_accum_buf_, rx_accum_len_, payload, sizeof(payload), &rx_type);
  if (payload_len < 0) {
    ESP_LOGW(TAG, "Failed to parse response frame (type=0x%02X, err=%d)",
             rx_accum_len_ > 1 ? rx_accum_buf_[1] : 0, payload_len);
    return false;
  }
  ESP_LOGVV(TAG, "GET payload (%d bytes, type=0x%02X): %s",
           payload_len, static_cast<uint8_t>(rx_type),
           format_hex_pretty(payload, payload_len).c_str());

  // For a 0x43 the parse_frame payload is still the nested-encrypted envelope
  // (addr-echo + 01 29 len 00 + counter + ciphertext + tag). Decrypt the inner
  // DLMS APDU first (non-mutating peek — same decrypt as the RX DLMS log line).
  const uint8_t *dlms_ptr = payload;
  size_t dlms_len = static_cast<size_t>(payload_len);
  uint8_t dlms_buf[MAX_DLMS_APDU_SIZE];
  const uint8_t raw_type = (rx_accum_len_ > 1) ? rx_accum_buf_[1] : 0;
  if (raw_type == 0x43) {
    int n = rf_.peek_nested_plain(payload, static_cast<size_t>(payload_len),
                                  dlms_buf, sizeof(dlms_buf));
    if (n <= 0) {
      ESP_LOGW(TAG, "0x43 nested decrypt failed (err=%d)", n);
      return false;
    }
    dlms_ptr = dlms_buf;
    dlms_len = static_cast<size_t>(n);
    ESP_LOGVV(TAG, "GET DLMS (%u B): %s", (unsigned) dlms_len,
             format_hex_pretty(dlms_buf, dlms_len).c_str());
  }

  DlmsValue values[DlmsClient::MAX_LIST_ATTRS];
  for (auto &v : values) v.valid = false;

  // Per-attribute COSEM class hints (in batch order) so the parser can render a
  // class-8 (Clock) octet-string as a date-time. Only meaningful for user reads;
  // the priming read is discarded, so it passes no hints.
  uint16_t class_ids[DlmsClient::MAX_LIST_ATTRS] = {0};
  if (session_primed_) {
    for (uint8_t i = 0; i < batch_count_ && i < DlmsClient::MAX_LIST_ATTRS &&
                        (batch_start_idx_ + i) < sensors_.size(); i++) {
      class_ids[i] = sensors_[batch_start_idx_ + i].class_id;
    }
  }

  uint8_t got_count = 0;
  bool ok = dlms_.parse_read_response_list(dlms_ptr, dlms_len, values,
                                           DlmsClient::MAX_LIST_ATTRS, &got_count,
                                           session_primed_ ? class_ids : nullptr);
  if (!ok) {
    ESP_LOGW(TAG, "parse_read_response_list rejected the payload");
    return false;
  }

  if (!session_primed_) {
    // Priming normal-get response — discard, don't touch batch state.
    ESP_LOGI(TAG, "Priming response: parsed %u result slots — discarding", got_count);
  } else {
    // Guard: a real with-list answer returns exactly one result per requested
    // attribute (access-errors included as invalid slots). A count mismatch
    // means this frame isn't the answer to the batch we just sent (e.g. a
    // stale/duplicate response) — reject so the caller re-sends rather than
    // storing values into the wrong sensor slots.
    if (got_count != batch_count_) {
      ESP_LOGW(TAG, "GET response: count mismatch (got %u, expected %u) — "
                    "ignoring mismatched/stale response", got_count, batch_count_);
      return false;
    }
    // Store each result into the corresponding user-sensor slot.
    for (uint8_t i = 0; i < got_count && (batch_start_idx_ + i) < sensors_.size(); i++) {
      if (values[i].has_value()) {
        sensors_[batch_start_idx_ + i].last_value = values[i];
        ESP_LOGI(TAG, "  sensor[%u] value OK (dlms_type=0x%02X, %u B)",
                 batch_start_idx_ + i, values[i].dtype, values[i].raw_len);
      } else {
        ESP_LOGW(TAG, "  sensor[%u] no value (compound type or parse-fail)", batch_start_idx_ + i);
      }
    }
    batch_start_idx_ += batch_count_;
  }
  retry_count_ = 0;
  return true;
}

void NartisRfMeterComponent::skip_current_batch_() {
  // Force progress past a batch whose response we couldn't parse, so the GET
  // cycle always terminates instead of re-querying the same batch forever.
  // (session-priming progression is handled by advance_after_get_ itself.)
  if (session_primed_) {
    batch_start_idx_ += batch_count_;  // skip this user batch
  }
}

void NartisRfMeterComponent::handle_ack_tx_() {
  ESP_LOGD(TAG, "Sending ACK for GET response...");

  // ACK frame: Mode 2 (header 0x44), RF AES-GCM encrypted, empty payload.
  // Mode-2 uses data_seq_ (continues the mode-1/2 counter).
  size_t frame_len = rf_.build_frame(tx_buf_.data(), tx_buf_.size(),
                                     RfFrameType::ACK, 0x00, data_seq_++,
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
  // Priming complete: the meter has now engaged (we ran one full normal-get
  // cycle). Mark the session primed and continue to the first with-list read.
  if (!session_primed_) {
    session_primed_ = true;
    ESP_LOGI(TAG, "Session primed — proceeding to with-list reads");
    // The session is now proven working end-to-end (handshake + priming read).
    // Persist it so a reboot resumes without re-pairing. (session_primed_ itself
    // is not stored — a restored session always re-primes once.)
    save_pairing_state_();
    set_state_(State::GET_TX);
    return;
  }
  // Batch-driven progression: batch_start_idx_ has been bumped by
  // handle_get_response_; keep cycling GET_TX until all user sensors are read.
  if (batch_start_idx_ >= sensors_.size()) {
    set_state_(State::PUBLISH);
  } else {
    set_state_(State::GET_TX);
  }
}

void NartisRfMeterComponent::handle_publish_() {
  ESP_LOGI(TAG, "Publishing sensor values...");

  for (auto &entry : sensors_) {
    if (!entry.last_value.has_value()) continue;

    const auto dtype = static_cast<DlmsDataType>(entry.last_value.dtype);
    const uint8_t *raw = entry.last_value.raw;
    const size_t rlen = entry.last_value.raw_len;

    // Numeric sensor: one shared converter handles every scalar DLMS type.
    if (entry.sensor) {
      entry.sensor->publish_state(DlmsClient::data_as_float(dtype, raw, rlen));
    }

    // Text sensor: shared converter renders strings/date-time/numerics. The
    // user picks the interpretation by setting obis_class (e.g. 8 = Clock).
    if (entry.text_sensor) {
      char buf[64];
      DlmsClient::data_to_string(dtype, raw, rlen, buf, sizeof(buf));
      entry.text_sensor->publish_state(buf);
    }
  }

  // Reaching PUBLISH is a healthy cycle — clear the re-pair failure streak.
  consecutive_read_failures_ = 0;

  // Persist the advanced TX/RX counters (and confirm the paired session) so the
  // next boot resumes exactly where we left off.
  save_pairing_state_();

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

  // Soft recovery: sleep the radio, account the failure, and return to IDLE so
  // the next update() retries (re-pairing after MAX_READ_FAILURES_BEFORE_REPAIR_
  // consecutive read failures).
  abort_to_idle_("Error recovery");
}

/* ================================================================
 * TX/RX Helpers
 * ================================================================ */

bool NartisRfMeterComponent::transmit_frame_(RfFrameType type,
                                             const uint8_t *frame, size_t len) {
  ESP_LOGD(TAG, "TX frame (%d bytes, type=0x%02X):", (int) len, static_cast<uint8_t>(type));
  ESP_LOGV(TAG, "TX: %s", format_hex_pretty(frame, len).c_str());
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
  // Explicit full CIU address wins (8 bytes as 16 hex chars). Use this to
  // impersonate the exact CIU a meter is already paired to — a meter typically
  // only answers its paired CIU's address, so a derived/MAC address is ignored.
  if (ciu_address_.size() == 16) {
    uint8_t b[8];
    bool ok = true;
    for (size_t i = 0; i < 8 && ok; i++) {
      auto hexval = [&](char c, bool &good) -> uint8_t {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        good = false;
        return 0;
      };
      b[i] = (hexval(ciu_address_[2 * i], ok) << 4) | hexval(ciu_address_[2 * i + 1], ok);
    }
    if (ok) {
      address_ = RfAddress::from_bytes(b);
      uint8_t out[8];
      address_.to_bytes(out);
      ESP_LOGI(TAG, "Using explicit CIU address: %s", format_hex_pretty(out, 8).c_str());
      return;
    }
    ESP_LOGW(TAG, "Invalid ciu_address '%s' — falling back to derivation", ciu_address_.c_str());
  }

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
  // timestamp patched in. Reconstructed byte-for-byte from the firmware's
  // beacon_address_builder (0xB5B0), which is authoritative for OUR target:
  // flash.bin is the mitgo CIU we emulate, and the meter in iq3/* is mitgo's.
  // The builder assembles this from the flash TLV template at 0xCA1C, the
  // "1234" constant at 0xB620, and two hard-coded bytes (0x07,0x04) — NOT from
  // the RF address (the earlier implementation's mistake).
  //
  //   [0..3]   0d fd 0d 04        TLV: tag 0x0dfd, type 0x0d, len 4   (flash 0xCA20)
  //   [4..7]   "1234"             4-byte payload (the static pairing PIN, 0xB620)
  //   [8..11]  0d fd 0f 02        TLV: tag 0x0dfd, type 0x0f, len 2   (flash 0xCA24)
  //   [12..13] 07 04              that TLV's 2-byte value (fw immediates *(buf+0xc)=7,(+0xd)=4)
  //   [14..15] 06 6d              i32 high half (flash 0xCA1C, constant 0x066d)
  //   [16..21] RTC timestamp      6-byte bit-packed clock (varies per beacon)
  //   [22..24] 04 fd 17           TLV: tag 0x04fd, len/type 0x17      (flash 0xCA28)
  //   [25..28] 00 00 00 00        subscription bitmask u32 (RAM b73c+0x24; 0 = no subs)
  //
  // NOTE: [12..13] was 02 05 in the original template, which came from a
  // dump-spi2 beacon belonging to the LICON CIU/meter pair — a different pair.
  // The mitgo firmware emits 07 04 here, now confirmed correct: with these
  // bytes pairing completes and the meter delivers data end-to-end.
  static constexpr uint8_t BEACON_HEAD[16] = {
      0x0D, 0xFD, 0x0D, 0x04, 0x31, 0x32, 0x33, 0x34,
      0x0D, 0xFD, 0x0F, 0x02, 0x07, 0x04, 0x06, 0x6D};
  static constexpr uint8_t BEACON_TAIL[7] = {0x04, 0xFD, 0x17, 0x00, 0x00, 0x00, 0x00};

  memcpy(out, BEACON_HEAD, sizeof(BEACON_HEAD));
  build_rtc_timestamp_(out + 16);          // [16..21]
  memcpy(out + 22, BEACON_TAIL, sizeof(BEACON_TAIL));

  return 29;
}

void NartisRfMeterComponent::build_rtc_timestamp_(uint8_t *out) {
  // The meter only requires a LIVE, ADVANCING clock here — not the real
  // wall-clock. Confirmed against real-CIU beacons (dump-spi2): the field ticks
  // forward each beacon; a frozen/zero value makes the meter keepalive the
  // request but refuse the data (0x40, never 0x43).
  //
  // So we don't depend on NTP/RTC: if a real time source happens to be present
  // we use it, otherwise we synthesize an advancing clock from millis() on top
  // of a fixed base epoch. Either way the timestamp keeps moving, which is all
  // the meter checks.
  static constexpr uint32_t SYNTHETIC_BASE_EPOCH = 1704067200;  // 2024-01-01 00:00:00 UTC
  time_t epoch = ::time(nullptr);
  auto now = esphome::ESPTime::from_epoch_utc(epoch);
  if (!now.is_valid() || now.year < 2024) {
    epoch = static_cast<time_t>(SYNTHETIC_BASE_EPOCH + esphome::millis() / 1000U);
    now = esphome::ESPTime::from_epoch_utc(epoch);
  }

  uint8_t yr  = now.year % 100;
  uint8_t mo  = now.month;
  uint8_t day = now.day_of_month;
  uint8_t hr  = now.hour;
  uint8_t mn  = now.minute;
  uint8_t sec = now.second;
  // Firmware day-of-week is ISO (Mon=1 … Sun=7). ESPHome day_of_week is 1=Sun … 7=Sat.
  uint8_t dow = (now.day_of_week == 1) ? 7 : static_cast<uint8_t>(now.day_of_week - 1);

  // 6-byte bit-packing — verified byte-for-byte against real-CIU beacons and
  // firmware rtc_timestamp_pack (@0x102FA). Beacon payload offset [16..21]:
  //   [0] seconds (6b)
  //   [1] minutes (6b)
  //   [2] hour(5b)  | dow(3b)<<5
  //   [3] day(5b)   | (year & 0x07)<<5
  //   [4] month(4b) | ((year >> 3) & 0x0F)<<4
  //   [5] 0
  out[0] = sec & 0x3F;
  out[1] = mn & 0x3F;
  out[2] = (hr & 0x1F) | ((dow & 0x07) << 5);
  out[3] = (day & 0x1F) | ((yr & 0x07) << 5);
  out[4] = (mo & 0x0F) | (((yr >> 3) & 0x0F) << 4);
  out[5] = 0x00;
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
  hal_.set_int_source(INT_SEL_RX_FIFO_TH);
  // Restore PAYLOAD_LENG to the large ceiling: the preceding TX shrank it to the
  // TX frame size, which would otherwise cap this RX at (TX_len + 1) bytes and
  // truncate the meter's longer reply (e.g. 45/66, 75/93).
  hal_.set_rx_payload_length();
  rx_tail_wait_ms_ = 0;
  rx_drain_stall_ms_ = 0;

  // Enter RX mode: STBY → RFS → RX
  hal_.write_reg(REG_MODE_CTL, GO_RFS);
  hal_.wait_for_state(STA_RFS);
  hal_.write_reg(REG_MODE_CTL, GO_RX);
  hal_.wait_for_state(STA_RX);

  ESP_LOGD(TAG, "RX started on ch%u (%.3f MHz%s)",
           active_channel_, active_rx_freq_mhz_(), channel_locked_ ? ", locked" : "");
}

NartisRfMeterComponent::RxStatus NartisRfMeterComponent::poll_rx_() {
  if (!rx_active_) return RxStatus::ERROR;

  uint32_t elapsed = esphome::millis() - state_entered_ms_;

  // Drain whatever the chip has in its FIFO right now (SPI poll, once per loop).
  size_t drained = hal_.poll_rx_drain(rx_accum_buf_ + rx_accum_len_,
                                      MAX_RF_FRAME_SIZE - rx_accum_len_);
  if (drained > 0) {
    const bool first_bytes = (rx_accum_len_ == 0);
    rx_accum_len_ += drained;
    rx_drain_stall_ms_ = esphome::millis();  // chunk-drain made progress
    if (first_bytes) {
      ESP_LOGV(TAG, "RX first byte +%u ms (meter reply latency)",
               (unsigned) (esphome::millis() - state_entered_ms_));
    }
    ESP_LOGVV(TAG, "RX drained +%d bytes (total %d)", (int) drained, (int) rx_accum_len_);
  } else if (rx_drain_stall_ms_ == 0 && rx_accum_len_ > 0) {
    // First bytes already in (via an earlier loop) but this poll drained none:
    // arm the stall window from now so the timeout below has a reference.
    rx_drain_stall_ms_ = esphome::millis();
  }

  // Extract expected length from first byte once we have it.
  if (rx_accum_len_ >= 1 && rx_expected_len_ == 0) {
    rx_expected_len_ = static_cast<size_t>(rx_accum_buf_[0]) + 1;
    if (rx_expected_len_ > MAX_RF_FRAME_SIZE) {
      ESP_LOGW(TAG, "RX frame too large: %d bytes", (int) rx_expected_len_);
      finish_rx_();
      return RxStatus::ERROR;
    }
    ESP_LOGVV(TAG, "RX expecting %d bytes total", (int) rx_expected_len_);
  }

  // Tail read: the final < FIFO_TH_VALUE bytes never raise RX_FIFO_TH, so the
  // chunk-drain can't fetch them. Once only the sub-threshold tail remains,
  // read it directly by length after a brief wait so those bytes have arrived.
  if (rx_expected_len_ > 0 && rx_accum_len_ < rx_expected_len_) {
    size_t remaining = rx_expected_len_ - rx_accum_len_;
    if (remaining < FIFO_TH_VALUE) {
      if (rx_tail_wait_ms_ == 0) {
        rx_tail_wait_ms_ = esphome::millis();
      } else if (esphome::millis() - rx_tail_wait_ms_ >= RX_TAIL_WAIT_MS_) {
        hal_.read_fifo(rx_accum_buf_ + rx_accum_len_, remaining);
        rx_accum_len_ += remaining;
      }
    } else if (drained == 0 && rx_drain_stall_ms_ != 0 &&
               esphome::millis() - rx_drain_stall_ms_ >= RX_DRAIN_STALL_MS_) {
      // A full chunk (>= FIFO_TH_VALUE) is still owed but the chunk-drain has
      // found nothing for RX_DRAIN_STALL_MS_: the chip finished receiving and
      // de-asserted RX_FIFO_TH with the remainder still unread (the 45/66,
      // 75/93 stall). Those bytes are sitting in the 64 B FIFO — read them
      // directly by length. CRC downstream rejects it if sync was truly lost.
      ESP_LOGD(TAG, "RX drain stalled at %d/%d — reading %d remaining from FIFO",
               (int) rx_accum_len_, (int) rx_expected_len_, (int) remaining);
      hal_.read_fifo(rx_accum_buf_ + rx_accum_len_, remaining);
      rx_accum_len_ += remaining;
    }
  }

  // Check if we have the complete packet
  if (rx_expected_len_ > 0 && rx_accum_len_ >= rx_expected_len_) {
    // Trim to expected length (last chunk may have extra bytes)
    rx_accum_len_ = rx_expected_len_;
    ESP_LOGVV(TAG, "RX complete: %d bytes", (int) rx_accum_len_);
    ESP_LOGV(TAG, "RX: %s", format_hex_pretty(rx_accum_buf_, rx_accum_len_).c_str());

    // Peek-decode for the log: if the frame contains an encrypted body, run
    // the parser non-destructively and dump the decrypted plaintext. This is
    // diagnostic only — the calling state handler re-parses separately. We
    // accept any parse_frame result here (CRC fail, decrypt fail, etc.) and
    // just print what came out.
    if (rx_accum_len_ > 1) {
      uint8_t peek_buf[MAX_DLMS_APDU_SIZE];
      RfFrameType peek_type;
      int peek_len = rf_.parse_frame(rx_accum_buf_, rx_accum_len_,
                                     peek_buf, sizeof(peek_buf), &peek_type);
      const uint8_t raw_type = rx_accum_buf_[1];
      if (peek_len > 0) {
        ESP_LOGV(TAG, "RX decoded (type=0x%02X, %d B plain%s): %s",
                 raw_type, peek_len,
                 (raw_type == 0x43 || raw_type == 0x53) ? " — nested-decrypted" : "",
                 format_hex_pretty(peek_buf, peek_len).c_str());
        // Third line: for 0x43 the outer payload still wraps a nested-encrypted
        // DLMS APDU — decrypt it (non-mutating peek) and dump the inner plaintext.
        if (raw_type == 0x43) {
          uint8_t dlms_buf[MAX_DLMS_APDU_SIZE];
          int dlms_len = rf_.peek_nested_plain(peek_buf, peek_len, dlms_buf, sizeof(dlms_buf));
          if (dlms_len > 0) {
            ESP_LOGV(TAG, "RX DLMS (%d B): %s", dlms_len,
                     format_hex_pretty(dlms_buf, dlms_len).c_str());
          } else {
            ESP_LOGW(TAG, "RX DLMS: nested decrypt failed (err=%d)", dlms_len);
          }
        }
      } else if (peek_len < 0) {
        ESP_LOGW(TAG, "RX decode failed (type=0x%02X, err=%d)", raw_type, peek_len);
      }
    }

    finish_rx_();
    return RxStatus::COMPLETE;
  }

  // Check for buffer overflow
  if (rx_accum_len_ >= MAX_RF_FRAME_SIZE) {
    ESP_LOGW(TAG, "RX buffer overflow (%d bytes)", (int) rx_accum_len_);
    finish_rx_();
    return RxStatus::ERROR;
  }

  // Check timeout. In the steady-state GET reply waits, use a short timeout so a
  // request dropped in the meter's post-TX deaf window re-sends fast instead of
  // stalling 3 s — but only until the first byte lands; once a reply has started
  // the full RX_TIMEOUT_MS_ governs completion so a large frame isn't aborted.
  uint32_t reply_timeout = rx_timeout_ms_;
  if (rx_accum_len_ == 0 &&
      (state_ == State::GET_WAIT_KEEPALIVE || state_ == State::GET_WAIT_DATA ||
       state_ == State::GET_WAIT_FINAL_ACK)) {
    reply_timeout = rx_reply_timeout_ms_;
  }
  if (elapsed > reply_timeout) {
    if (rx_accum_len_ > 0) {
      ESP_LOGW(TAG, "RX timeout (%d/%d bytes received in %dms)",
               (int) rx_accum_len_, (int) rx_expected_len_, (int) elapsed);
      ESP_LOGD(TAG, "RX partial: %s", format_hex_pretty(rx_accum_buf_, rx_accum_len_).c_str());
    } else {
      ESP_LOGW(TAG, "RX timeout (0/%d bytes received in %dms)",
               (int) rx_expected_len_, (int) elapsed);
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
