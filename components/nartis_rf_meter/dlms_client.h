/*
 * Nartis DLMS Client — Proprietary Protocol Layer
 *
 * Layer 3: Builds and parses Nartis proprietary DLMS APDUs.
 * Knows protocol but NOT RF details.
 * Produces/consumes raw byte buffers that Layer 2 (RF) transports.
 *
 * Key finding: Nartis CIU firmware does NOT use standard DLMS AARQ/AARE
 * association. Instead, each register read is a self-contained proprietary
 * request with format: C0 01 C1 00 [class_id] [obis] [attr_id] [auth_flag]
 * (firmware nartis_build_read (0x8CAC)). No separate association or release phase.
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include "helpers.h"

namespace esphome::nartis_rf_meter {

/// DLMS client session state
enum class DlmsState : uint8_t {
  IDLE,                  // Not connected
  REQUEST_SENT,          // Read request sent, waiting for response
};

/// DLMS data types (subset used by Nartis meters)
enum class DlmsDataType : uint8_t {
  NULL_DATA = 0x00,
  BOOLEAN = 0x03,
  INT8 = 0x0F,
  INT16 = 0x10,
  INT32 = 0x05,
  INT64 = 0x14,
  UINT8 = 0x11,
  UINT16 = 0x12,
  UINT32 = 0x06,
  UINT64 = 0x15,
  FLOAT32 = 0x17,
  FLOAT64 = 0x18,
  OCTET_STRING = 0x09,
  VISIBLE_STRING = 0x0A,
  UTF8_STRING = 0x0C,
  ENUM = 0x16,
  STRUCTURE = 0x02,
  ARRAY = 0x01,
  DATE_TIME = 0x19,
};

class DlmsClient {
 public:
  DlmsClient() = default;

  /// Configure DLMS credentials
  void set_credentials(const char *password, uint16_t client_addr, uint16_t server_addr);

  /* ---- APDU Builders (TX) ---- */

  /// Build proprietary read request matching firmware nartis_build_read (0x8cac).
  /// Format: C0 01 C1 00 [class_id:1B] [obis:6B] [attr_id:1B] 00
  /// No DLMS-level auth — security is RF-layer AES-CCM only.
  size_t build_read_request(uint8_t *out, size_t max,
                            const ObisCode &obis, uint16_t class_id, uint8_t attr_id);

  /* ---- APDU Parsers (RX) ---- */

  /// Parse proprietary read response from meter.
  /// Response format: [3-byte header][status=0x00][attr_id][DLMS typed data]
  /// Strips envelope header and extracts typed value.
  bool parse_read_response(const uint8_t *data, size_t len, DlmsValue *value_out);

  /* ---- State ---- */

  DlmsState get_state() const { return state_; }
  void set_state(DlmsState state) { state_ = state; }
  void reset() { state_ = DlmsState::IDLE; }

 private:
  /// Parse a DLMS typed value at data[offset]. Returns bytes consumed, or -1 on error.
  static int parse_typed_value(const uint8_t *data, size_t len, DlmsValue *value_out);

  DlmsState state_{DlmsState::IDLE};
  uint16_t client_addr_{16};
  uint16_t server_addr_{1};
  char password_[32]{};
};

}  // namespace esphome::nartis_rf_meter
