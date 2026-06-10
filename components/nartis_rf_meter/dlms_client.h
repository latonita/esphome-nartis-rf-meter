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

  /// Single COSEM attribute reference for a get-request list entry.
  struct AttrSpec {
    uint16_t class_id;
    ObisCode obis;
    uint8_t attr_id;
  };

  static constexpr uint8_t MAX_LIST_ATTRS = 10;  // firmware caps at 10 per request

  /// Build a `get-request-with-list` APDU containing one or more attribute
  /// references. Format (matches dump-spi2 frame #0 / firmware
  /// nartis_build_read 0x8cac):
  ///   c0 03  c1  <count>
  ///   <count × { class-id (2B BE) | OBIS (6B) | attr-id (1B) | access-sel (1B = 0x00) }>
  /// Returns total bytes written, or 0 on error.
  size_t build_get_request_with_list(uint8_t *out, size_t max,
                                     const AttrSpec *attrs, uint8_t count);

  /// Single-OBIS convenience wrapper — wraps a single attribute in the
  /// `with-list` form (count=1). Used for steady-state reads.
  size_t build_read_request(uint8_t *out, size_t max,
                            const ObisCode &obis, uint16_t class_id, uint8_t attr_id);

  /// Build a `get-request-normal` APDU (single attribute, c0 01 form).
  /// Format (verified against spi4 pairing capture frame #10 — the FIRST
  /// post-pairing read the real CIU issues):
  ///   c0 01  c1  <class-id 2B BE>  <OBIS 6B>  <attr-id 1B>  <access-sel 1B = 0x00>
  /// The freshly-paired meter requires this normal-get (answered by a direct
  /// 0x43, no keepalive) before it will engage the with-list flow.
  /// Returns total bytes written (13), or 0 on error.
  size_t build_get_request_normal(uint8_t *out, size_t max,
                                  const ObisCode &obis, uint16_t class_id, uint8_t attr_id);

  /* ---- APDU Parsers (RX) ---- */

  /// Parse a `get-response-with-list` (firmware-style multi-result response).
  /// Strips the Nartis prefix (0d fd f8 + 2B tag) and IEC 62056-47 wrapper if
  /// present. Then walks the result list, filling `values[]` from left to right.
  ///
  /// @param data    pointer to the decrypted payload (Nartis prefix + IEC + DLMS)
  /// @param len     total payload bytes
  /// @param values  caller-owned array of at least max_results DlmsValue slots
  /// @param max_results capacity of `values[]`
  /// @param count_out actual number of results returned by the meter (≤ max_results)
  /// @return true if header parsed OK; individual result slots may carry NONE
  ///         type if their data couldn't be parsed.
  bool parse_read_response_list(const uint8_t *data, size_t len,
                                DlmsValue *values, uint8_t max_results,
                                uint8_t *count_out);

  /// Legacy single-attribute response parser (kept for backwards compat).
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
