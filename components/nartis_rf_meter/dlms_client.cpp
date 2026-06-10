/*
 * Nartis DLMS Client — Implementation
 *
 * Lightweight DLMS/COSEM APDU builder and parser.
 * Based on reverse-engineered CIU firmware protocol.
 */

#include "dlms_client.h"
#include <cstring>
#include <cmath>

#include "esphome/core/log.h"

namespace esphome::nartis_rf_meter {

static const char *const TAG = "dlms_client";

/* ================================================================
 * Configuration
 * ================================================================ */

void DlmsClient::set_credentials(const char *password, uint16_t client_addr, uint16_t server_addr) {
  strncpy(password_, password, sizeof(password_) - 1);
  password_[sizeof(password_) - 1] = '\0';
  client_addr_ = client_addr;
  server_addr_ = server_addr;
}

/* ================================================================
 * Read Request Builder — DLMS get-request-with-list (matches firmware)
 *
 * spi2 frame #0 (real-CIU first post-pairing GET) shows the firmware uses
 * the standard DLMS `get-request-with-list` form, NOT `get-request-normal`,
 * even when only one attribute is interesting. Meter responds with
 * `get-response-with-list`. The single-OBIS form (`c0 01`) gets `0x40`
 * (no-data short-ack) instead of `0x43`.
 *
 * Format (verified against decrypted spi2 frame #0 payload):
 *
 *   c0 03                          ← tag + type (get-request-with-list)
 *   c1                             ← invoke-id-priority
 *   N                              ← attribute-list count (1..10 in captures)
 *   [N × 10-byte attribute spec]:
 *     2 B    class-id (BE u16)     ← e.g. 00 01 = 1 (Data class)
 *     6 B    OBIS code             ← e.g. 00 00 60 80 03 FF
 *     1 B    attr-id               ← e.g. 02 (value)
 *     1 B    access-selector       ← 0x00 (none) in all firmware captures
 *
 * The legacy single-OBIS form is kept as `build_read_request_normal` for
 * reference but not used in the GET path.
 * ================================================================ */

size_t DlmsClient::build_get_request_with_list(uint8_t *out, size_t max,
                                               const AttrSpec *attrs, uint8_t count) {
  if (count == 0 || count > MAX_LIST_ATTRS) return 0;
  // Each entry is 10 bytes (class 2 + OBIS 6 + attr 1 + access 1); header is 4.
  const size_t needed = 4 + 10u * count;
  if (max < needed) return 0;

  size_t pos = 0;
  out[pos++] = 0xC0;   // tag = get-request
  out[pos++] = 0x03;   // type = get-request-with-list
  out[pos++] = 0xC1;   // invoke-id-priority
  out[pos++] = count;  // attribute-list count

  for (uint8_t i = 0; i < count; i++) {
    const AttrSpec &a = attrs[i];
    out[pos++] = static_cast<uint8_t>((a.class_id >> 8) & 0xFF);
    out[pos++] = static_cast<uint8_t>(a.class_id & 0xFF);
    memcpy(out + pos, a.obis.bytes, 6);
    pos += 6;
    out[pos++] = a.attr_id;
    out[pos++] = 0x00;  // access-selector = none
  }

  state_ = DlmsState::REQUEST_SENT;

  ESP_LOGD(TAG, "Built get-request-with-list: count=%d (%d bytes)", count, (int) pos);
  for (uint8_t i = 0; i < count; i++) {
    const AttrSpec &a = attrs[i];
    ESP_LOGD(TAG, "  [%d] class=%u OBIS=%u.%u.%u.%u.%u.%u attr=%u",
             i, a.class_id, a.obis.bytes[0], a.obis.bytes[1], a.obis.bytes[2],
             a.obis.bytes[3], a.obis.bytes[4], a.obis.bytes[5], a.attr_id);
  }
  return pos;
}

size_t DlmsClient::build_read_request(uint8_t *out, size_t max,
                                      const ObisCode &obis, uint16_t class_id,
                                      uint8_t attr_id) {
  // Single-attribute wrapper around the list builder.
  AttrSpec a{class_id, obis, attr_id};
  return build_get_request_with_list(out, max, &a, 1);
}

size_t DlmsClient::build_get_request_normal(uint8_t *out, size_t max,
                                            const ObisCode &obis, uint16_t class_id,
                                            uint8_t attr_id) {
  // c0 01 c1 <class 2B> <OBIS 6B> <attr 1B> <access-sel 1B> = 13 bytes.
  const size_t needed = 13;
  if (max < needed) return 0;

  size_t pos = 0;
  out[pos++] = 0xC0;   // tag = get-request
  out[pos++] = 0x01;   // type = get-request-normal
  out[pos++] = 0xC1;   // invoke-id-priority
  out[pos++] = static_cast<uint8_t>((class_id >> 8) & 0xFF);
  out[pos++] = static_cast<uint8_t>(class_id & 0xFF);
  memcpy(out + pos, obis.bytes, 6);
  pos += 6;
  out[pos++] = attr_id;
  out[pos++] = 0x00;   // access-selector = none

  state_ = DlmsState::REQUEST_SENT;
  ESP_LOGD(TAG, "Built get-request-normal: class=%u OBIS=%u.%u.%u.%u.%u.%u attr=%u (%d bytes)",
           class_id, obis.bytes[0], obis.bytes[1], obis.bytes[2], obis.bytes[3],
           obis.bytes[4], obis.bytes[5], attr_id, (int) pos);
  return pos;
}

/* ================================================================
 * Read Response Parser — Proprietary Nartis Protocol
 *
 * Firmware response format (parsed by nartis_parse_read_response + nartis_process_read_data):
 *   [0-2]  = 3-byte envelope header (stripped by nartis_parse_read_response)
 *   [3]    = status byte (0x00 = success)
 *   [4]    = attribute ID echo
 *   [5+]   = DLMS typed data (standard type tag + value)
 *
 * The typed data at [5+] uses standard DLMS data type encoding
 * (parsed by firmware dlms_parse_typed_value (0x9634)).
 * ================================================================ */

bool DlmsClient::parse_read_response(const uint8_t *data, size_t len, DlmsValue *value_out) {
  if (len < 6) {
    ESP_LOGW(TAG, "Response too short: %d bytes", (int) len);
    return false;
  }

  // Log first bytes for debugging
  ESP_LOGD(TAG, "Response (%d bytes): %02X %02X %02X %02X %02X %02X ...",
           (int) len, data[0], data[1], data[2],
           len > 3 ? data[3] : 0, len > 4 ? data[4] : 0, len > 5 ? data[5] : 0);

  // Skip 3-byte envelope header (firmware nartis_parse_read_response strips these)
  size_t offset = 3;

  // Check status byte — must be 0x00 for success
  if (data[offset] != 0x00) {
    ESP_LOGW(TAG, "Read response error: status=0x%02X", data[offset]);
    return false;
  }
  offset++;  // skip status

  offset++;  // skip attribute ID echo

  // Parse DLMS typed value
  if (offset >= len) {
    ESP_LOGW(TAG, "No data after response header");
    return false;
  }

  int consumed = parse_typed_value(data + offset, len - offset, value_out);
  if (consumed < 0) {
    ESP_LOGW(TAG, "Failed to parse response value at offset %d", (int) offset);
    return false;
  }

  state_ = DlmsState::IDLE;
  return true;
}

/* ================================================================
 * Multi-result response parser — get-response-with-list
 *
 * Decrypted payload layout observed in dump-spi2 frame #3:
 *
 *   0d fd f8 <2B tag/id>           ← Nartis prefix (5 bytes, optional)
 *   00 01 00 01 00 66 00 <len>     ← IEC 62056-47 wrapper (8 bytes)
 *   c4 03 c1 <count>               ← get-response-with-list header
 *   <count × result>               ← each: tag (0=data | 1=err) + value
 *
 * Each `data` result is a DLMS typed value (already handled by parse_typed_value).
 * Compound types (structure / array) are length-prefixed: we recursively skip
 * past their inner items so the outer parser stays aligned.
 * ================================================================ */

// Recursively consume one DLMS typed value (returns bytes consumed, or -1 on error).
// Used for skipping compound types we don't extract into DlmsValue.
static int skip_typed_value(const uint8_t *data, size_t len) {
  if (len < 1) return -1;
  uint8_t type = data[0];
  size_t pos = 1;
  switch (type) {
    case 0x00: return 1;                                          // null

    case 0x03:                                                    // boolean
    case 0x0F:                                                    // int8
    case 0x11:                                                    // uint8
    case 0x16:                                                    // enum
      return (len < 2) ? -1 : 2;

    case 0x10:                                                    // int16
    case 0x12:                                                    // uint16
      return (len < 3) ? -1 : 3;

    case 0x05:                                                    // int32 (double-long)
    case 0x06:                                                    // uint32 (double-long-unsigned)
    case 0x17:                                                    // float32
      return (len < 5) ? -1 : 5;

    case 0x14:                                                    // int64
    case 0x15:                                                    // uint64 (long64-unsigned)
    case 0x18:                                                    // float64
      return (len < 9) ? -1 : 9;

    case 0x09:                                                    // octet-string
    case 0x0A:                                                    // visible-string
    case 0x0C: {                                                  // UTF-8 string
      if (len < 2) return -1;
      uint8_t slen = data[pos++];
      if (pos + slen > len) return -1;
      return static_cast<int>(pos + slen);
    }

    case 0x01:                                                    // array
    case 0x02: {                                                  // structure
      if (len < 2) return -1;
      uint8_t count = data[pos++];
      for (uint8_t i = 0; i < count; i++) {
        int n = skip_typed_value(data + pos, len - pos);
        if (n < 0) return -1;
        pos += n;
      }
      return static_cast<int>(pos);
    }

    case 0x19: return (len < 13) ? -1 : 13;                       // date-time
    case 0x1A: return (len < 6)  ? -1 : 6;                        // date
    case 0x1B: return (len < 5)  ? -1 : 5;                        // time

    default:
      return -1;
  }
}

bool DlmsClient::parse_read_response_list(const uint8_t *data, size_t len,
                                          DlmsValue *values, uint8_t max_results,
                                          uint8_t *count_out) {
  if (count_out) *count_out = 0;
  if (len < 6) {
    ESP_LOGW(TAG, "list response too short (%u B)", (unsigned) len);
    return false;
  }

  size_t pos = 0;

  // Optional Nartis prefix `0d fd f8 <tag1> <tag2>`
  if (len >= 5 && data[0] == 0x0D && data[1] == 0xFD && data[2] == 0xF8) {
    pos = 5;
  }

  // IEC 62056-47 wrapper: 00 01 <src-wport 2B> <dst-wport 2B> <length 2B BE>
  if (len < pos + 8) {
    ESP_LOGW(TAG, "list response: no IEC wrapper at pos %u", (unsigned) pos);
    return false;
  }
  if (data[pos] != 0x00 || data[pos + 1] != 0x01) {
    ESP_LOGW(TAG, "list response: bad IEC version %02X %02X", data[pos], data[pos + 1]);
    return false;
  }
  pos += 8;  // skip wrapper (version + wPorts + length)

  // DLMS get-response-with-list header
  if (len < pos + 4) {
    ESP_LOGW(TAG, "list response: too short for DLMS header");
    return false;
  }
  if (data[pos] != 0xC4) {
    ESP_LOGW(TAG, "list response: tag != 0xC4 (got 0x%02X)", data[pos]);
    return false;
  }
  // Accept both get-response-with-list (0x03: invoke + count, then N results)
  // and get-response-normal (0x01: invoke, then a single result — the meter
  // answers a 1-attr list this way). Normal has no count byte.
  uint8_t count;
  if (data[pos + 1] == 0x03) {
    count = data[pos + 3];   // C4 03 <invoke> <count>
    pos += 4;
  } else if (data[pos + 1] == 0x01) {
    count = 1;               // C4 01 <invoke> <single result>
    pos += 3;
  } else {
    ESP_LOGW(TAG, "list response: type != normal/with-list (got 0x%02X)", data[pos + 1]);
    return false;
  }

  ESP_LOGD(TAG, "get-response: count=%u (max %u slots)", count, max_results);

  uint8_t stored = 0;
  for (uint8_t i = 0; i < count; i++) {
    if (pos >= len) {
      ESP_LOGW(TAG, "  [%d] truncated", i);
      break;
    }
    uint8_t result_tag = data[pos++];
    if (result_tag == 1) {
      // data-access-result: 1 error byte
      if (pos >= len) break;
      uint8_t err = data[pos++];
      ESP_LOGW(TAG, "  [%d] access-error = 0x%02X", i, err);
      if (stored < max_results) {
        values[stored].type = DlmsValue::NONE;
        stored++;
      }
      continue;
    }
    if (result_tag != 0) {
      ESP_LOGW(TAG, "  [%d] unknown result tag 0x%02X", i, result_tag);
      break;
    }

    // result_tag == 0: data follows. Decide whether to extract (simple scalar)
    // or skip (compound type). Either way, advance `pos` correctly.
    if (pos >= len) break;
    uint8_t type_byte = data[pos];
    const bool is_compound = (type_byte == 0x01 || type_byte == 0x02);

    if (!is_compound && stored < max_results) {
      int n = parse_typed_value(data + pos, len - pos, &values[stored]);
      if (n < 0) {
        ESP_LOGW(TAG, "  [%d] parse_typed_value failed (type=0x%02X)", i, type_byte);
        // skip whatever follows so we stay aligned
        int s = skip_typed_value(data + pos, len - pos);
        if (s < 0) break;
        pos += s;
        values[stored].type = DlmsValue::NONE;
      } else {
        pos += n;
        ESP_LOGD(TAG, "  [%d] type=0x%02X parsed (%d B)", i, type_byte, n);
      }
      stored++;
    } else {
      // Compound, or we've run out of caller-supplied slots — just skip.
      int s = skip_typed_value(data + pos, len - pos);
      if (s < 0) {
        ESP_LOGW(TAG, "  [%d] skip failed (type=0x%02X)", i, type_byte);
        break;
      }
      ESP_LOGD(TAG, "  [%d] type=0x%02X skipped (%d B)", i, type_byte, s);
      pos += s;
      if (stored < max_results) {
        values[stored].type = DlmsValue::NONE;
        stored++;
      }
    }
  }

  if (count_out) *count_out = stored;
  state_ = DlmsState::IDLE;
  return true;
}

/* ================================================================
 * Typed Value Parser
 *
 * Parses DLMS Data type-length-value encoding.
 * Returns bytes consumed, or -1 on error.
 * ================================================================ */

int DlmsClient::parse_typed_value(const uint8_t *data, size_t len, DlmsValue *value_out) {
  if (len < 1) return -1;

  auto type = static_cast<DlmsDataType>(data[0]);
  size_t pos = 1;

  switch (type) {
    case DlmsDataType::NULL_DATA:
      value_out->type = DlmsValue::NONE;
      return 1;

    case DlmsDataType::BOOLEAN:
      if (len < 2) return -1;
      value_out->type = DlmsValue::UINT_VAL;
      value_out->uint_val = data[pos] ? 1 : 0;
      return 2;

    case DlmsDataType::INT8:
      if (len < 2) return -1;
      value_out->type = DlmsValue::INT_VAL;
      value_out->int_val = static_cast<int8_t>(data[pos]);
      return 2;

    case DlmsDataType::INT16:
      if (len < 3) return -1;
      value_out->type = DlmsValue::INT_VAL;
      value_out->int_val = static_cast<int16_t>((data[pos] << 8) | data[pos + 1]);
      return 3;

    case DlmsDataType::INT32:
      if (len < 5) return -1;
      value_out->type = DlmsValue::INT_VAL;
      value_out->int_val = static_cast<int32_t>(
          (static_cast<uint32_t>(data[pos]) << 24) |
          (static_cast<uint32_t>(data[pos + 1]) << 16) |
          (static_cast<uint32_t>(data[pos + 2]) << 8) |
          data[pos + 3]);
      return 5;

    case DlmsDataType::UINT8:
    case DlmsDataType::ENUM:
      if (len < 2) return -1;
      value_out->type = DlmsValue::UINT_VAL;
      value_out->uint_val = data[pos];
      return 2;

    case DlmsDataType::UINT16:
      if (len < 3) return -1;
      value_out->type = DlmsValue::UINT_VAL;
      value_out->uint_val = (static_cast<uint32_t>(data[pos]) << 8) | data[pos + 1];
      return 3;

    case DlmsDataType::UINT32:
      if (len < 5) return -1;
      value_out->type = DlmsValue::UINT_VAL;
      value_out->uint_val = (static_cast<uint32_t>(data[pos]) << 24) |
                            (static_cast<uint32_t>(data[pos + 1]) << 16) |
                            (static_cast<uint32_t>(data[pos + 2]) << 8) |
                            data[pos + 3];
      return 5;

    case DlmsDataType::FLOAT32: {
      if (len < 5) return -1;
      uint32_t raw = (static_cast<uint32_t>(data[pos]) << 24) |
                     (static_cast<uint32_t>(data[pos + 1]) << 16) |
                     (static_cast<uint32_t>(data[pos + 2]) << 8) |
                     data[pos + 3];
      float fval;
      memcpy(&fval, &raw, sizeof(float));
      value_out->type = DlmsValue::FLOAT_VAL;
      value_out->float_val = fval;
      return 5;
    }

    case DlmsDataType::FLOAT64: {
      if (len < 9) return -1;
      uint64_t raw = 0;
      for (int i = 0; i < 8; i++) {
        raw = (raw << 8) | data[pos + i];
      }
      double dval;
      memcpy(&dval, &raw, sizeof(double));
      value_out->type = DlmsValue::FLOAT_VAL;
      value_out->float_val = static_cast<float>(dval);
      return 9;
    }

    case DlmsDataType::OCTET_STRING:
    case DlmsDataType::VISIBLE_STRING:
    case DlmsDataType::UTF8_STRING: {
      if (len < 2) return -1;
      uint8_t str_len = data[pos++];
      if (pos + str_len > len) return -1;

      size_t copy_len = str_len;
      if (copy_len >= sizeof(value_out->str_val)) {
        copy_len = sizeof(value_out->str_val) - 1;
      }

      if (type == DlmsDataType::OCTET_STRING) {
        // Format as hex string
        size_t hex_pos = 0;
        for (size_t i = 0; i < str_len && hex_pos + 2 < sizeof(value_out->str_val); i++) {
          static const char hex_chars[] = "0123456789ABCDEF";
          value_out->str_val[hex_pos++] = hex_chars[(data[pos + i] >> 4) & 0x0F];
          value_out->str_val[hex_pos++] = hex_chars[data[pos + i] & 0x0F];
        }
        value_out->str_val[hex_pos] = '\0';
      } else {
        memcpy(value_out->str_val, data + pos, copy_len);
        value_out->str_val[copy_len] = '\0';
      }

      value_out->type = DlmsValue::STRING_VAL;
      return 1 + 1 + str_len;  // type + length + data
    }

    case DlmsDataType::INT64: {
      if (len < 9) return -1;
      int64_t val = 0;
      for (int i = 0; i < 8; i++) {
        val = (val << 8) | data[pos + i];
      }
      value_out->type = DlmsValue::INT_VAL;
      value_out->int_val = static_cast<int32_t>(val);  // May truncate
      return 9;
    }

    case DlmsDataType::UINT64: {
      if (len < 9) return -1;
      uint64_t val = 0;
      for (int i = 0; i < 8; i++) {
        val = (val << 8) | data[pos + i];
      }
      value_out->type = DlmsValue::UINT_VAL;
      value_out->uint_val = static_cast<uint32_t>(val);  // May truncate
      return 9;
    }

    default:
      ESP_LOGW(TAG, "Unsupported DLMS data type: 0x%02X", static_cast<uint8_t>(type));
      value_out->type = DlmsValue::NONE;
      return -1;
  }
}

}  // namespace esphome::nartis_rf_meter
