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
 * Read Request Builder — Proprietary Nartis Protocol
 *
 * Firmware (nartis_build_read (0x8CAC)) uses a proprietary format instead of
 * standard DLMS AARQ + GET. Each register read is self-contained:
 *
 *   [0]  = 0xC0  (request tag)
 *   [1]  = 0x01  (single-address mode)
 *   [2]  = 0xC1  (application sub-tag)
 *   [3]  = 0x00  (reserved)
 *   [4]  = class_id (1 byte — DLMS object class)
 *   [5-10] = OBIS code (6 bytes)
 *   [11] = attr_id (attribute index)
 *   [12] = 0x00  (no auth data)
 *
 * No separate association phase — meter responds with data directly.
 * ================================================================ */

size_t DlmsClient::build_read_request(uint8_t *out, size_t max,
                                      const ObisCode &obis, uint16_t class_id,
                                      uint8_t attr_id) {
  if (max < 13) return 0;

  size_t pos = 0;
  out[pos++] = 0xC0;  // request tag
  out[pos++] = 0x01;  // single-address mode
  out[pos++] = 0xC1;  // application sub-tag
  out[pos++] = 0x00;  // reserved

  // COSEM attribute descriptor (1-byte class_id + 6-byte OBIS + 1-byte attr_id)
  out[pos++] = static_cast<uint8_t>(class_id & 0xFF);
  memcpy(out + pos, obis.bytes, 6);
  pos += 6;
  out[pos++] = attr_id;

  // No auth data
  out[pos++] = 0x00;

  state_ = DlmsState::REQUEST_SENT;

  ESP_LOGD(TAG, "Built read request: class=%d, obis=%d.%d.%d.%d.%d.%d, attr=%d (%d bytes)",
           class_id, obis.bytes[0], obis.bytes[1], obis.bytes[2],
           obis.bytes[3], obis.bytes[4], obis.bytes[5], attr_id, (int) pos);

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
