/*
 * Nartis DLMS Client — Implementation
 *
 * Lightweight DLMS/COSEM APDU builder and parser.
 */

#include "dlms_client.h"
#include <cstring>
#include <cmath>
#include <cstdio>
#include <cinttypes>

#include "esphome/core/log.h"

namespace esphome::nartis_rf_meter {

static const char *const TAG = "dlms_client";

// get-response framing bytes (meter→CIU).
static constexpr uint8_t DLMS_GET_RESPONSE   = 0xC4;  // get-response tag (request tag is 0xC0)
static constexpr uint8_t DLMS_RESP_WITH_LIST = 0x03;  // response type: with-list
static constexpr uint8_t DLMS_RESP_NORMAL    = 0x01;  // response type: normal
static constexpr uint8_t DLMS_RESULT_DATA    = 0x00;  // per-entry result: data follows
static constexpr uint8_t DLMS_RESULT_ERROR   = 0x01;  // per-entry result: access-error byte

/* ---- Big-endian byte readers (COSEM values are big-endian on the wire) ---- */
static inline uint16_t be16(const uint8_t *p) {
  return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}
static inline uint32_t be32(const uint8_t *p) {
  return (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) |
         (static_cast<uint32_t>(p[2]) << 8) | static_cast<uint32_t>(p[3]);
}
static inline uint64_t be64(const uint8_t *p) {
  uint64_t v = 0;
  for (int i = 0; i < 8; i++) v = (v << 8) | p[i];
  return v;
}

void DlmsClient::set_credentials(const char *password, uint16_t client_addr, uint16_t server_addr) {
  strncpy(password_, password, sizeof(password_) - 1);
  password_[sizeof(password_) - 1] = '\0';
  client_addr_ = client_addr;
  server_addr_ = server_addr;
}

/* ================================================================
 * Read Request Builder — DLMS get-request-with-list
 *
 * The meter uses the standard DLMS `get-request-with-list` form, NOT
 * `get-request-normal`, even when only one attribute is interesting. Meter
 * responds with `get-response-with-list`. The single-OBIS form (`c0 01`) gets
 * `0x40` (no-data short-ack) instead of `0x43`.
 *
 * Format:
 *
 *   c0 03                          ← tag + type (get-request-with-list)
 *   c1                             ← invoke-id-priority
 *   N                              ← attribute-list count (1..10)
 *   [N × 10-byte attribute spec]:
 *     2 B    class-id (BE u16)     ← e.g. 00 01 = 1 (Data class)
 *     6 B    OBIS code             ← e.g. 00 00 60 80 03 FF
 *     1 B    attr-id               ← e.g. 02 (value)
 *     1 B    access-selector       ← 0x00 (none)
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

  ESP_LOGV(TAG, "Built get-request-with-list: count=%d (%d bytes)", count, (int) pos);
  for (uint8_t i = 0; i < count; i++) {
    const AttrSpec &a = attrs[i];
    ESP_LOGV(TAG, "  [%d] class=%u OBIS=%u.%u.%u.%u.%u.%u attr=%u",
             i, a.class_id, a.obis.bytes[0], a.obis.bytes[1], a.obis.bytes[2],
             a.obis.bytes[3], a.obis.bytes[4], a.obis.bytes[5], a.attr_id);
  }
  return pos;
}

size_t DlmsClient::build_read_request(uint8_t *out, size_t max,
                                      const ObisCode &obis, uint16_t class_id,
                                      uint8_t attr_id) {
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

  ESP_LOGD(TAG, "Built get-request-normal: class=%u OBIS=%u.%u.%u.%u.%u.%u attr=%u (%d bytes)",
           class_id, obis.bytes[0], obis.bytes[1], obis.bytes[2], obis.bytes[3],
           obis.bytes[4], obis.bytes[5], attr_id, (int) pos);
  return pos;
}

/* ================================================================
 * Read Response Parser — Proprietary Nartis Protocol
 *
 * The meter's response format is:
 *   [0-2]  = 3-byte envelope header (stripped)
 *   [3]    = status byte (0x00 = success)
 *   [4]    = attribute ID echo
 *   [5+]   = DLMS typed data (standard type tag + value)
 *
 * The typed data at [5+] uses standard DLMS data type encoding.
 * ================================================================ */

bool DlmsClient::parse_read_response(const uint8_t *data, size_t len, DlmsValue *value_out) {
  if (len < 6) {
    ESP_LOGW(TAG, "Response too short: %d bytes", (int) len);
    return false;
  }

  ESP_LOGD(TAG, "Response (%d bytes): %02X %02X %02X %02X %02X %02X ...",
           (int) len, data[0], data[1], data[2],
           len > 3 ? data[3] : 0, len > 4 ? data[4] : 0, len > 5 ? data[5] : 0);

  // Skip 3-byte envelope header
  size_t offset = 3;

  // Check status byte — must be 0x00 for success
  if (data[offset] != 0x00) {
    ESP_LOGW(TAG, "Read response error: status=0x%02X", data[offset]);
    return false;
  }
  offset++;

  offset++;

  if (offset >= len) {
    ESP_LOGW(TAG, "No data after response header");
    return false;
  }

  int consumed = parse_typed_value(data + offset, len - offset, value_out);
  if (consumed < 0) {
    ESP_LOGW(TAG, "Failed to parse response value at offset %d", (int) offset);
    return false;
  }

  return true;
}

/* ================================================================
 * Multi-result response parser — get-response-with-list
 *
 * Decrypted payload layout:
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
                                          uint8_t *count_out, const uint16_t *class_ids) {
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
  if (data[pos] != DLMS_GET_RESPONSE) {
    ESP_LOGW(TAG, "list response: tag != 0xC4 (got 0x%02X)", data[pos]);
    return false;
  }
  // Our user reads are always get-request-with-list, and the meter always
  // answers them with get-response-with-list (0xC4 0x03) — even a 1-attr batch
  // (a single-OBIS request still returns `C4 03 C1 01 …`). The ONLY
  // thing that emits get-response-normal (0xC4 0x01) is the priming
  // get-request-normal. The meter sometimes BUFFERS that priming answer and
  // delivers it LATE, landing as the first 0x43 of the user-read phase. If we
  // accepted the normal form here we'd store its stray scalar into the first
  // batch slot (e.g. Voltage) — the bogus 5.4e17 V reading. Reject anything
  // that isn't a with-list response so the caller re-sends the request and
  // gets the real C4 03 answer.
  if (data[pos + 1] != DLMS_RESP_WITH_LIST) {
    ESP_LOGW(TAG, "list response: not get-response-with-list (got 0x%02X 0x%02X) — "
                  "ignoring stale/mismatched response", data[pos], data[pos + 1]);
    return false;
  }
  uint8_t count = data[pos + 3];   // C4 03 <invoke> <count>
  pos += 4;

  ESP_LOGD(TAG, "get-response: count=%u (max %u slots)", count, max_results);

  uint8_t stored = 0;
  for (uint8_t i = 0; i < count; i++) {
    if (pos >= len) {
      ESP_LOGW(TAG, "  [%d] truncated", i);
      break;
    }
    uint8_t result_tag = data[pos++];
    if (result_tag == DLMS_RESULT_ERROR) {
      // data-access-result: 1 error byte
      if (pos >= len) break;
      uint8_t err = data[pos++];
      ESP_LOGW(TAG, "  [%d] access-error = 0x%02X", i, err);
      if (stored < max_results) {
        values[stored].valid = false;
        stored++;
      }
      continue;
    }
    if (result_tag != DLMS_RESULT_DATA) {
      ESP_LOGW(TAG, "  [%d] unknown result tag 0x%02X", i, result_tag);
      break;
    }

    // result_tag == 0: data follows. Decide whether to extract (simple scalar)
    // or skip (compound type). Either way, advance `pos` correctly.
    if (pos >= len) break;
    uint8_t type_byte = data[pos];
    const bool is_compound = (type_byte == 0x01 || type_byte == 0x02);

    if (!is_compound && stored < max_results) {
      const uint16_t class_hint = (class_ids && stored < max_results) ? class_ids[stored] : 0;
      int n = parse_typed_value(data + pos, len - pos, &values[stored], class_hint);
      if (n < 0) {
        ESP_LOGW(TAG, "  [%d] parse_typed_value failed (type=0x%02X)", i, type_byte);
        // skip whatever follows so we stay aligned
        int s = skip_typed_value(data + pos, len - pos);
        if (s < 0) break;
        pos += s;
        values[stored].valid = false;
      } else {
        pos += n;
        ESP_LOGV(TAG, "  [%d] type=0x%02X parsed (%d B)", i, type_byte, n);
      }
      stored++;
    } else {
      // Compound, or we've run out of caller-supplied slots — just skip.
      int s = skip_typed_value(data + pos, len - pos);
      if (s < 0) {
        ESP_LOGW(TAG, "  [%d] skip failed (type=0x%02X)", i, type_byte);
        break;
      }
      ESP_LOGV(TAG, "  [%d] type=0x%02X skipped (%d B)", i, type_byte, s);
      pos += s;
      if (stored < max_results) {
        values[stored].valid = false;
        stored++;
      }
    }
  }

  if (count_out) *count_out = stored;
  return true;
}

/* ================================================================
 * COSEM date-time formatter
 *
 * Renders the 12-byte COSEM date_time octet layout as
 *   "YYYY-MM-DD HH:MM:SS[.cc][ ±HH:MM]"
 * Unspecified fields (0xFF / year 0x0000|0xFFFF) print as '?'; the
 * hundredths and timezone-deviation suffixes are omitted when not present.
 * Layout: [0..1] year BE, [2] month, [3] day, [4] day-of-week (ignored),
 *         [5] hour, [6] min, [7] sec, [8] hundredths, [9..10] deviation BE
 *         (int16, minutes; 0x8000 = unspecified), [11] clock status (ignored).
 * ================================================================ */
static void format_cosem_datetime(const uint8_t *data, size_t len,
                                  char *buffer, size_t buf_size) {
  if (buf_size == 0) return;
  buffer[0] = '\0';
  if (len < 12) return;

  const uint16_t year = be16(data);
  const uint8_t month = data[2], day = data[3];
  const uint8_t hour = data[5], minute = data[6], second = data[7];
  const uint8_t hundredths = data[8];
  const auto deviation = static_cast<int16_t>(be16(data + 9));

  size_t pos = 0;
  auto advance = [&](int n) {
    if (n > 0 && pos + static_cast<size_t>(n) < buf_size) pos += static_cast<size_t>(n);
  };

  if (year != 0x0000 && year != 0xFFFF)
    advance(snprintf(buffer + pos, buf_size - pos, "%04u", year));
  else
    advance(snprintf(buffer + pos, buf_size - pos, "????"));
  advance(snprintf(buffer + pos, buf_size - pos, "-"));
  if (month != 0xFF && month >= 1 && month <= 12)
    advance(snprintf(buffer + pos, buf_size - pos, "%02u", month));
  else
    advance(snprintf(buffer + pos, buf_size - pos, "??"));
  advance(snprintf(buffer + pos, buf_size - pos, "-"));
  if (day != 0xFF && day >= 1 && day <= 31)
    advance(snprintf(buffer + pos, buf_size - pos, "%02u", day));
  else
    advance(snprintf(buffer + pos, buf_size - pos, "??"));
  advance(snprintf(buffer + pos, buf_size - pos, " "));
  if (hour != 0xFF && hour <= 23)
    advance(snprintf(buffer + pos, buf_size - pos, "%02u", hour));
  else
    advance(snprintf(buffer + pos, buf_size - pos, "??"));
  advance(snprintf(buffer + pos, buf_size - pos, ":"));
  if (minute != 0xFF && minute <= 59)
    advance(snprintf(buffer + pos, buf_size - pos, "%02u", minute));
  else
    advance(snprintf(buffer + pos, buf_size - pos, "??"));
  advance(snprintf(buffer + pos, buf_size - pos, ":"));
  if (second != 0xFF && second <= 59)
    advance(snprintf(buffer + pos, buf_size - pos, "%02u", second));
  else
    advance(snprintf(buffer + pos, buf_size - pos, "??"));
  // Seconds are the minimal resolution we report — hundredths are not useful.
  // if (hundredths != 0xFF && hundredths <= 99)
  //   advance(snprintf(buffer + pos, buf_size - pos, ".%02u", hundredths));
  (void) hundredths;
  if (deviation != static_cast<int16_t>(0x8000)) {
    const int abs_dev = deviation >= 0 ? deviation : -deviation;
    advance(snprintf(buffer + pos, buf_size - pos, " %c%02d:%02d",
                     deviation >= 0 ? '+' : '-', abs_dev / 60, abs_dev % 60));
  }
}

float DlmsClient::data_as_float(DlmsDataType value_type, const uint8_t *data, size_t len) {
  if (len == 0 || data == nullptr) return 0.0f;
  const uint8_t *ptr = data;

  switch (value_type) {
    case DlmsDataType::BOOLEAN:
    case DlmsDataType::ENUM:
    case DlmsDataType::UINT8:
    case DlmsDataType::BIT_STRING:
      return ptr[0];
    case DlmsDataType::INT8:
      return static_cast<int8_t>(ptr[0]);
    case DlmsDataType::UINT16:
      return len >= 2 ? static_cast<float>(be16(ptr)) : 0.0f;
    case DlmsDataType::INT16:
      return len >= 2 ? static_cast<float>(static_cast<int16_t>(be16(ptr))) : 0.0f;
    case DlmsDataType::UINT32:
      return len >= 4 ? static_cast<float>(be32(ptr)) : 0.0f;
    case DlmsDataType::INT32:
      return len >= 4 ? static_cast<float>(static_cast<int32_t>(be32(ptr))) : 0.0f;
    case DlmsDataType::UINT64:
      return len >= 8 ? static_cast<float>(be64(ptr)) : 0.0f;
    case DlmsDataType::INT64:
      return len >= 8 ? static_cast<float>(static_cast<int64_t>(be64(ptr))) : 0.0f;
    case DlmsDataType::FLOAT32: {
      if (len < 4) return 0.0f;
      const uint32_t i32 = be32(ptr);
      float f;
      memcpy(&f, &i32, sizeof(float));
      return f;
    }
    case DlmsDataType::FLOAT64: {
      if (len < 8) return 0.0f;
      const uint64_t i64 = be64(ptr);
      double d;
      memcpy(&d, &i64, sizeof(double));
      return static_cast<float>(d);
    }
    default:
      return 0.0f;
  }
}

void DlmsClient::data_to_string(DlmsDataType value_type, const uint8_t *data, size_t len,
                                char *buffer, size_t buf_size) {
  if (buf_size == 0) return;
  buffer[0] = '\0';
  if (len == 0 || data == nullptr) return;

  auto hex_of = [](const uint8_t *input, size_t in_len, char *output, size_t out_size) {
    if (out_size == 0) return;
    output[0] = '\0';
    size_t pos = 0;
    for (size_t i = 0; i < in_len && pos + 2 < out_size; i++) {
      const int written = snprintf(output + pos, out_size - pos, "%02x", input[i]);
      if (written > 0) pos += static_cast<size_t>(written);
    }
  };

  switch (value_type) {
    case DlmsDataType::OCTET_STRING:
    case DlmsDataType::VISIBLE_STRING:
    case DlmsDataType::UTF8_STRING: {
      // Copy the raw bytes verbatim. The meter stores textual registers
      // (serials, names, etc.) as octet-strings whose bytes ARE the character
      // codes — ASCII for digits, CP1251 for Cyrillic. Trailing NUL padding is
      // trimmed; control bytes (< 0x20) become '.' so an interior NUL never
      // truncates mid-string. High bytes (>= 0x80, CP1251) are preserved here
      // and converted to UTF-8 by the caller before publishing.
      size_t eff = len;
      while (eff > 0 && data[eff - 1] == 0x00) eff--;
      size_t pos = 0;
      for (size_t i = 0; i < eff && pos + 1 < buf_size; i++) {
        const uint8_t b = data[i];
        buffer[pos++] = (b >= 0x20) ? static_cast<char>(b) : '.';
      }
      buffer[pos] = '\0';
      break;
    }
    case DlmsDataType::DATE_TIME:
      format_cosem_datetime(data, len, buffer, buf_size);
      break;
    case DlmsDataType::BIT_STRING:
    case DlmsDataType::BCD:
    case DlmsDataType::DATE:
    case DlmsDataType::TIME:
      hex_of(data, len, buffer, buf_size);
      break;
    case DlmsDataType::BOOLEAN:
    case DlmsDataType::ENUM:
    case DlmsDataType::UINT8:
      snprintf(buffer, buf_size, "%u", static_cast<unsigned>(data[0]));
      break;
    case DlmsDataType::INT8:
      snprintf(buffer, buf_size, "%d", static_cast<int>(static_cast<int8_t>(data[0])));
      break;
    case DlmsDataType::UINT16:
      if (len >= 2) snprintf(buffer, buf_size, "%u", be16(data));
      break;
    case DlmsDataType::INT16:
      if (len >= 2) snprintf(buffer, buf_size, "%d", static_cast<int16_t>(be16(data)));
      break;
    case DlmsDataType::UINT32:
      if (len >= 4) snprintf(buffer, buf_size, "%" PRIu32, be32(data));
      break;
    case DlmsDataType::INT32:
      if (len >= 4) snprintf(buffer, buf_size, "%" PRId32, static_cast<int32_t>(be32(data)));
      break;
    case DlmsDataType::UINT64:
      if (len >= 8) snprintf(buffer, buf_size, "%" PRIu64, be64(data));
      break;
    case DlmsDataType::INT64:
      if (len >= 8) snprintf(buffer, buf_size, "%" PRId64, static_cast<int64_t>(be64(data)));
      break;
    case DlmsDataType::FLOAT32:
    case DlmsDataType::FLOAT64:
      snprintf(buffer, buf_size, "%f", static_cast<double>(data_as_float(value_type, data, len)));
      break;
    default:
      break;
  }
}

/* ================================================================
 * Typed Value Parser
 *
 * Captures the DLMS type tag + raw value bytes for one attribute and advances
 * past it. Interpretation is deferred to data_as_float()/data_to_string() so
 * every sensor shares one conversion. Returns total bytes consumed, or -1.
 * ================================================================ */

int DlmsClient::parse_typed_value(const uint8_t *data, size_t len, DlmsValue *value_out,
                                  uint16_t class_hint) {
  if (len < 1) return -1;

  auto type = static_cast<DlmsDataType>(data[0]);
  size_t hdr = 1;   // bytes before the value (type tag, + length byte for strings)
  size_t vlen = 0;  // value byte count

  switch (type) {
    case DlmsDataType::NULL_DATA:
      value_out->dtype = static_cast<uint8_t>(type);
      value_out->raw_len = 0;
      value_out->valid = false;  // NULL = no value
      return 1;

    case DlmsDataType::BOOLEAN:
    case DlmsDataType::ENUM:
    case DlmsDataType::UINT8:
    case DlmsDataType::INT8:
    case DlmsDataType::BIT_STRING:
      vlen = 1;
      break;
    case DlmsDataType::UINT16:
    case DlmsDataType::INT16:
      vlen = 2;
      break;
    case DlmsDataType::TIME:
      vlen = 4;
      break;
    case DlmsDataType::DATE:
      vlen = 5;
      break;
    case DlmsDataType::UINT32:
    case DlmsDataType::INT32:
    case DlmsDataType::FLOAT32:
      vlen = 4;
      break;
    case DlmsDataType::UINT64:
    case DlmsDataType::INT64:
    case DlmsDataType::FLOAT64:
      vlen = 8;
      break;
    case DlmsDataType::DATE_TIME:
      vlen = 12;
      break;
    case DlmsDataType::OCTET_STRING:
    case DlmsDataType::VISIBLE_STRING:
    case DlmsDataType::UTF8_STRING:
    case DlmsDataType::BCD:
      if (len < 2) return -1;
      vlen = data[1];
      hdr = 2;
      break;

    default:
      ESP_LOGW(TAG, "Unsupported DLMS data type: 0x%02X", static_cast<uint8_t>(type));
      value_out->valid = false;
      return -1;
  }

  if (len < hdr + vlen) return -1;

  // COSEM Clock (class 8) returns its time as a 12-byte octet-string holding a
  // date_time structure. When the user marks the attribute as class 8, treat
  // those bytes as a date_time so data_to_string() renders a timestamp.
  DlmsDataType eff_type = type;
  if (class_hint == 8 && type == DlmsDataType::OCTET_STRING && vlen >= 12) {
    eff_type = DlmsDataType::DATE_TIME;
  }

  size_t copy = vlen;
  if (copy > sizeof(value_out->raw)) copy = sizeof(value_out->raw);
  memcpy(value_out->raw, data + hdr, copy);
  value_out->raw_len = static_cast<uint8_t>(copy);
  value_out->dtype = static_cast<uint8_t>(eff_type);
  value_out->valid = true;
  return static_cast<int>(hdr + vlen);
}

}  // namespace esphome::nartis_rf_meter
