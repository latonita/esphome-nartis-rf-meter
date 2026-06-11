/*
 * Nartis RF Meter — Shared Types and Constants
 *
 * Frame structure verified end-to-end against:
 *   - 104 CIU→meter SPI-dumped frames (fifo.txt, Pair-B)
 *   - 104 meter→CIU SPI-dumped frames (Pair-B)
 *   - Ghidra decompile of rf_build_frame, rf_parse_frame, crc_block_insert,
 *     crc_block_deframe, rx_address_verify, derive_rf_address
 *
 * See FRAME_HEADER_SPEC.md in the firmware research repo for full layout
 * documentation including dual-CRC handling for frames > 128 bytes.
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <array>

namespace esphome::nartis_rf_meter {

/* ================================================================
 * RF Protocol Constants (from firmware template at 0x0000fdf4)
 * ================================================================ */
static constexpr uint16_t RF_DEVICE_ID = 0x2CCD;   // bytes "CD 2C" little-endian
static constexpr uint8_t RF_GROUP_ID = 0x50;        // byte [8] of 8-byte address
static constexpr uint8_t RF_DEVICE_TYPE_CIU = 0x25; // byte [9] of CIU address
static constexpr uint8_t RF_DEVICE_TYPE_METER = 0x02; // byte [9] of meter address

/* ================================================================
 * Mode markers and AES-GCM security flag
 * ================================================================ */
static constexpr uint8_t MODE_MARKER_NORMAL = 0x7A;   // header[10] for Mode 1/2/3 (TX)
static constexpr uint8_t MODE_MARKER_SPECIAL = 0x8A;  // header[10] for Mode 6 (TX)
static constexpr uint8_t AES_GCM_FLAG = 0x29;         // fixed security/AAD header byte (AAD = 01 29 len 00)

/* ================================================================
 * AES-128-GCM Constants
 * ================================================================ */
static constexpr size_t AES_KEY_SIZE = 16;
static constexpr size_t AES_NONCE_SIZE = 12;     // 8-byte addr + 4-byte counter
static constexpr size_t AES_TAG_SIZE = 12;       // GCM tag length (12 bytes)

/* ================================================================
 * CRC-16/DNP Constants
 * ================================================================ */
static constexpr uint16_t CRC16_POLY = 0x3D65;
static constexpr uint16_t CRC16_INIT = 0x0000;
static constexpr uint16_t CRC16_XOROUT = 0xFFFF;

/* ================================================================
 * RF Address — 8-byte session identifier
 *
 * Per-device, derived from serial:
 *   [0..1] device_id_prefix  = 0x2CCD constant (bytes CD 2C on wire)
 *   [2..4] serial_digits     = strtoul(serial[-7:], 10) as 3 bytes BE
 *   [5]    version_seed      = some byte derived per device
 *   [6]    group_id          = 0x50 constant
 *   [7]    device_type       = 0x25 CIU | 0x02 meter
 *
 * The CIU has device_type=0x25; meter responses carry meter's address
 * with device_type=0x02.
 * ================================================================ */
struct RfAddress {
  uint16_t device_id{RF_DEVICE_ID};
  uint32_t serial_hash{0};  // [2..5]: 3 BE digit bytes + 1 seed byte, packed LE into u32
  uint8_t group_id{RF_GROUP_ID};
  uint8_t device_type{RF_DEVICE_TYPE_CIU};

  /// Serialize to 8-byte on-wire buffer.
  /// Layout: [device_id_lo, device_id_hi, hash[0..3], group_id, device_type]
  void to_bytes(uint8_t out[8]) const {
    out[0] = static_cast<uint8_t>(device_id & 0xFF);
    out[1] = static_cast<uint8_t>((device_id >> 8) & 0xFF);
    out[2] = static_cast<uint8_t>(serial_hash & 0xFF);
    out[3] = static_cast<uint8_t>((serial_hash >> 8) & 0xFF);
    out[4] = static_cast<uint8_t>((serial_hash >> 16) & 0xFF);
    out[5] = static_cast<uint8_t>((serial_hash >> 24) & 0xFF);
    out[6] = group_id;
    out[7] = device_type;
  }

  /// Deserialize from 8-byte on-wire buffer.
  static RfAddress from_bytes(const uint8_t in[8]) {
    RfAddress addr;
    addr.device_id = static_cast<uint16_t>(in[0]) | (static_cast<uint16_t>(in[1]) << 8);
    addr.serial_hash = static_cast<uint32_t>(in[2]) |
                       (static_cast<uint32_t>(in[3]) << 8) |
                       (static_cast<uint32_t>(in[4]) << 16) |
                       (static_cast<uint32_t>(in[5]) << 24);
    addr.group_id = in[6];
    addr.device_type = in[7];
    return addr;
  }

  bool operator==(const RfAddress &other) const {
    return device_id == other.device_id &&
           serial_hash == other.serial_hash &&
           group_id == other.group_id &&
           device_type == other.device_type;
  }

  /// Derive RF address from a CIU serial string + version seed.
  /// Matches firmware derive_rf_address (0xFAAC).
  ///
  /// Algorithm:
  ///   1. Take last 7 chars of serial
  ///   2. strtoul(str, 10) — stops at first non-digit (e.g. "02RV634" → 2)
  ///   3. config[3..5] = 3 bytes big-endian of result
  ///   4. config[6] = seed % 255
  ///   5. Fixed device_id=0x2CCD, group_id=0x50, device_type=0x25
  ///
  /// Verified examples:
  ///   "MPCUA00294W6" + seed 230 → CD 2C 00 01 26 E6 50 25  (Pair A)
  ///   "MPCUA002RV634" + seed 107 → CD 2C 00 00 02 6B 50 25  (Pair B)
  static RfAddress derive(const char *ciu_serial, uint32_t seed) {
    RfAddress addr;
    addr.device_type = RF_DEVICE_TYPE_CIU;

    size_t len = 0;
    while (ciu_serial[len] != '\0') len++;
    const char *substr = (len > 7) ? (ciu_serial + (len - 7)) : ciu_serial;

    // strtoul base 10, stops at first non-digit
    uint32_t serial_digits = 0;
    for (const char *p = substr; *p != '\0'; p++) {
      if (*p < '0' || *p > '9') break;
      serial_digits = serial_digits * 10 + static_cast<uint32_t>(*p - '0');
    }

    uint8_t seed_byte = static_cast<uint8_t>(seed % 255);

    // Pack as 3 bytes BE (high to low) + seed_byte into u32 LE
    addr.serial_hash = static_cast<uint32_t>((serial_digits >> 16) & 0xFF) |
                       (static_cast<uint32_t>((serial_digits >> 8) & 0xFF) << 8) |
                       (static_cast<uint32_t>(serial_digits & 0xFF) << 16) |
                       (static_cast<uint32_t>(seed_byte) << 24);
    return addr;
  }
};

/* ================================================================
 * RF Frame Types — outgoing (CIU→meter) header byte [1]
 *
 * Verified in fifo.txt (104 frames): only 0x44 and 0x08 observed.
 * The other modes are documented in firmware radio_set_mode.
 * ================================================================ */
enum class RfFrameType : uint8_t {
  BEACON = 0x08,       // Mode 3: wake/beacon (short encrypted frame)
  DATA = 0x46,         // Mode 1: data frame
  ACK = 0x44,          // Mode 2: ACK / response (large encrypted frame)
  PLAIN_DATA = 0x00,   // Mode 6: special-channel config (no RF AES-GCM)
};

/* ================================================================
 * RF Frame Header Offsets — OUTGOING (CIU → meter)
 *
 * Total header = 15 bytes [0..14], then body starts at [15].
 * ================================================================ */
static constexpr size_t RF_TX_LENGTH       =  0;  // length-1
static constexpr size_t RF_TX_FLAGS        =  1;  // RfFrameType value
static constexpr size_t RF_TX_ADDR         =  2;  // 8 bytes: CIU address
static constexpr size_t RF_TX_ADDR_LEN     =  8;
static constexpr size_t RF_TX_MODE_MARKER  = 10;  // 0x7A normal | 0x8A special
static constexpr size_t RF_TX_SEQUENCE     = 11;  // per-mode sequence counter
static constexpr size_t RF_TX_CHANNEL_BYTE = 12;  // (chan<<6) | (quality & 0x3F)
static constexpr size_t RF_TX_ENC_FLAG     = 13;  // 0x01 encrypted | 0x00 plain
static constexpr size_t RF_TX_SEC_FLAG     = 14;  // 0x29 if encrypted | 0x00 plain
static constexpr size_t RF_TX_HDR_SIZE     = 15;  // bytes [0..14]

// Encrypted body offsets (when ENC_FLAG=0x01)
static constexpr size_t RF_TX_ENC_DATA_LEN = 15;  // L = ciphertext length
static constexpr size_t RF_TX_ENC_ZERO     = 16;  // constant 0x00
static constexpr size_t RF_TX_ENC_COUNTER  = 17;  // 4-byte BE u32
static constexpr size_t RF_TX_ENC_CIPHER   = 21;  // L bytes of ciphertext
// MIC follows at [21 + L .. 21 + L + 11]
// CRC(s) follow MIC — single CRC if total ≤ 128, dual CRC if total > 128

/* ================================================================
 * RF Frame Header Offsets — INCOMING (meter → CIU)
 *
 * Address [2..9] is the meter's own address (device_type = 0x02).
 * Mode flags at [11]/[12] BOTH zero ⇒ plain branch (firmware parser logic).
 * For our observed dataset all RX frames are plain at the transport layer;
 * RX type 0x43 plain payloads carry a NESTED encrypted frame inside.
 * ================================================================ */
static constexpr size_t RF_RX_LENGTH       =  0;
static constexpr size_t RF_RX_TYPE         =  1;  // 0x40, 0x43, 0x53, 0x5B, 0x06
static constexpr size_t RF_RX_ADDR         =  2;  // 8 bytes: meter address
static constexpr size_t RF_RX_PASSTHROUGH  = 10;  // varies by type (0x80, 0x5B, ...)
static constexpr size_t RF_RX_MODE_FLAG_LO = 11;  // 0 ⇒ plain branch
static constexpr size_t RF_RX_MODE_FLAG_HI = 12;  // 0 ⇒ plain branch
static constexpr size_t RF_RX_HDR_SIZE     = 13;  // bytes [0..12]
static constexpr size_t RF_RX_PAYLOAD      = 13;  // plain payload start
// In plain frames, CRC(s) follow payload (single or dual based on total length)

/* ================================================================
 * Dual-CRC threshold (from crc_block_insert @ 0xB9B2)
 *
 * Pre-CRC body size > 126 ⇒ insert CRC1 at offset 0x7E, CRC2 at end.
 * Equivalently: final on-air frame size > 128 ⇒ dual CRC.
 * ================================================================ */
static constexpr size_t CRC_DUAL_THRESHOLD = 128;  // total size; > this ⇒ dual CRC
static constexpr size_t CRC1_OFFSET        = 0x7E; // CRC1 position in dual-CRC frames

/* ================================================================
 * Legacy aliases (deprecated — use RF_TX_* / RF_RX_* instead)
 * Kept for transient compile compatibility; will be removed.
 * ================================================================ */
static constexpr size_t RF_HDR_LENGTH = RF_TX_LENGTH;
static constexpr size_t RF_HDR_FLAGS  = RF_TX_FLAGS;
static constexpr size_t RF_HDR_ADDR   = RF_TX_ADDR;
static constexpr size_t RF_HDR_ADDR_LEN = RF_TX_ADDR_LEN;
static constexpr size_t RF_HDR_SIZE   = RF_TX_HDR_SIZE;

/* ================================================================
 * Nartis Proprietary Protocol Tags (DLMS-layer)
 * ================================================================ */
static constexpr uint8_t NARTIS_TAG_READ_REQ = 0xC0;    // Read request
static constexpr uint8_t NARTIS_TAG_READ_RSP = 0xC1;    // Read response / ACK
static constexpr uint8_t NARTIS_TAG_ACTION_REQ = 0xC3;  // Action/write request

/* ================================================================
 * OBIS Code
 * ================================================================ */
struct ObisCode {
  uint8_t bytes[6];
  bool operator==(const ObisCode &other) const {
    for (int i = 0; i < 6; i++) {
      if (bytes[i] != other.bytes[i]) return false;
    }
    return true;
  }
};

/* ================================================================
 * DLMS Value — parsed response data
 * ================================================================ */
// A parsed COSEM attribute value, kept as the RAW on-the-wire bytes plus the
// DLMS type tag. Interpretation (→ float for sensors, → string for text
// sensors) is deferred to DlmsClient::data_as_float() / data_to_string(), so
// every consumer shares one battle-tested conversion. `dtype` is stored as the
// raw type byte (helpers.h can't see the DlmsDataType enum); cast when used.
struct DlmsValue {
  uint8_t dtype{0x00};   // DLMS data-type tag (0x00 = null / no value)
  uint8_t raw_len{0};    // valid bytes in raw[]
  uint8_t raw[64]{};     // value bytes, big-endian as received
  bool valid{false};     // false ⇒ NULL-data / unparsed
  bool has_value() const { return valid; }
};

/* ================================================================
 * Buffer sizes
 * ================================================================ */
static constexpr size_t MAX_RF_FRAME_SIZE = 512;
static constexpr size_t MAX_DLMS_APDU_SIZE = 400;

}  // namespace esphome::nartis_rf_meter
