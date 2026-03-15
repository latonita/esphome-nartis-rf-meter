/*
 * Nartis RF Meter — Shared Types and Constants
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <array>

namespace esphome::nartis_rf_meter {

/* ================================================================
 * RF Protocol Constants (from firmware template at 0x0000fdf4)
 *
 * These are fixed for all Nartis CIU devices — verified against
 * EEPROM dumps and derive_rf_address (derive_rf_address (0xFAAC)).
 * ================================================================ */
static constexpr uint16_t RF_DEVICE_ID = 0x2CCD;   // bytes CD 2C in frame
static constexpr uint8_t RF_GROUP_ID = 0x50;        // byte [6] of 8-byte address
static constexpr uint8_t RF_DEVICE_TYPE = 0x25;     // byte [7] of 8-byte address

/* ================================================================
 * RF Address — 8-byte shared session identifier
 *
 * Used in frame header bytes [2-9] and as AES-CCM nonce prefix.
 * Shared by CIU and meter after pairing.
 *
 * Layout on wire:
 *   [0]   = device_id low   (0xCD)
 *   [1]   = device_id high  (0x2C)
 *   [2-5] = serial_hash     (derived from CIU serial + seed)
 *   [6]   = group_id        (0x50)
 *   [7]   = device_type     (0x25)
 * ================================================================ */
struct RfAddress {
  uint16_t device_id{RF_DEVICE_ID};
  uint32_t serial_hash{0};
  uint8_t group_id{RF_GROUP_ID};
  uint8_t device_type{RF_DEVICE_TYPE};

  /// Serialize to 8-byte buffer (little-endian for multi-byte fields)
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

  /// Deserialize from 8-byte buffer
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

  /// Derive RF address from a CIU serial string, matching firmware algorithm
  /// (derive_rf_address (0xFAAC) derive_rf_address).
  ///
  /// Algorithm:
  ///   1. Take last 7 chars of serial
  ///   2. strtoul(str, 10) → decimal digits
  ///   3. 3 bytes big-endian + seed_byte → serial_hash (LE uint32)
  ///   4. Fixed device_id=0x2CCD, group_id=0x50, device_type=0x25
  static RfAddress derive(const char *ciu_serial, uint32_t seed) {
    RfAddress addr;

    size_t len = 0;
    while (ciu_serial[len] != '\0') len++;

    const char *substr = ciu_serial;
    if (len > 7) {
      substr = ciu_serial + (len - 7);
    }

    // Parse decimal digits (strtoul base 10, stops at first non-digit)
    uint32_t serial_digits = 0;
    for (const char *p = substr; *p != '\0'; p++) {
      if (*p < '0' || *p > '9') break;
      serial_digits = serial_digits * 10 + (*p - '0');
    }

    uint8_t seed_byte = static_cast<uint8_t>(seed % 255);

    // Pack as 3 bytes big-endian + seed into serial_hash little-endian
    // config[3] = (digits>>16)&0xFF → addr byte[2] → serial_hash byte[0]
    // config[4] = (digits>>8)&0xFF  → addr byte[3] → serial_hash byte[1]
    // config[5] = digits&0xFF       → addr byte[4] → serial_hash byte[2]
    // config[6] = seed_byte         → addr byte[5] → serial_hash byte[3]
    addr.serial_hash = static_cast<uint32_t>((serial_digits >> 16) & 0xFF) |
                       (static_cast<uint32_t>((serial_digits >> 8) & 0xFF) << 8) |
                       (static_cast<uint32_t>(serial_digits & 0xFF) << 16) |
                       (static_cast<uint32_t>(seed_byte) << 24);

    return addr;
  }
};

/* ================================================================
 * RF Frame Types — header byte [1] values
 * ================================================================ */
enum class RfFrameType : uint8_t {
  BEACON = 0x08,       // Mode 3: RF AES-CCM encrypted beacon/wake-up
  DATA = 0x46,         // Mode 1: DLMS-wrapped data (DLMS-level security, no RF AES-CCM)
  ACK = 0x44,          // Mode 2: RF AES-CCM encrypted ACK
  PLAIN_DATA = 0x00,   // Mode 6: config/key exchange on secondary channel (no encryption)
};

/* ================================================================
 * RF Frame Header Offsets
 * ================================================================ */
static constexpr size_t RF_HDR_LENGTH = 0;       // [0] total_size - 1
static constexpr size_t RF_HDR_FLAGS = 1;        // [1] header flags / frame type
static constexpr size_t RF_HDR_ADDR = 2;         // [2-9] 8-byte destination address
static constexpr size_t RF_HDR_ADDR_LEN = 8;
static constexpr size_t RF_HDR_SRC_ID = 10;      // [10] source ID
static constexpr size_t RF_HDR_CHANNEL = 11;     // [11] channel ID (0x7A or 0x8A)
static constexpr size_t RF_HDR_TYPE = 12;        // [12] frame sub-type
static constexpr size_t RF_HDR_SYNC = 13;        // [13] sync/protocol byte
static constexpr size_t RF_HDR_SEQ = 14;         // [14] sequence number
static constexpr size_t RF_HDR_SIZE = 15;        // Header is 15 bytes total

/* Channel IDs */
static constexpr uint8_t CHANNEL_PRIMARY = 0x7A;
static constexpr uint8_t CHANNEL_SECONDARY = 0x8A;

/* ================================================================
 * AES-CCM Constants
 * ================================================================ */
static constexpr size_t AES_KEY_SIZE = 16;
static constexpr size_t AES_NONCE_SIZE = 12;     // 8-byte addr + 4-byte counter
static constexpr size_t AES_TAG_SIZE = 12;       // MIC tag length
static constexpr uint8_t AES_CCM_FLAGS = 0x29;   // CCM B0 byte[1] from firmware: Adata=0, t=12 encoded, q=2

/* ================================================================
 * CRC-16/DNP Constants
 * ================================================================ */
static constexpr uint16_t CRC16_POLY = 0x3D65;
static constexpr uint16_t CRC16_INIT = 0x0000;
static constexpr uint16_t CRC16_XOROUT = 0xFFFF;

/* ================================================================
 * Nartis Proprietary Protocol Tags
 *
 * Firmware uses a proprietary protocol, NOT standard DLMS AARQ/AARE.
 * Each read request is self-contained with 0xC0/0xC1 wrapper.
 * See firmware nartis_build_read (0x8CAC) and apdu_builder_dispatch (0xA938).
 * ================================================================ */
static constexpr uint8_t NARTIS_TAG_READ_REQ = 0xC0;    // Read request (firmware dispatch 0xC4)
static constexpr uint8_t NARTIS_TAG_READ_RSP = 0xC1;    // Read response / ACK (firmware dispatch 0xC5)
static constexpr uint8_t NARTIS_TAG_ACTION_REQ = 0xC3;  // Action/write request (firmware dispatch 0xC7)

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
struct DlmsValue {
  enum Type : uint8_t {
    NONE = 0,
    FLOAT_VAL,
    INT_VAL,
    UINT_VAL,
    STRING_VAL,
  };

  Type type{NONE};
  float float_val{0.0f};
  int32_t int_val{0};
  uint32_t uint_val{0};
  char str_val[64]{};

  bool has_value() const { return type != NONE; }
};

/* ================================================================
 * Buffer sizes
 * ================================================================ */
static constexpr size_t MAX_RF_FRAME_SIZE = 512;
static constexpr size_t MAX_DLMS_APDU_SIZE = 400;

}  // namespace esphome::nartis_rf_meter
