/*
 * Nartis RF Data Layer — Implementation
 *
 * CRC-16/DNP per-block framing, AES-128-CCM encryption, and RF packet assembly.
 */

#include "rf_data.h"
#include <cstring>

#include "esphome/core/log.h"
#include <mbedtls/ccm.h>

namespace esphome::nartis_rf_meter {

static const char *const TAG = "rf_data";

/* ================================================================
 * CRC-16/DNP Lookup Table
 *
 * Polynomial: 0x3D65 (CRC-16/EN-13757 = IEC 62056-46)
 * Init: 0x0000, XorOut: 0xFFFF, No reflection
 * Storage: big-endian (high byte first)
 *
 * Generated from firmware flash.bin at 0xB77C (256 entries).
 * ================================================================ */

// clang-format off
const uint16_t RfDataLayer::crc_table_[256] = {
    0x0000, 0x3D65, 0x7ACA, 0x47AF, 0xF594, 0xC8F1, 0x8F5E, 0xB23B,
    0xD64D, 0xEB28, 0xAC87, 0x91E2, 0x23D9, 0x1EBC, 0x5913, 0x6476,
    0x91FF, 0xAC9A, 0xEB35, 0xD650, 0x646B, 0x590E, 0x1EA1, 0x23C4,
    0x47B2, 0x7AD7, 0x3D78, 0x001D, 0xB226, 0x8F43, 0xC8EC, 0xF589,
    0x1E9B, 0x23FE, 0x6451, 0x5934, 0xEB0F, 0xD66A, 0x91C5, 0xACA0,
    0xC8D6, 0xF5B3, 0xB21C, 0x8F79, 0x3D42, 0x0027, 0x4788, 0x7AED,
    0x8F64, 0xB201, 0xF5AE, 0xC8CB, 0x7AF0, 0x4795, 0x003A, 0x3D5F,
    0x5929, 0x644C, 0x23E3, 0x1E86, 0xACBD, 0x91D8, 0xD677, 0xEB12,
    0x3D36, 0x0053, 0x47FC, 0x7A99, 0xC8A2, 0xF5C7, 0xB268, 0x8F0D,
    0xEB7B, 0xD61E, 0x91B1, 0xACD4, 0x1EEF, 0x238A, 0x6425, 0x5940,
    0xACC9, 0x91AC, 0xD603, 0xEB66, 0x595D, 0x6438, 0x2397, 0x1EF2,
    0x7A84, 0x47E1, 0x004E, 0x3D2B, 0x8F10, 0xB275, 0xF5DA, 0xC8BF,
    0x23AD, 0x1EC8, 0x5967, 0x6402, 0xD639, 0xEB5C, 0xACF3, 0x9196,
    0xF5E0, 0xC885, 0x8F2A, 0xB24F, 0x0074, 0x3D11, 0x7ABE, 0x47DB,
    0xB252, 0x8F37, 0xC898, 0xF5FD, 0x47C6, 0x7AA3, 0x3D0C, 0x0069,
    0x641F, 0x597A, 0x1ED5, 0x23B0, 0x918B, 0xACEE, 0xEB41, 0xD624,
    0x7A6C, 0x4709, 0x00A6, 0x3DC3, 0x8FF8, 0xB29D, 0xF532, 0xC857,
    0xAC21, 0x9144, 0xD6EB, 0xEB8E, 0x59B5, 0x64D0, 0x237F, 0x1E1A,
    0xEB93, 0xD6F6, 0x9159, 0xAC3C, 0x1E07, 0x2362, 0x64CD, 0x59A8,
    0x3DDE, 0x00BB, 0x4714, 0x7A71, 0xC84A, 0xF52F, 0xB280, 0x8FE5,
    0x64F7, 0x5992, 0x1E3D, 0x2358, 0x9163, 0xAC06, 0xEBA9, 0xD6CC,
    0xB2BA, 0x8FDF, 0xC870, 0xF515, 0x472E, 0x7A4B, 0x3DE4, 0x0081,
    0xF508, 0xC86D, 0x8FC2, 0xB2A7, 0x009C, 0x3DF9, 0x7A56, 0x4733,
    0x2345, 0x1E20, 0x598F, 0x64EA, 0xD6D1, 0xEBB4, 0xAC1B, 0x917E,
    0x475A, 0x7A3F, 0x3D90, 0x00F5, 0xB2CE, 0x8FAB, 0xC804, 0xF561,
    0x9117, 0xAC72, 0xEBDD, 0xD6B8, 0x6483, 0x59E6, 0x1E49, 0x232C,
    0xD6A5, 0xEBC0, 0xAC6F, 0x910A, 0x2331, 0x1E54, 0x59FB, 0x649E,
    0x00E8, 0x3D8D, 0x7A22, 0x4747, 0xF57C, 0xC819, 0x8FB6, 0xB2D3,
    0x59C1, 0x64A4, 0x230B, 0x1E6E, 0xAC55, 0x9130, 0xD69F, 0xEBFA,
    0x8F8C, 0xB2E9, 0xF546, 0xC823, 0x7A18, 0x477D, 0x00D2, 0x3DB7,
    0xC83E, 0xF55B, 0xB2F4, 0x8F91, 0x3DAA, 0x00CF, 0x4760, 0x7A05,
    0x1E73, 0x2316, 0x64B9, 0x59DC, 0xEBE7, 0xD682, 0x912D, 0xAC48,
};
// clang-format on

/* ================================================================
 * CRC-16/DNP Calculation
 * ================================================================ */

uint16_t RfDataLayer::crc16_calc(const uint8_t *data, size_t len) {
  uint16_t crc = CRC16_INIT;
  for (size_t i = 0; i < len; i++) {
    crc = (crc << 8) ^ crc_table_[((crc >> 8) ^ data[i]) & 0xFF];
  }
  return static_cast<uint16_t>(~crc);  // XorOut = 0xFFFF
}

bool RfDataLayer::crc16_verify(const uint8_t *data, size_t len) {
  if (len < 3) return false;
  uint16_t computed = crc16_calc(data, len - 2);
  uint16_t stored = (static_cast<uint16_t>(data[len - 2]) << 8) | data[len - 1];  // big-endian
  return computed == stored;
}

/* ================================================================
 * CRC Block Framing — Plaintext Mode
 *
 * Block 0: 10B header + 2B CRC
 * Block 1..N-1: 16B data + 2B CRC each
 * Block N: remainder + 2B CRC
 * ================================================================ */

size_t RfDataLayer::crc_block_insert_plain(uint8_t *out, size_t out_max,
                                           const uint8_t *data, size_t len) {
  size_t out_pos = 0;
  size_t in_pos = 0;

  // Block 0: first 10 bytes + CRC
  size_t block_size = (len >= 10) ? 10 : len;
  if (out_pos + block_size + 2 > out_max) return 0;
  memcpy(out + out_pos, data + in_pos, block_size);
  uint16_t crc = crc16_calc(data + in_pos, block_size);
  out[out_pos + block_size] = static_cast<uint8_t>(crc >> 8);      // high byte
  out[out_pos + block_size + 1] = static_cast<uint8_t>(crc & 0xFF); // low byte
  out_pos += block_size + 2;
  in_pos += block_size;

  // Remaining blocks: 16 bytes each + CRC
  while (in_pos < len) {
    block_size = (len - in_pos >= 16) ? 16 : (len - in_pos);
    if (out_pos + block_size + 2 > out_max) return 0;
    memcpy(out + out_pos, data + in_pos, block_size);
    crc = crc16_calc(data + in_pos, block_size);
    out[out_pos + block_size] = static_cast<uint8_t>(crc >> 8);
    out[out_pos + block_size + 1] = static_cast<uint8_t>(crc & 0xFF);
    out_pos += block_size + 2;
    in_pos += block_size;
  }

  return out_pos;
}

size_t RfDataLayer::crc_block_insert_encrypted(uint8_t *out, size_t out_max,
                                               const uint8_t *data, size_t len) {
  if (len < 129) {
    // Short encrypted: single block + CRC
    if (len + 2 > out_max) return 0;
    memcpy(out, data, len);
    uint16_t crc = crc16_calc(data, len);
    out[len] = static_cast<uint8_t>(crc >> 8);
    out[len + 1] = static_cast<uint8_t>(crc & 0xFF);
    return len + 2;
  } else {
    // Long encrypted: 126B + CRC + remainder + CRC
    size_t out_pos = 0;

    // Part 1: first 126 bytes
    if (126 + 2 > out_max) return 0;
    memcpy(out, data, 126);
    uint16_t crc = crc16_calc(data, 126);
    out[126] = static_cast<uint8_t>(crc >> 8);
    out[127] = static_cast<uint8_t>(crc & 0xFF);
    out_pos = 128;

    // Part 2: remaining bytes
    size_t remaining = len - 126;
    if (out_pos + remaining + 2 > out_max) return 0;
    memcpy(out + out_pos, data + 126, remaining);
    crc = crc16_calc(data + 126, remaining);
    out[out_pos + remaining] = static_cast<uint8_t>(crc >> 8);
    out[out_pos + remaining + 1] = static_cast<uint8_t>(crc & 0xFF);
    out_pos += remaining + 2;

    return out_pos;
  }
}

int RfDataLayer::crc_block_strip_plain(uint8_t *data, size_t len) {
  // Verify and strip CRC blocks, compact in-place
  size_t in_pos = 0;
  size_t out_pos = 0;

  // Block 0: 10B + 2B CRC = 12B
  if (len < 12) return -1;
  if (!crc16_verify(data + in_pos, 12)) {
    ESP_LOGW(TAG, "CRC error in block 0");
    return -1;
  }
  // Data already in place for block 0 (first 10 bytes)
  out_pos = 10;
  in_pos = 12;

  // Remaining blocks: up to 16B + 2B CRC = 18B each
  while (in_pos < len) {
    size_t remaining = len - in_pos;
    // Minimum block: 1B data + 2B CRC = 3B
    if (remaining < 3) {
      ESP_LOGW(TAG, "Truncated block at offset %d", (int) in_pos);
      return -1;
    }

    // Determine block data size (max 16, or whatever remains minus CRC)
    size_t block_data_size;
    if (remaining >= 18) {
      block_data_size = 16;
    } else {
      block_data_size = remaining - 2;
    }

    if (!crc16_verify(data + in_pos, block_data_size + 2)) {
      ESP_LOGW(TAG, "CRC error at block offset %d", (int) in_pos);
      return -1;
    }

    // Compact: move data to remove CRC gaps
    if (out_pos != in_pos) {
      memmove(data + out_pos, data + in_pos, block_data_size);
    }
    out_pos += block_data_size;
    in_pos += block_data_size + 2;
  }

  return static_cast<int>(out_pos);
}

int RfDataLayer::crc_block_strip_encrypted(uint8_t *data, size_t len) {
  if (len < 3) return -1;

  // Check if short (total with CRC < 131) or long
  if (len < 131) {
    // Short: single CRC at end
    if (!crc16_verify(data, len)) {
      ESP_LOGW(TAG, "CRC error in encrypted short block");
      return -1;
    }
    return static_cast<int>(len - 2);
  } else {
    // Long: 126B + 2B CRC + remaining + 2B CRC
    if (len < 131) return -1;  // minimum: 126+2+1+2 = 131

    // Verify first CRC (126 bytes)
    if (!crc16_verify(data, 128)) {
      ESP_LOGW(TAG, "CRC error in encrypted long block part 1");
      return -1;
    }

    // Verify second CRC (remaining bytes after offset 128)
    size_t part2_total = len - 128;
    if (!crc16_verify(data + 128, part2_total)) {
      ESP_LOGW(TAG, "CRC error in encrypted long block part 2");
      return -1;
    }

    // Strip middle CRC: shift data[128..] left by 2
    size_t part2_data = part2_total - 2;
    memmove(data + 126, data + 128, part2_data);

    return static_cast<int>(126 + part2_data);
  }
}

/* ================================================================
 * AES-128-CCM Encryption/Decryption
 * ================================================================ */

void RfDataLayer::build_nonce(uint8_t nonce[AES_NONCE_SIZE], uint32_t counter) const {
  // Nonce = 8-byte address + 4-byte counter (big-endian)
  uint8_t addr_bytes[8];
  address_.to_bytes(addr_bytes);
  memcpy(nonce, addr_bytes, 8);
  nonce[8] = static_cast<uint8_t>((counter >> 24) & 0xFF);
  nonce[9] = static_cast<uint8_t>((counter >> 16) & 0xFF);
  nonce[10] = static_cast<uint8_t>((counter >> 8) & 0xFF);
  nonce[11] = static_cast<uint8_t>(counter & 0xFF);
}

size_t RfDataLayer::aes_ccm_encrypt(uint8_t *data, size_t data_len, size_t buf_max,
                                    uint32_t counter) {
  if (data_len + AES_TAG_SIZE > buf_max) {
    ESP_LOGE(TAG, "Buffer too small for AES-CCM encrypt");
    return 0;
  }

  uint8_t nonce[AES_NONCE_SIZE];
  build_nonce(nonce, counter);

  mbedtls_ccm_context ctx;
  mbedtls_ccm_init(&ctx);

  int ret = mbedtls_ccm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, aes_key_, 128);
  if (ret != 0) {
    ESP_LOGE(TAG, "mbedtls_ccm_setkey failed: %d", ret);
    mbedtls_ccm_free(&ctx);
    return 0;
  }

  // Encrypt in-place: plaintext → ciphertext, tag appended
  uint8_t tag[AES_TAG_SIZE];
  ret = mbedtls_ccm_encrypt_and_tag(&ctx, data_len,
                                     nonce, AES_NONCE_SIZE,
                                     nullptr, 0,   // no additional data
                                     data, data,   // in-place
                                     tag, AES_TAG_SIZE);
  mbedtls_ccm_free(&ctx);

  if (ret != 0) {
    ESP_LOGE(TAG, "mbedtls_ccm_encrypt_and_tag failed: %d", ret);
    return 0;
  }

  // Append tag
  memcpy(data + data_len, tag, AES_TAG_SIZE);
  return data_len + AES_TAG_SIZE;
}

int RfDataLayer::aes_ccm_decrypt(uint8_t *data, size_t data_len, uint32_t counter) {
  if (data_len < AES_TAG_SIZE) {
    ESP_LOGE(TAG, "Data too short for AES-CCM decrypt");
    return -1;
  }

  size_t ct_len = data_len - AES_TAG_SIZE;

  uint8_t nonce[AES_NONCE_SIZE];
  build_nonce(nonce, counter);

  mbedtls_ccm_context ctx;
  mbedtls_ccm_init(&ctx);

  int ret = mbedtls_ccm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, aes_key_, 128);
  if (ret != 0) {
    ESP_LOGE(TAG, "mbedtls_ccm_setkey failed: %d", ret);
    mbedtls_ccm_free(&ctx);
    return -1;
  }

  // Tag is at data + ct_len
  ret = mbedtls_ccm_auth_decrypt(&ctx, ct_len,
                                  nonce, AES_NONCE_SIZE,
                                  nullptr, 0,   // no additional data
                                  data, data,   // in-place
                                  data + ct_len, AES_TAG_SIZE);
  mbedtls_ccm_free(&ctx);

  if (ret != 0) {
    ESP_LOGW(TAG, "AES-CCM auth failed (ret=%d) — tampered or wrong key/counter", ret);
    return -1;
  }

  return static_cast<int>(ct_len);
}

/* ================================================================
 * Address & Key Configuration
 * ================================================================ */

void RfDataLayer::set_address(const RfAddress &addr) {
  address_ = addr;
}

void RfDataLayer::set_aes_key(const uint8_t key[AES_KEY_SIZE]) {
  memcpy(aes_key_, key, AES_KEY_SIZE);
}

/* ================================================================
 * Frame Building (TX Path)
 *
 * Frame layout:
 * [0]     Length (total - 1)
 * [1]     Header flags (frame type)
 * [2-9]   8-byte destination address
 * [10]    Source ID
 * [11]    Channel ID
 * [12]    Frame sub-type
 * [13]    Sync byte
 * [14]    Sequence number
 * [15+]   Payload with CRC blocks (and optional encryption wrapper)
 *
 * For encrypted frames, payload bytes [15+] contain:
 * [15]    Payload length byte
 * [16]    Reserved (0x00)
 * [17-20] Frame counter (4 bytes, big-endian)
 * [21+]   AES-CCM encrypted data + 12B MIC tag
 * ================================================================ */

size_t RfDataLayer::build_frame(uint8_t *out, size_t out_max,
                                RfFrameType type, uint8_t src_id,
                                uint8_t sequence,
                                const uint8_t *payload, size_t payload_len) {
  if (out_max < RF_HDR_SIZE + 2) return 0;  // At minimum header + CRC

  // RF-level AES-CCM is used for ACK (Mode 2) and BEACON (Mode 3).
  // DATA (Mode 1) uses DLMS-level security; PLAIN_DATA (Mode 6) is unencrypted.
  // Verified against firmware check_framing_mode (0xB71A): returns 1 (encrypt) when header != 0x46 AND channel != 0x8A.
  bool encrypted = (type == RfFrameType::ACK || type == RfFrameType::BEACON);
  uint8_t channel_id = (type == RfFrameType::PLAIN_DATA) ? CHANNEL_SECONDARY : CHANNEL_PRIMARY;

  // Build header
  // [0] will be filled with total length at the end
  out[RF_HDR_FLAGS] = static_cast<uint8_t>(type);
  address_.to_bytes(out + RF_HDR_ADDR);
  out[RF_HDR_SRC_ID] = src_id;
  out[RF_HDR_CHANNEL] = channel_id;
  out[RF_HDR_TYPE] = 0x00;   // Sub-type (filled by caller if needed)
  out[RF_HDR_SYNC] = 0x00;   // Sync byte
  out[RF_HDR_SEQ] = sequence;

  size_t total;

  if (encrypted) {
    // Build encrypted payload area
    // Work in a temp buffer: [payload_len_byte][0x00][counter_4B][encrypted_payload+tag]
    uint8_t work[MAX_RF_FRAME_SIZE];
    size_t work_len = 0;

    // Encrypt the DLMS payload
    if (payload_len > sizeof(work) - 6 - AES_TAG_SIZE) return 0;
    memcpy(work + 6, payload, payload_len);
    size_t enc_len = aes_ccm_encrypt(work + 6, payload_len, sizeof(work) - 6, frame_counter_);
    if (enc_len == 0) return 0;

    // Prepend: payload length + reserved + frame counter
    work[0] = static_cast<uint8_t>(enc_len & 0xFF);
    work[1] = 0x00;
    work[2] = static_cast<uint8_t>((frame_counter_ >> 24) & 0xFF);
    work[3] = static_cast<uint8_t>((frame_counter_ >> 16) & 0xFF);
    work[4] = static_cast<uint8_t>((frame_counter_ >> 8) & 0xFF);
    work[5] = static_cast<uint8_t>(frame_counter_ & 0xFF);
    work_len = 6 + enc_len;

    // Apply CRC block framing to encrypted payload
    size_t crc_len = crc_block_insert_encrypted(out + RF_HDR_SIZE, out_max - RF_HDR_SIZE,
                                                work, work_len);
    if (crc_len == 0) return 0;
    total = RF_HDR_SIZE + crc_len;

    increment_frame_counter();
  } else {
    // Plaintext: header (first 10 bytes = header part after length byte, then payload)
    // CRC blocks wrap the entire content after the length byte
    // Build the full content first
    uint8_t work[MAX_RF_FRAME_SIZE];
    size_t work_len = 0;

    // Copy header bytes [1..14] as block 0 source (10 bytes = bytes 1-10 of header)
    // Then payload follows
    memcpy(work, out + 1, RF_HDR_SIZE - 1);  // 14 bytes of header (excluding length)
    work_len = RF_HDR_SIZE - 1;

    if (payload_len > 0) {
      memcpy(work + work_len, payload, payload_len);
      work_len += payload_len;
    }

    // Apply CRC block framing to plaintext
    size_t crc_len = crc_block_insert_plain(out + 1, out_max - 1, work, work_len);
    if (crc_len == 0) return 0;
    total = 1 + crc_len;  // length byte + CRC-framed content
  }

  // Fill length byte
  out[RF_HDR_LENGTH] = static_cast<uint8_t>(total - 1);

  return total;
}

/* ================================================================
 * Frame Parsing (RX Path)
 * ================================================================ */

int RfDataLayer::parse_frame(const uint8_t *in, size_t in_len,
                             uint8_t *payload_out, size_t payload_max,
                             RfFrameType *type_out) {
  if (in_len < RF_HDR_SIZE + 2) {
    ESP_LOGW(TAG, "Frame too short: %d bytes", (int) in_len);
    return -1;
  }

  // Validate length byte
  uint8_t declared_len = in[RF_HDR_LENGTH];
  if (declared_len + 1 > in_len) {
    ESP_LOGW(TAG, "Declared length %d exceeds buffer %d", declared_len + 1, (int) in_len);
    return -1;
  }

  // Extract frame type
  auto frame_type = static_cast<RfFrameType>(in[RF_HDR_FLAGS]);
  if (type_out) *type_out = frame_type;

  // Verify destination address matches ours
  RfAddress rx_addr = RfAddress::from_bytes(in + RF_HDR_ADDR);
  if (!(rx_addr == address_)) {
    ESP_LOGD(TAG, "Address mismatch — discarding frame");
    return -1;
  }

  // RF-level AES-CCM for ACK and BEACON only (matches firmware check_framing_mode (0xB71A))
  bool encrypted = (frame_type == RfFrameType::ACK || frame_type == RfFrameType::BEACON);

  if (encrypted) {
    // Strip CRC blocks from encrypted payload
    size_t payload_start = RF_HDR_SIZE;
    size_t payload_total = declared_len + 1 - RF_HDR_SIZE;

    uint8_t work[MAX_RF_FRAME_SIZE];
    if (payload_total > sizeof(work)) return -1;
    memcpy(work, in + payload_start, payload_total);

    int stripped_len = crc_block_strip_encrypted(work, payload_total);
    if (stripped_len < 0) return -1;

    // Parse encryption wrapper: [len][0x00][counter_4B][ciphertext+tag]
    if (stripped_len < 6) {
      ESP_LOGW(TAG, "Encrypted payload too short: %d", stripped_len);
      return -1;
    }

    // uint8_t enc_payload_len = work[0];  // informational
    uint32_t rx_counter = (static_cast<uint32_t>(work[2]) << 24) |
                          (static_cast<uint32_t>(work[3]) << 16) |
                          (static_cast<uint32_t>(work[4]) << 8) |
                          static_cast<uint32_t>(work[5]);

    // Decrypt
    int dec_len = aes_ccm_decrypt(work + 6, stripped_len - 6, rx_counter);
    if (dec_len < 0) return -1;

    if (static_cast<size_t>(dec_len) > payload_max) return -1;
    memcpy(payload_out, work + 6, dec_len);
    return dec_len;

  } else {
    // Plaintext: strip CRC blocks from entire content after length byte
    uint8_t work[MAX_RF_FRAME_SIZE];
    size_t content_len = declared_len;  // bytes after length byte
    if (content_len > sizeof(work)) return -1;
    memcpy(work, in + 1, content_len);

    int stripped_len = crc_block_strip_plain(work, content_len);
    if (stripped_len < 0) return -1;

    // First 14 bytes are header [1..14], payload starts at offset 14
    if (stripped_len <= (int)(RF_HDR_SIZE - 1)) {
      // No payload, just header
      return 0;
    }

    int payload_len = stripped_len - (RF_HDR_SIZE - 1);
    if (static_cast<size_t>(payload_len) > payload_max) return -1;
    memcpy(payload_out, work + (RF_HDR_SIZE - 1), payload_len);
    return payload_len;
  }
}

/* ================================================================
 * Channel Selection
 * ================================================================ */

uint8_t RfDataLayer::select_best_channel(const int8_t rssi[4]) {
  // Firmware (rssi_channel_select / 0x134a4) picks the QUIETEST channel
  // (lowest RSSI = least interference) for TX. Only channels 1-3 are
  // candidates — channel 0 is scanned but never selected.
  uint8_t best = 1;
  int8_t best_rssi = rssi[1];

  for (uint8_t i = 2; i < 4; i++) {
    if (rssi[i] < best_rssi) {
      best_rssi = rssi[i];
      best = i;
    }
  }

  return best;
}

}  // namespace esphome::nartis_rf_meter
