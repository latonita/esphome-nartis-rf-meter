/*
 * Nartis RF Data Layer — Implementation
 *
 * CRC-16/DNP framing (single + dual modes), AES-128-CCM, packet assembly.
 *
 * Verified against:
 *   - 104 outgoing SPI-captured frames (fifo.txt, Pair B)
 *   - 104 incoming SPI-captured frames (Pair B meter responses)
 *   - Ghidra decompile of rf_build_frame, rf_parse_frame, crc_block_insert,
 *     crc_block_deframe, derive_rf_address
 */

#include "rf_data.h"
#include <cstring>
#include <algorithm>

#include "esphome/core/log.h"
#include "esphome/core/defines.h"

#ifdef USE_ESP32
  #include <mbedtls/ccm.h>
#elif defined(USE_ESP8266)
  // ESP8266 Arduino core ships with BearSSL — use it for AES-128-CCM here.
  // Note: per FRAME_HEADER_SPEC.md §8 the firmware's actual cipher is
  // AES-128-GCM (not CCM) with AAD = [01 29 L 00]. The current component
  // implements CCM-without-AAD, which is preserved here for parity with
  // the ESP32 build. A GCM-with-AAD migration is a separate TODO.
  #include <bearssl/bearssl_block.h>
  #include <bearssl/bearssl_aead.h>
#endif

namespace esphome::nartis_rf_meter {

static const char *const TAG = "rf_data";

/* ================================================================
 * CRC-16/DNP Lookup Table
 * Polynomial 0x3D65, init 0x0000, no reflection, xorout 0xFFFF.
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
 * CRC-16/DNP
 * ================================================================ */

uint16_t RfDataLayer::crc16_calc(const uint8_t *data, size_t len) {
  uint16_t crc = CRC16_INIT;
  for (size_t i = 0; i < len; i++) {
    crc = (crc << 8) ^ crc_table_[((crc >> 8) ^ data[i]) & 0xFF];
  }
  return static_cast<uint16_t>(~crc);
}

bool RfDataLayer::crc16_verify(const uint8_t *data, size_t len) {
  if (len < 3) return false;
  uint16_t computed = crc16_calc(data, len - 2);
  uint16_t stored = (static_cast<uint16_t>(data[len - 2]) << 8) | data[len - 1];
  return computed == stored;
}

/* ================================================================
 * CRC Insert / Strip  —  matches firmware crc_block_insert (0xB9B2)
 * and crc_block_deframe (0xBE10) for the encrypted-path mode (param_3 != 0).
 *
 * Two on-air layouts:
 *   A) body_size <= 126: single CRC at end. Frame size = body_size + 2.
 *   B) body_size  > 126: CRC1 at offset 0x7E over [0..0x7D], body bytes
 *      [0x7E..body_size-1] shifted right by 2, CRC2 appended at end over
 *      [0x80..frame_size-3]. Frame size = body_size + 4.
 * ================================================================ */

size_t RfDataLayer::crc_insert(uint8_t *buf, size_t body_size, size_t out_max) {
  if (body_size <= 126) {
    // Single CRC at end
    if (body_size + 2 > out_max) return 0;
    uint16_t crc = crc16_calc(buf, body_size);
    buf[body_size]     = static_cast<uint8_t>(crc >> 8);
    buf[body_size + 1] = static_cast<uint8_t>(crc & 0xFF);
    return body_size + 2;
  } else {
    // Dual CRC mode
    if (body_size + 4 > out_max) return 0;
    // Shift bytes [0x7E..body_size-1] right by 2 to make room for CRC1.
    // Use memmove for overlapping region.
    size_t tail_size = body_size - CRC1_OFFSET;
    memmove(buf + CRC1_OFFSET + 2, buf + CRC1_OFFSET, tail_size);

    // CRC1 over [0..0x7D]
    uint16_t crc1 = crc16_calc(buf, CRC1_OFFSET);
    buf[CRC1_OFFSET]     = static_cast<uint8_t>(crc1 >> 8);
    buf[CRC1_OFFSET + 1] = static_cast<uint8_t>(crc1 & 0xFF);

    // CRC2 over [0x80..0x80 + tail_size - 1]
    uint16_t crc2 = crc16_calc(buf + 0x80, tail_size);
    buf[0x80 + tail_size]     = static_cast<uint8_t>(crc2 >> 8);
    buf[0x80 + tail_size + 1] = static_cast<uint8_t>(crc2 & 0xFF);

    return 0x80 + tail_size + 2;  // = body_size + 4
  }
}

int RfDataLayer::crc_strip(uint8_t *buf, size_t frame_size) {
  if (frame_size < 3) return -1;
  if (frame_size <= CRC_DUAL_THRESHOLD) {
    // Single CRC
    if (!crc16_verify(buf, frame_size)) {
      ESP_LOGW(TAG, "CRC fail (single, size=%u)", static_cast<unsigned>(frame_size));
      return -1;
    }
    return static_cast<int>(frame_size - 2);
  } else {
    // Dual CRC
    if (frame_size < 0x82) return -1;  // need at least 0x7E + 2 + 0 + 2
    // Verify CRC1
    uint16_t crc1_obs  = (static_cast<uint16_t>(buf[CRC1_OFFSET]) << 8) | buf[CRC1_OFFSET + 1];
    uint16_t crc1_comp = crc16_calc(buf, CRC1_OFFSET);
    if (crc1_obs != crc1_comp) {
      ESP_LOGW(TAG, "CRC1 fail at 0x7E (got 0x%04X, want 0x%04X)", crc1_obs, crc1_comp);
      return -1;
    }
    // Verify CRC2
    size_t tail_size = frame_size - 0x80 - 2;
    uint16_t crc2_obs  = (static_cast<uint16_t>(buf[frame_size - 2]) << 8) | buf[frame_size - 1];
    uint16_t crc2_comp = crc16_calc(buf + 0x80, tail_size);
    if (crc2_obs != crc2_comp) {
      ESP_LOGW(TAG, "CRC2 fail at end (got 0x%04X, want 0x%04X)", crc2_obs, crc2_comp);
      return -1;
    }
    // Shift bytes [0x80..frame_size-3] left by 2 to drop CRC1
    memmove(buf + CRC1_OFFSET, buf + 0x80, tail_size);
    return static_cast<int>(CRC1_OFFSET + tail_size);  // body_size = frame_size - 4
  }
}

/* ================================================================
 * AES-128-CCM
 * ================================================================ */

void RfDataLayer::build_nonce(uint8_t nonce[AES_NONCE_SIZE], const RfAddress &addr, uint32_t counter) const {
  uint8_t addr_bytes[8];
  addr.to_bytes(addr_bytes);
  memcpy(nonce, addr_bytes, 8);
  nonce[8]  = static_cast<uint8_t>((counter >> 24) & 0xFF);
  nonce[9]  = static_cast<uint8_t>((counter >> 16) & 0xFF);
  nonce[10] = static_cast<uint8_t>((counter >> 8) & 0xFF);
  nonce[11] = static_cast<uint8_t>(counter & 0xFF);
}

size_t RfDataLayer::aes_ccm_encrypt(uint8_t *data, size_t data_len, size_t buf_max, uint32_t counter) {
  if (data_len + AES_TAG_SIZE > buf_max) {
    ESP_LOGE(TAG, "Buffer too small for AES-CCM encrypt");
    return 0;
  }
  uint8_t nonce[AES_NONCE_SIZE];
  build_nonce(nonce, address_, counter);

#ifdef USE_ESP32
  mbedtls_ccm_context ctx;
  mbedtls_ccm_init(&ctx);
  int ret = mbedtls_ccm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, aes_key_, 128);
  if (ret != 0) {
    ESP_LOGE(TAG, "ccm_setkey failed: %d", ret);
    mbedtls_ccm_free(&ctx);
    return 0;
  }
  uint8_t tag[AES_TAG_SIZE];
  ret = mbedtls_ccm_encrypt_and_tag(&ctx, data_len, nonce, AES_NONCE_SIZE,
                                     nullptr, 0, data, data, tag, AES_TAG_SIZE);
  mbedtls_ccm_free(&ctx);
  if (ret != 0) {
    ESP_LOGE(TAG, "ccm_encrypt_and_tag failed: %d", ret);
    return 0;
  }
  memcpy(data + data_len, tag, AES_TAG_SIZE);
  return data_len + AES_TAG_SIZE;
#elif defined(USE_ESP8266)
  // BearSSL CCM (AAD-free, 12-byte tag)
  br_aes_big_ctrcbc_keys aes;
  br_aes_big_ctrcbc_init(&aes, aes_key_, AES_KEY_SIZE);
  br_ccm_context cctx;
  br_ccm_init(&cctx, &aes.vtable);
  br_ccm_reset(&cctx, nonce, AES_NONCE_SIZE, /*aad_len=*/0, data_len, AES_TAG_SIZE);
  br_ccm_aad_inject(&cctx, nullptr, 0);
  br_ccm_flip(&cctx);
  br_ccm_run(&cctx, /*encrypt=*/1, data, data_len);
  uint8_t tag[AES_TAG_SIZE];
  br_ccm_get_tag(&cctx, tag);
  memcpy(data + data_len, tag, AES_TAG_SIZE);
  return data_len + AES_TAG_SIZE;
#else
  (void) nonce;
  ESP_LOGE(TAG, "aes_ccm_encrypt: no AES backend available on this platform.");
  return 0;
#endif
}

int RfDataLayer::aes_ccm_decrypt(uint8_t *data, size_t data_len, uint32_t counter) {
  if (data_len < AES_TAG_SIZE) return -1;
  size_t ct_len = data_len - AES_TAG_SIZE;

  uint8_t nonce[AES_NONCE_SIZE];
  // Note: for incoming meter responses, the AES nonce uses the METER's address,
  // not the CIU's. The firmware's session keeps both. For our component,
  // we currently use the meter_address_ when decrypting RX frames.
  // For nested-encrypted frames inside RX 0x43 payloads, the nonce source is
  // the address embedded in the inner frame header.
  build_nonce(nonce, meter_address_set_ ? meter_address_ : address_, counter);

#ifdef USE_ESP32
  mbedtls_ccm_context ctx;
  mbedtls_ccm_init(&ctx);
  int ret = mbedtls_ccm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, aes_key_, 128);
  if (ret != 0) {
    ESP_LOGE(TAG, "ccm_setkey failed: %d", ret);
    mbedtls_ccm_free(&ctx);
    return -1;
  }
  ret = mbedtls_ccm_auth_decrypt(&ctx, ct_len, nonce, AES_NONCE_SIZE,
                                  nullptr, 0, data, data,
                                  data + ct_len, AES_TAG_SIZE);
  mbedtls_ccm_free(&ctx);
  if (ret != 0) {
    ESP_LOGW(TAG, "ccm_auth_decrypt failed: %d (wrong key/counter or tampered)", ret);
    return -1;
  }
  return static_cast<int>(ct_len);
#elif defined(USE_ESP8266)
  // BearSSL CCM decrypt (AAD-free, 12-byte tag)
  br_aes_big_ctrcbc_keys aes;
  br_aes_big_ctrcbc_init(&aes, aes_key_, AES_KEY_SIZE);
  br_ccm_context cctx;
  br_ccm_init(&cctx, &aes.vtable);
  br_ccm_reset(&cctx, nonce, AES_NONCE_SIZE, /*aad_len=*/0, ct_len, AES_TAG_SIZE);
  br_ccm_aad_inject(&cctx, nullptr, 0);
  br_ccm_flip(&cctx);
  br_ccm_run(&cctx, /*encrypt=*/0, data, ct_len);
  if (br_ccm_check_tag(&cctx, data + ct_len) != 1) {
    ESP_LOGW(TAG, "br_ccm_check_tag failed (wrong key/counter or tampered)");
    return -1;
  }
  return static_cast<int>(ct_len);
#else
  (void) ct_len;
  (void) nonce;
  ESP_LOGW(TAG, "aes_ccm_decrypt: no AES backend available on this platform.");
  return -1;
#endif
}

/* ================================================================
 * Configuration
 * ================================================================ */

void RfDataLayer::set_address(const RfAddress &addr) { address_ = addr; }

void RfDataLayer::set_meter_address(const RfAddress &addr) {
  meter_address_ = addr;
  meter_address_set_ = true;
}

void RfDataLayer::set_aes_key(const uint8_t key[AES_KEY_SIZE]) {
  memcpy(aes_key_, key, AES_KEY_SIZE);
}

/* ================================================================
 * Channel byte: (chan_idx << 6) | (quality & 0x3F)
 *   quality = clamp((rssi_dbm + 130) / 2, 1, 62)
 *
 * Verified against fifo.txt:
 *   -68 dBm → quality 31 → channel 1 gives 0x5F, channel 2 gives 0x9F
 *    -6 dBm → quality 62 (saturated) → channel 1 gives 0x7E
 * ================================================================ */
uint8_t RfDataLayer::compose_channel_byte(uint8_t channel_idx, int8_t rssi_dbm) {
  int q = (static_cast<int>(rssi_dbm) + 130) / 2;
  if (q < 1) q = 1;
  if (q > 62) q = 62;
  return static_cast<uint8_t>(((channel_idx & 0x3) << 6) | (q & 0x3F));
}

/* ================================================================
 * Frame Building (TX, CIU→meter)
 * ================================================================ */

size_t RfDataLayer::build_frame(uint8_t *out, size_t out_max,
                                RfFrameType type, uint8_t sequence,
                                const uint8_t *payload, size_t payload_len) {
  if (out_max < RF_TX_HDR_SIZE + 4) return 0;

  // Mode 1 (DATA): DLMS-level security, no RF AES-CCM.
  // Mode 6 (PLAIN_DATA): plain on secondary channel.
  // Mode 2 (ACK) + Mode 3 (BEACON): RF AES-CCM.
  const bool encrypted   = (type == RfFrameType::ACK || type == RfFrameType::BEACON);
  const uint8_t marker   = (type == RfFrameType::PLAIN_DATA) ? MODE_MARKER_SPECIAL : MODE_MARKER_NORMAL;

  // Header [0..14]
  out[RF_TX_LENGTH]       = 0;  // filled at the end
  out[RF_TX_FLAGS]        = static_cast<uint8_t>(type);
  address_.to_bytes(out + RF_TX_ADDR);
  out[RF_TX_MODE_MARKER]  = marker;
  out[RF_TX_SEQUENCE]     = sequence;
  out[RF_TX_CHANNEL_BYTE] = compose_channel_byte(current_channel_, last_rssi_dbm_);
  out[RF_TX_ENC_FLAG]     = encrypted ? 0x01 : 0x00;
  out[RF_TX_CCM_FLAG]     = encrypted ? AES_CCM_FLAGS : 0x00;

  size_t body_size = RF_TX_HDR_SIZE;

  if (encrypted) {
    if (payload_len > 0xFF) {
      ESP_LOGE(TAG, "Payload too large for enc_data_len byte (%u)", static_cast<unsigned>(payload_len));
      return 0;
    }
    if (RF_TX_ENC_CIPHER + payload_len + AES_TAG_SIZE + 4 > out_max) {
      ESP_LOGE(TAG, "Output buffer too small");
      return 0;
    }
    // Counter management — matches firmware rf_prepare_and_queue @ 0xBC3E:
    //   ctx counter increments ONLY when the sequence byte changes from the
    //   previous TX. Same sequence ⇒ reuse the same counter (e.g. beacon
    //   retransmissions of identical content).
    if (!has_sent_first_tx_ || sequence != last_tx_sequence_) {
      frame_counter_++;
      last_tx_sequence_ = sequence;
      has_sent_first_tx_ = true;
    }
    // [15] enc_data_len, [16] 0, [17..20] counter BE
    out[RF_TX_ENC_DATA_LEN] = static_cast<uint8_t>(payload_len & 0xFF);
    out[RF_TX_ENC_ZERO]     = 0x00;
    out[RF_TX_ENC_COUNTER + 0] = static_cast<uint8_t>((frame_counter_ >> 24) & 0xFF);
    out[RF_TX_ENC_COUNTER + 1] = static_cast<uint8_t>((frame_counter_ >> 16) & 0xFF);
    out[RF_TX_ENC_COUNTER + 2] = static_cast<uint8_t>((frame_counter_ >> 8) & 0xFF);
    out[RF_TX_ENC_COUNTER + 3] = static_cast<uint8_t>(frame_counter_ & 0xFF);

    // Copy plaintext to [21..] and encrypt in place; AES-CCM appends MIC.
    memcpy(out + RF_TX_ENC_CIPHER, payload, payload_len);
    size_t enc_len = aes_ccm_encrypt(out + RF_TX_ENC_CIPHER, payload_len,
                                     out_max - RF_TX_ENC_CIPHER - 4, frame_counter_);
    if (enc_len == 0) return 0;

    body_size = RF_TX_ENC_CIPHER + enc_len;   // = 21 + L + 12
    // NOTE: counter has already been bumped above (only if seq changed).
  } else {
    // Plain body: payload starts at [15].
    if (RF_TX_HDR_SIZE + payload_len + 4 > out_max) {
      ESP_LOGE(TAG, "Output buffer too small");
      return 0;
    }
    memcpy(out + RF_TX_HDR_SIZE, payload, payload_len);
    body_size = RF_TX_HDR_SIZE + payload_len;
  }

  // Insert CRC(s)
  size_t total = crc_insert(out, body_size, out_max);
  if (total == 0) return 0;

  out[RF_TX_LENGTH] = static_cast<uint8_t>(total - 1);
  return total;
}

/* ================================================================
 * Frame Parsing (RX, meter→CIU)
 *
 * Steps:
 *   1. Read length-1 from [0], validate against in_len.
 *   2. CRC strip (in-place on a working copy): single or dual.
 *   3. Validate meter address at [2..9] matches expected.
 *   4. If [11]=[12]=0 → plain branch, return payload at [13..body_end].
 *   5. Else → encrypted branch (rare in observed dataset).
 * ================================================================ */

int RfDataLayer::parse_frame(const uint8_t *in, size_t in_len,
                             uint8_t *payload_out, size_t payload_max,
                             RfFrameType *type_out) {
  if (in_len < RF_RX_HDR_SIZE + 2) {
    ESP_LOGW(TAG, "Frame too short: %u bytes", static_cast<unsigned>(in_len));
    return static_cast<int>(ParseResult::ERR_LEN);
  }
  // Firmware master_rx_handler upper bound: rx_len <= 0x122 (290).
  if (in_len > 0x122) {
    ESP_LOGW(TAG, "Frame too large: %u > 0x122 bytes", static_cast<unsigned>(in_len));
    return static_cast<int>(ParseResult::ERR_LEN);
  }
  size_t frame_size = static_cast<size_t>(in[RF_RX_LENGTH]) + 1;
  if (frame_size > in_len) {
    ESP_LOGW(TAG, "Declared length %u exceeds buffer %u",
             static_cast<unsigned>(frame_size), static_cast<unsigned>(in_len));
    return static_cast<int>(ParseResult::ERR_LEN);
  }

  // Copy to a working buffer (CRC strip is in-place).
  uint8_t work[MAX_RF_FRAME_SIZE];
  if (frame_size > sizeof(work)) {
    ESP_LOGW(TAG, "Frame size %u > working buffer", static_cast<unsigned>(frame_size));
    return static_cast<int>(ParseResult::ERR_LEN);
  }
  memcpy(work, in, frame_size);

  int body_size = crc_strip(work, frame_size);
  if (body_size < 0) return static_cast<int>(ParseResult::ERR_CRC);

  if (type_out) *type_out = static_cast<RfFrameType>(work[RF_RX_TYPE]);

  // Verify meter address at [2..9] matches configured meter address (if set).
  if (meter_address_set_) {
    RfAddress src = RfAddress::from_bytes(work + RF_RX_ADDR);
    if (!(src == meter_address_)) {
      ESP_LOGD(TAG, "RX address mismatch — discarding (got %02X%02X / hash %08X, type %02X)",
               src.device_id & 0xFF, (src.device_id >> 8) & 0xFF,
               static_cast<unsigned>(src.serial_hash), src.device_type);
      return static_cast<int>(ParseResult::ERR_ADDR);
    }
  }

  const uint8_t mode_lo = work[RF_RX_MODE_FLAG_LO];
  const uint8_t mode_hi = work[RF_RX_MODE_FLAG_HI];
  const bool is_plain = (mode_lo == 0 && mode_hi == 0);

  if (is_plain) {
    if (body_size <= static_cast<int>(RF_RX_PAYLOAD)) return 0;  // no payload
    size_t payload_len = static_cast<size_t>(body_size) - RF_RX_PAYLOAD;
    if (payload_len > payload_max) {
      ESP_LOGW(TAG, "Payload too large: %u > %u", static_cast<unsigned>(payload_len),
               static_cast<unsigned>(payload_max));
      return static_cast<int>(ParseResult::ERR_LEN);
    }
    memcpy(payload_out, work + RF_RX_PAYLOAD, payload_len);
    return static_cast<int>(payload_len);
  } else {
    // Encrypted RX (per firmware rf_parse_frame encrypted branch).
    // [13] enc_data_len, [14] must be 0, [15..18] counter BE, [19..] cipher+MIC.
    if (body_size < 19 + AES_TAG_SIZE) {
      ESP_LOGW(TAG, "Encrypted RX body too short: %d", body_size);
      return static_cast<int>(ParseResult::ERR_LEN);
    }
    const uint8_t enc_data_len = work[13];
    if (work[14] != 0) {
      ESP_LOGW(TAG, "Encrypted RX: byte[14]=0x%02X (expected 0)", work[14]);
      return static_cast<int>(ParseResult::ERR_MUSTBEZERO);
    }
    uint32_t counter = (static_cast<uint32_t>(work[15]) << 24) |
                       (static_cast<uint32_t>(work[16]) << 16) |
                       (static_cast<uint32_t>(work[17]) << 8) |
                       static_cast<uint32_t>(work[18]);

    // Firmware exact length math: enc_data_len + must_be_zero + 0x1F == parse_len
    // ⇒ enc_data_len + 0 + 0x1F == body_size  (since body_size is post-CRC parse_len).
    if (static_cast<size_t>(enc_data_len) + 0x1F != static_cast<size_t>(body_size)) {
      ESP_LOGW(TAG, "Encrypted RX length math: L+0x1F (%u) != body_size (%d)",
               static_cast<unsigned>(enc_data_len + 0x1F), body_size);
      return static_cast<int>(ParseResult::ERR_LEN_MATH);
    }

    // Replay protection: reject any counter ≤ the highest counter ever accepted.
    // Firmware frame_counter_validate @ 0xC078 returns 9 (replay) on this case.
    if (counter <= last_rx_counter_) {
      ESP_LOGW(TAG, "Encrypted RX replay rejected: counter 0x%08X ≤ last 0x%08X",
               static_cast<unsigned>(counter), static_cast<unsigned>(last_rx_counter_));
      return static_cast<int>(ParseResult::ERR_REPLAY);
    }

    int dec_len = aes_ccm_decrypt(work + 19, enc_data_len + AES_TAG_SIZE, counter);
    if (dec_len < 0) return static_cast<int>(ParseResult::ERR_MIC);
    if (static_cast<size_t>(dec_len) > payload_max)
      return static_cast<int>(ParseResult::ERR_LEN);

    // Accept: bump the high-water-mark counter so future replays are rejected.
    last_rx_counter_ = counter;

    memcpy(payload_out, work + 19, dec_len);
    return dec_len;
  }
}

/* ================================================================
 * Nested encrypted frame (inside RX 0x43 plain payload)
 *
 * Layout (verified on all RX 0x43 frames in dataset):
 *   [0..1]   2 bytes prefix (last 2 bytes of CIU serial_hash typically)
 *   [2..3]   CD 2C
 *   [4..5]   50 25 (CIU's group/type)
 *   [6..7]   sub-sequence (02 1A / 02 18 / 02 19 ...)
 *   [8]      0x01 (enc_flag)
 *   [9]      0x29 (ccm_flag)
 *   [10]     enc_data_len = L'
 *   [11]     0x00
 *   [12..15] counter (BE u32)
 *   [16..15+L']    inner ciphertext
 *   [16+L'..27+L'] inner MIC (12 bytes)
 *
 * Nonce: per firmware, uses an address derived from the embedded prefix +
 * CIU constants. For now we use the configured CIU address (set_address)
 * as the nonce prefix — works for sessions where the meter responds to
 * a known CIU.
 * ================================================================ */
int RfDataLayer::parse_nested_encrypted(const uint8_t *payload, size_t payload_len,
                                        uint8_t *plain_out, size_t plain_max) {
  if (payload_len < 16 + AES_TAG_SIZE) {
    ESP_LOGW(TAG, "Nested frame too short: %u", static_cast<unsigned>(payload_len));
    return static_cast<int>(ParseResult::ERR_LEN);
  }
  if (payload[8] != 0x01 || payload[9] != AES_CCM_FLAGS) {
    ESP_LOGW(TAG, "Nested frame: enc_flag/ccm = 0x%02X 0x%02X (expected 0x01 0x29)",
             payload[8], payload[9]);
    return static_cast<int>(ParseResult::ERR_LEN);
  }
  const uint8_t enc_data_len = payload[10];
  if (payload[11] != 0) {
    ESP_LOGW(TAG, "Nested frame: byte[11]=0x%02X (expected 0)", payload[11]);
    return static_cast<int>(ParseResult::ERR_MUSTBEZERO);
  }
  uint32_t counter = (static_cast<uint32_t>(payload[12]) << 24) |
                     (static_cast<uint32_t>(payload[13]) << 16) |
                     (static_cast<uint32_t>(payload[14]) << 8) |
                     static_cast<uint32_t>(payload[15]);

  size_t expected = 16 + enc_data_len + AES_TAG_SIZE;
  if (expected > payload_len) {
    ESP_LOGW(TAG, "Nested frame: L=%u + headers > payload %u",
             enc_data_len, static_cast<unsigned>(payload_len));
    return static_cast<int>(ParseResult::ERR_LEN_MATH);
  }
  if (enc_data_len > plain_max) return static_cast<int>(ParseResult::ERR_LEN);

  // Replay protection for nested encrypted frames.
  if (counter <= last_nested_rx_counter_) {
    ESP_LOGW(TAG, "Nested RX replay rejected: counter 0x%08X ≤ last 0x%08X",
             static_cast<unsigned>(counter), static_cast<unsigned>(last_nested_rx_counter_));
    return static_cast<int>(ParseResult::ERR_REPLAY);
  }

  // Copy ciphertext+MIC to mutable buffer and decrypt.
  uint8_t work[MAX_RF_FRAME_SIZE];
  if (enc_data_len + AES_TAG_SIZE > sizeof(work)) return static_cast<int>(ParseResult::ERR_LEN);
  memcpy(work, payload + 16, enc_data_len + AES_TAG_SIZE);

  int dec_len = aes_ccm_decrypt(work, enc_data_len + AES_TAG_SIZE, counter);
  if (dec_len < 0) return static_cast<int>(ParseResult::ERR_MIC);

  // Accept — bump the nested high-water-mark counter.
  last_nested_rx_counter_ = counter;

  memcpy(plain_out, work, dec_len);
  return dec_len;
}

/* ================================================================
 * RX-type classification & test-ping handling
 * ================================================================ */

RfDataLayer::RxClass RfDataLayer::classify_rx_type(uint8_t b1) {
  switch (b1) {
    case 0x06: return RxClass::PING;
    case 0x40: return RxClass::SHORT_ACK;
    case 0x43: return RxClass::LARGE_RESPONSE;
    case 0x53: return RxClass::SESSION_SETUP;
    case 0x5B: return RxClass::KEEPALIVE;
    default:   return RxClass::UNKNOWN;
  }
}

/// Test-ping detection — matches firmware master_rx_handler @ 0xC254
/// special-case branch. The meter sends:
///   in[0] ∈ {0x11, 0x13}, in[1] == 'h' (0x68), in[16] == 0x16
/// The CIU's response is to increment byte[0], stamp the channel/quality at
/// byte[0x12], and retransmit the first 0x13 bytes.
bool RfDataLayer::is_test_ping(const uint8_t *in, size_t in_len) {
  if (in == nullptr || in_len < 0x13) return false;
  if (in[0] != 0x11 && in[0] != 0x13) return false;
  if (in[1] != 'h') return false;
  if (in[0x10] != 0x16) return false;
  return true;
}

size_t RfDataLayer::build_test_ping_response(uint8_t *buf, size_t buf_max) {
  if (buf == nullptr || buf_max < 0x13) return 0;
  if (!is_test_ping(buf, 0x13)) return 0;
  // Firmware does: rx_buf[0] += 1; rx_buf[0x12] = channel_id_lookup(rssi); retransmit 0x13B.
  buf[0]    = static_cast<uint8_t>(buf[0] + 1);
  // The "quality" byte at [0x12] is the 6-bit-clamped quality from current RSSI.
  uint8_t q = compose_channel_byte(0, last_rssi_dbm_) & 0x3F;
  buf[0x12] = q;
  return 0x13;
}

/* ================================================================
 * Channel Selection
 * ================================================================ */
uint8_t RfDataLayer::select_best_channel(const int8_t rssi[4]) {
  // Firmware (rssi_channel_select / 0x134A4) picks the quietest channel
  // (lowest measured RSSI). Channel 0 is scanned but never selected.
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
