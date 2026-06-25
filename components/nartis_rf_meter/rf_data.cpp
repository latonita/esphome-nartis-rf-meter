/*
 * Nartis RF Data Layer - Implementation
 *
 * CRC-16/DNP framing (single + dual modes), AES-128-GCM, packet assembly.
 */

#include "rf_data.h"
#include <cstring>
#include <algorithm>

#include "esphome/core/log.h"
#include "esphome/core/defines.h"

// Crypto: AES-128-GCM is implemented self-contained below (pure-C AES-128 +
// GHASH/CTR). ESPHome's mbedTLS port compiles out MBEDTLS_GCM_C, and under the
// esp-idf framework the component link does not even resolve mbedtls_aes_* -
// so we depend on NO external crypto library. The on-air cipher is AES-128-GCM
// with AAD = [01 29 L 00], nonce = permuted CIU address + 4-byte counter (BE),
// 12-byte tag (confirmed by decrypting real frames).

namespace esphome::nartis_rf_meter {

static const char *const TAG = "rf_data";

// CRC-16/DNP lookup table, 256 entries.
// Polynomial 0x3D65, init 0x0000, no reflection, xorout 0xFFFF.
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
 * CRC Insert / Strip (encrypted-path mode).
 *
 * Two on-air layouts:
 *   A) body_size <= 126: single CRC at end. Frame size = body_size + 2.
 *   B) body_size  > 126: CRC1 at offset 0x7E over [0..0x7D], body bytes
 *      [0x7E..body_size-1] shifted right by 2, CRC2 appended at end over
 *      [0x80..frame_size-3]. Frame size = body_size + 4.
 * ================================================================ */

size_t RfDataLayer::crc_insert(uint8_t *buf, size_t body_size, size_t out_max) {
  if (body_size <= 126) {
    if (body_size + 2 > out_max) return 0;
    // CRITICAL: the meter sets the length byte
    // (buf[0] += 2) BEFORE computing the CRC, so the CRC covers the final
    // length-1 value. Set it first here too, otherwise the CRC is computed over
    // buf[0]==0 and every TX frame fails the meter's CRC check.
    buf[RF_TX_LENGTH] = static_cast<uint8_t>((body_size + 2) - 1);
    uint16_t crc = crc16_calc(buf, body_size);
    buf[body_size]     = static_cast<uint8_t>(crc >> 8);
    buf[body_size + 1] = static_cast<uint8_t>(crc & 0xFF);
    return body_size + 2;
  } else {
    if (body_size + 4 > out_max) return 0;
    // Length byte covers the final frame (buf[0] += 4) - set before CRC1.
    buf[RF_TX_LENGTH] = static_cast<uint8_t>((body_size + 4) - 1);
    // Shift bytes [0x7E..body_size-1] right by 2 to make room for CRC1.
    // Use memmove for overlapping region.
    size_t tail_size = body_size - CRC1_OFFSET;
    memmove(buf + CRC1_OFFSET + 2, buf + CRC1_OFFSET, tail_size);

    // CRC1 over [0..0x7D] (includes the length byte set above)
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
    if (!crc16_verify(buf, frame_size)) {
      ESP_LOGW(TAG, "CRC fail (single, size=%u)", static_cast<unsigned>(frame_size));
      return -1;
    }
    return static_cast<int>(frame_size - 2);
  } else {
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
 * AES-128-GCM  (verified by decrypting real frames)
 *
 *   key   = ASCII(meter_serial)[12] + BD 02 9B BE
 *   nonce = permuted CIU address (8B) + counter (4B, big-endian)
 *           permutation: prefix[i] = addr_wire[{2,3,4,5,0,1,6,7}[i]]
 *   AAD   = { 0x01, 0x29, enc_len, 0x00 }  (= on-air frame bytes [13..16])
 *   tag   = 12 bytes
 *
 * The same CIU-address nonce prefix is used in BOTH directions (the meter's
 * responses embed the CIU address in their nested header) - so decrypt uses
 * address_, not meter_address_.
 * ================================================================ */

namespace {

// ---- Pure-C AES-128 block encrypt (FIPS-197). No external crypto library. ----
const uint8_t kSbox[256] = {
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,
    0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,
    0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
    0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,
    0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,
    0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
    0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,
    0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
    0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
    0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,
    0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,
    0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
    0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
    0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,
    0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
    0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16};
const uint8_t kRcon[10] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36};

inline uint8_t aes_gmul(uint8_t a, uint8_t b) {
  uint8_t r = 0;
  for (int i = 0; i < 8; i++) {
    if (b & 1) r ^= a;
    uint8_t hi = a & 0x80;
    a = static_cast<uint8_t>(a << 1);
    if (hi) a ^= 0x1b;
    b >>= 1;
  }
  return r;
}

void aes128_key_expansion(const uint8_t key[16], uint8_t rk[176]) {
  memcpy(rk, key, 16);
  for (int i = 4; i < 44; i++) {
    uint8_t t[4];
    for (int j = 0; j < 4; j++) t[j] = rk[(i - 1) * 4 + j];
    if (i % 4 == 0) {
      uint8_t tmp = t[0]; t[0] = t[1]; t[1] = t[2]; t[2] = t[3]; t[3] = tmp;  // RotWord
      for (int j = 0; j < 4; j++) t[j] = kSbox[t[j]];                         // SubWord
      t[0] ^= kRcon[i / 4 - 1];
    }
    for (int j = 0; j < 4; j++) rk[i * 4 + j] = rk[(i - 4) * 4 + j] ^ t[j];
  }
}

inline void aes128_ecb(const uint8_t key[16], const uint8_t in[16], uint8_t out[16]) {
  uint8_t rk[176];
  aes128_key_expansion(key, rk);
  uint8_t s[16];
  memcpy(s, in, 16);
  for (int i = 0; i < 16; i++) s[i] ^= rk[i];               // AddRoundKey 0
  for (int rnd = 1; rnd <= 10; rnd++) {
    for (int i = 0; i < 16; i++) s[i] = kSbox[s[i]];         // SubBytes
    uint8_t ns[16];                                          // ShiftRows (col-major: byte = col*4+row)
    for (int r = 0; r < 4; r++)
      for (int c = 0; c < 4; c++) ns[c * 4 + r] = s[((c + r) & 3) * 4 + r];
    memcpy(s, ns, 16);
    if (rnd != 10) {                                         // MixColumns
      for (int c = 0; c < 4; c++) {
        uint8_t *col = s + c * 4;
        uint8_t a0 = col[0], a1 = col[1], a2 = col[2], a3 = col[3];
        col[0] = static_cast<uint8_t>(aes_gmul(a0, 2) ^ aes_gmul(a1, 3) ^ a2 ^ a3);
        col[1] = static_cast<uint8_t>(a0 ^ aes_gmul(a1, 2) ^ aes_gmul(a2, 3) ^ a3);
        col[2] = static_cast<uint8_t>(a0 ^ a1 ^ aes_gmul(a2, 2) ^ aes_gmul(a3, 3));
        col[3] = static_cast<uint8_t>(aes_gmul(a0, 3) ^ a1 ^ a2 ^ aes_gmul(a3, 2));
      }
    }
    for (int i = 0; i < 16; i++) s[i] ^= rk[rnd * 16 + i];   // AddRoundKey
  }
  memcpy(out, s, 16);
}

// GHASH multiply: X = X * H in GF(2^128) (NIST SP 800-38D bit convention).
void ghash_mul(uint8_t X[16], const uint8_t H[16]) {
  uint8_t Z[16] = {0};
  uint8_t V[16];
  memcpy(V, H, 16);
  for (int i = 0; i < 128; i++) {
    if ((X[i >> 3] >> (7 - (i & 7))) & 1) {
      for (int j = 0; j < 16; j++) Z[j] ^= V[j];
    }
    uint8_t lsb = V[15] & 1;
    for (int j = 15; j > 0; j--) V[j] = static_cast<uint8_t>((V[j] >> 1) | (V[j - 1] << 7));
    V[0] >>= 1;
    if (lsb) V[0] ^= 0xE1;
  }
  memcpy(X, Z, 16);
}

// GHASH over AAD || CIPHERTEXT || lengths, producing the auth tag in tag[0..15].
void gcm_tag(const uint8_t H[16], const uint8_t ej0[16], const uint8_t *aad, size_t aad_len,
             const uint8_t *ct, size_t ct_len, uint8_t tag[16]) {
  uint8_t X[16] = {0};
  uint8_t blk[16];
  for (size_t off = 0; off < aad_len; off += 16) {
    size_t n = (aad_len - off < 16) ? aad_len - off : 16;
    memset(blk, 0, 16);
    memcpy(blk, aad + off, n);
    for (int j = 0; j < 16; j++) X[j] ^= blk[j];
    ghash_mul(X, H);
  }
  for (size_t off = 0; off < ct_len; off += 16) {
    size_t n = (ct_len - off < 16) ? ct_len - off : 16;
    memset(blk, 0, 16);
    memcpy(blk, ct + off, n);
    for (int j = 0; j < 16; j++) X[j] ^= blk[j];
    ghash_mul(X, H);
  }
  uint8_t L[16] = {0};
  uint64_t aad_bits = static_cast<uint64_t>(aad_len) * 8;
  uint64_t ct_bits = static_cast<uint64_t>(ct_len) * 8;
  for (int j = 0; j < 8; j++) L[7 - j] = static_cast<uint8_t>(aad_bits >> (8 * j));
  for (int j = 0; j < 8; j++) L[15 - j] = static_cast<uint8_t>(ct_bits >> (8 * j));
  for (int j = 0; j < 16; j++) X[j] ^= L[j];
  ghash_mul(X, H);
  for (int j = 0; j < 16; j++) tag[j] = X[j] ^ ej0[j];
}

// CTR-XOR data in place, keystream starts at inc32(J0).
void gcm_ctr(const uint8_t key[16], const uint8_t j0[16], uint8_t *data, size_t len) {
  uint8_t ctr[16];
  memcpy(ctr, j0, 16);
  uint8_t ks[16];
  for (size_t off = 0; off < len; off += 16) {
    for (int k = 15; k >= 12; k--) { if (++ctr[k] != 0) break; }  // inc32
    aes128_ecb(key, ctr, ks);
    size_t n = (len - off < 16) ? len - off : 16;
    for (size_t j = 0; j < n; j++) data[off + j] ^= ks[j];
  }
}

// AES-128-GCM with a 12-byte IV. encrypt: seals data+writes tag. decrypt: returns
// false on tag mismatch. nonce is 12 bytes; tag is 12 bytes (truncated).
bool gcm128(const uint8_t key[16], const uint8_t nonce[12], const uint8_t *aad, size_t aad_len,
            uint8_t *data, size_t len, uint8_t *tag12, bool encrypt) {
  uint8_t H[16], j0[16], ej0[16], full_tag[16];
  const uint8_t zero[16] = {0};
  aes128_ecb(key, zero, H);
  memcpy(j0, nonce, 12);
  j0[12] = 0; j0[13] = 0; j0[14] = 0; j0[15] = 1;
  aes128_ecb(key, j0, ej0);
  if (encrypt) {
    gcm_ctr(key, j0, data, len);              // plaintext -> ciphertext
    gcm_tag(H, ej0, aad, aad_len, data, len, full_tag);
    memcpy(tag12, full_tag, 12);
    return true;
  }
  gcm_tag(H, ej0, aad, aad_len, data, len, full_tag);  // tag over received ciphertext
  uint8_t diff = 0;
  for (int j = 0; j < 12; j++) diff |= static_cast<uint8_t>(full_tag[j] ^ tag12[j]);
  if (diff != 0) return false;
  gcm_ctr(key, j0, data, len);                // ciphertext -> plaintext
  return true;
}

}  // namespace

void RfDataLayer::build_nonce(uint8_t nonce[AES_NONCE_SIZE], const RfAddress &addr, uint32_t counter) const {
  uint8_t aw[8];
  addr.to_bytes(aw);  // on-wire order: dev_lo dev_hi hash0 hash1 hash2 hash3 group type
  // Permute to the nonce prefix: [2,3,4,5,0,1,6,7].
  static const uint8_t kPerm[8] = {2, 3, 4, 5, 0, 1, 6, 7};
  for (uint8_t i = 0; i < 8; i++) nonce[i] = aw[kPerm[i]];
  nonce[8]  = static_cast<uint8_t>((counter >> 24) & 0xFF);
  nonce[9]  = static_cast<uint8_t>((counter >> 16) & 0xFF);
  nonce[10] = static_cast<uint8_t>((counter >> 8) & 0xFF);
  nonce[11] = static_cast<uint8_t>(counter & 0xFF);
}

size_t RfDataLayer::aes_gcm_encrypt(uint8_t *data, size_t data_len, size_t buf_max, uint32_t counter) {
  if (data_len + AES_TAG_SIZE > buf_max) {
    ESP_LOGE(TAG, "Buffer too small for AES-GCM encrypt");
    return 0;
  }
  uint8_t nonce[AES_NONCE_SIZE];
  build_nonce(nonce, address_, counter);
  const uint8_t aad[4] = {0x01, AES_GCM_FLAG, static_cast<uint8_t>(data_len), 0x00};

  uint8_t tag[AES_TAG_SIZE];
  gcm128(aes_key_, nonce, aad, sizeof(aad), data, data_len, tag, /*encrypt=*/true);
  memcpy(data + data_len, tag, AES_TAG_SIZE);
  return data_len + AES_TAG_SIZE;
}

int RfDataLayer::aes_gcm_decrypt(uint8_t *data, size_t data_len, uint32_t counter) {
  if (data_len < AES_TAG_SIZE) return -1;
  size_t ct_len = data_len - AES_TAG_SIZE;

  // Both directions use the CIU address as the nonce prefix (the meter's
  // nested responses carry the CIU address in their header).
  uint8_t nonce[AES_NONCE_SIZE];
  build_nonce(nonce, address_, counter);
  const uint8_t aad[4] = {0x01, AES_GCM_FLAG, static_cast<uint8_t>(ct_len), 0x00};

  if (!gcm128(aes_key_, nonce, aad, sizeof(aad), data, ct_len, data + ct_len, /*encrypt=*/false)) {
    ESP_LOGW(TAG, "gcm tag check failed (wrong key/counter or tampered)");
    return -1;
  }
  return static_cast<int>(ct_len);
}

bool RfDataLayer::extract_session_key(const uint8_t *payload, size_t len,
                                      uint8_t out_key[AES_KEY_SIZE]) const {
  // The meter ships the per-pairing data key inside the 0x53 SESSION_SETUP,
  // GCM-encrypted with the factory default key. Locate the nested key-install
  // header (0D FD 19 1E), read the 4-byte inner counter, then the 18-byte
  // encrypted blob ([slot][len=0x10][16-byte key]).
  static const uint8_t kKeyInstallMarker[4] = {0x0D, 0xFD, 0x19, 0x1E};
  static const uint8_t kFactoryKey[AES_KEY_SIZE] = {
      0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22,
      0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22};
  const size_t kHdr = 4, kCtr = 4, kBlob = 18;  // marker + counter + blob
  for (size_t h = 0; h + kHdr + kCtr + kBlob <= len; h++) {
    if (memcmp(payload + h, kKeyInstallMarker, kHdr) != 0) continue;
    const uint8_t *c = payload + h + kHdr;
    uint32_t counter = (static_cast<uint32_t>(c[0]) << 24) | (static_cast<uint32_t>(c[1]) << 16) |
                       (static_cast<uint32_t>(c[2]) << 8) | static_cast<uint32_t>(c[3]);
    uint8_t nonce[AES_NONCE_SIZE];
    build_nonce(nonce, address_, counter);
    // CTR is symmetric: running gcm128 "encrypt" over the ciphertext yields the
    // plaintext. The tag is irrelevant here (8-byte MIC, validated by the caller
    // via the embedded serial), so the throwaway tag buffer is fine.
    uint8_t buf[kBlob];
    memcpy(buf, payload + h + kHdr + kCtr, kBlob);
    uint8_t throwaway_tag[AES_TAG_SIZE];
    gcm128(kFactoryKey, nonce, nullptr, 0, buf, kBlob, throwaway_tag, /*encrypt=*/true);
    if (buf[1] != AES_KEY_SIZE) return false;  // len byte must be 0x10
    memcpy(out_key, buf + 2, AES_KEY_SIZE);
    return true;
  }
  return false;
}

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
 * Verified examples:
 *   -68 dBm -> quality 31 -> channel 1 gives 0x5F, channel 2 gives 0x9F
 *    -6 dBm -> quality 62 (saturated) -> channel 1 gives 0x7E
 * ================================================================ */
uint8_t RfDataLayer::compose_channel_byte(uint8_t channel_idx, int8_t rssi_dbm) {
  int q = (static_cast<int>(rssi_dbm) + 130) / 2;
  if (q < 1) q = 1;
  if (q > 62) q = 62;
  return static_cast<uint8_t>(((channel_idx & 0x3) << 6) | (q & 0x3F));
}

size_t RfDataLayer::build_frame(uint8_t *out, size_t out_max,
                                RfFrameType type, uint8_t sequence,
                                const uint8_t *payload, size_t payload_len) {
  if (out_max < RF_TX_HDR_SIZE + 4) return 0;

  // Mode 1 (DATA): DLMS-level security, no RF AES-GCM.
  // Mode 6 (PLAIN_DATA): plain on secondary channel.
  // Mode 2 (ACK) + Mode 3 (BEACON): RF AES-GCM.
  const bool encrypted   = (type == RfFrameType::ACK || type == RfFrameType::BEACON);
  const uint8_t ci_field = (type == RfFrameType::PLAIN_DATA) ? CI_FIELD_SPECIAL : CI_FIELD_NORMAL;

  out[RF_TX_LENGTH]       = 0;  // filled at the end
  out[RF_TX_FLAGS]        = static_cast<uint8_t>(type);
  address_.to_bytes(out + RF_TX_ADDR);
  out[RF_TX_CI_FIELD]  = ci_field;
  out[RF_TX_SEQUENCE]     = sequence;
  out[RF_TX_CHANNEL_BYTE] = compose_channel_byte(current_channel_, last_rssi_dbm_);
  out[RF_TX_ENC_FLAG]     = encrypted ? 0x01 : 0x00;
  out[RF_TX_SEC_FLAG]     = encrypted ? AES_GCM_FLAG : 0x00;

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
    // Counter management:
    //   the counter increments ONLY when the sequence byte changes from the
    //   previous TX. Same sequence => reuse the same counter (e.g. beacon
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

    memcpy(out + RF_TX_ENC_CIPHER, payload, payload_len);
    size_t enc_len = aes_gcm_encrypt(out + RF_TX_ENC_CIPHER, payload_len,
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

  size_t total = crc_insert(out, body_size, out_max);
  if (total == 0) return 0;

  out[RF_TX_LENGTH] = static_cast<uint8_t>(total - 1);
  return total;
}

/* ================================================================
 * Frame Parsing (RX, meter->CIU).
 *
 * Steps:
 *   1. Read length-1 from [0], validate against in_len.
 *   2. CRC strip (in-place on a working copy): single or dual.
 *   3. Validate meter address at [2..9] matches expected.
 *   4. If [11]=[12]=0 -> plain branch, return payload at [13..body_end].
 *   5. Else -> encrypted branch (rare in observed dataset).
 * ================================================================ */

int RfDataLayer::parse_frame(const uint8_t *in, size_t in_len,
                             uint8_t *payload_out, size_t payload_max,
                             RfFrameType *type_out) {
  if (in_len < RF_RX_HDR_SIZE + 2) {
    ESP_LOGW(TAG, "Frame too short: %u bytes", static_cast<unsigned>(in_len));
    return static_cast<int>(ParseResult::ERR_LEN);
  }
  // Meter upper bound: rx_len <= 0x122 (290).
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
  uint8_t *const work = work_;
  if (frame_size > sizeof(work_)) {
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
      ESP_LOGD(TAG, "RX address mismatch - discarding (got %02X%02X / hash %08X, type %02X)",
               src.manufacturer_id & 0xFF, (src.manufacturer_id >> 8) & 0xFF,
               static_cast<unsigned>(src.address_id), src.device_type);
      return static_cast<int>(ParseResult::ERR_ADDR);
    }
  }

  // Plain vs encrypted branch - discriminate by frame_type, NOT by bytes
  // [11..12]. The meter uses (work[11]==0 && work[12]==0)
  // as a "plain" indicator, but those bytes are actually the high two bytes
  // of the embedded 4-byte target-CIU hash echo. The meter addresses we
  // observed all have CIU hashes whose top 2 bytes are 0x00, so the meter's
  // check passes accidentally. Our CIU has hash 00 01 26 5D - byte [12] = 0x01,
  // which falsely triggers the encrypted branch.
  //
  // Every RX frame we observed is plain at the transport layer.
  // 0x43 (large RESPONSE) and 0x53 (SESSION_SETUP) carry a *nested* encrypted
  // body, but the outer envelope is plain and the parser hands its payload
  // to the application layer which deals
  // with the nesting.
  const uint8_t frame_type = work[RF_RX_TYPE];
  const bool is_plain = (frame_type == RX_TYPE_PRESENCE_ACK || frame_type == RX_TYPE_SHORT_ACK ||
                         frame_type == RX_TYPE_DATA || frame_type == RX_TYPE_SESSION_SETUP ||
                         frame_type == RX_TYPE_REQUEST_ACK);

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
    // Encrypted RX branch.
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

    // Exact length math: enc_data_len + must_be_zero + 0x1F == parse_len
    // => enc_data_len + 0 + 0x1F == body_size  (since body_size is post-CRC parse_len).
    if (static_cast<size_t>(enc_data_len) + 0x1F != static_cast<size_t>(body_size)) {
      ESP_LOGW(TAG, "Encrypted RX length math: L+0x1F (%u) != body_size (%d)",
               static_cast<unsigned>(enc_data_len + 0x1F), body_size);
      return static_cast<int>(ParseResult::ERR_LEN_MATH);
    }

    // Replay protection: reject any counter <= the highest counter ever accepted.
    // The meter returns 9 (replay) on this case.
    if (counter <= last_rx_counter_) {
      ESP_LOGW(TAG, "Encrypted RX replay rejected: counter 0x%08X <= last 0x%08X",
               static_cast<unsigned>(counter), static_cast<unsigned>(last_rx_counter_));
      return static_cast<int>(ParseResult::ERR_REPLAY);
    }

    int dec_len = aes_gcm_decrypt(work + 19, enc_data_len + AES_TAG_SIZE, counter);
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
 * Layout (verified on all observed RX 0x43 frames):
 *   [0..1]   2 bytes prefix (last 2 bytes of CIU address_id typically)
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
 * Nonce: the meter uses an address derived from the embedded prefix +
 * CIU constants. For now we use the configured CIU address (set_address)
 * as the nonce prefix - works for sessions where the meter responds to
 * a known CIU.
 * ================================================================ */
int RfDataLayer::parse_nested_encrypted(const uint8_t *payload, size_t payload_len,
                                        uint8_t *plain_out, size_t plain_max) {
  if (payload_len < 16 + AES_TAG_SIZE) {
    ESP_LOGW(TAG, "Nested frame too short: %u", static_cast<unsigned>(payload_len));
    return static_cast<int>(ParseResult::ERR_LEN);
  }
  if (payload[8] != 0x01 || payload[9] != AES_GCM_FLAG) {
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
    ESP_LOGW(TAG, "Nested RX replay rejected: counter 0x%08X <= last 0x%08X",
             static_cast<unsigned>(counter), static_cast<unsigned>(last_nested_rx_counter_));
    return static_cast<int>(ParseResult::ERR_REPLAY);
  }

  uint8_t *const work = work_;
  if (enc_data_len + AES_TAG_SIZE > sizeof(work_)) return static_cast<int>(ParseResult::ERR_LEN);
  memcpy(work, payload + 16, enc_data_len + AES_TAG_SIZE);

  int dec_len = aes_gcm_decrypt(work, enc_data_len + AES_TAG_SIZE, counter);
  if (dec_len < 0) return static_cast<int>(ParseResult::ERR_MIC);

  // Accept - bump the nested high-water-mark counter.
  last_nested_rx_counter_ = counter;

  memcpy(plain_out, work, dec_len);
  return dec_len;
}

int RfDataLayer::peek_nested_plain(const uint8_t *payload, size_t payload_len,
                                   uint8_t *plain_out, size_t plain_max) const {
  // Mirror parse_nested_encrypted's layout checks but WITHOUT the replay
  // check or counter bump - purely to show the decrypted DLMS in the log.
  if (payload_len < 16 + AES_TAG_SIZE) return -1;
  if (payload[8] != 0x01 || payload[9] != AES_GCM_FLAG || payload[11] != 0) return -1;
  const uint8_t enc_data_len = payload[10];
  uint32_t counter = (static_cast<uint32_t>(payload[12]) << 24) |
                     (static_cast<uint32_t>(payload[13]) << 16) |
                     (static_cast<uint32_t>(payload[14]) << 8) |
                     static_cast<uint32_t>(payload[15]);
  if (16 + static_cast<size_t>(enc_data_len) + AES_TAG_SIZE > payload_len) return -1;
  if (enc_data_len > plain_max) return -1;
  uint8_t *const work = work_;
  if (static_cast<size_t>(enc_data_len) + AES_TAG_SIZE > sizeof(work_)) return -1;
  memcpy(work, payload + 16, static_cast<size_t>(enc_data_len) + AES_TAG_SIZE);
  uint8_t nonce[AES_NONCE_SIZE];
  build_nonce(nonce, address_, counter);
  const uint8_t aad[4] = {0x01, AES_GCM_FLAG, enc_data_len, 0x00};
  if (!gcm128(aes_key_, nonce, aad, sizeof(aad), work, enc_data_len, work + enc_data_len,
              /*encrypt=*/false)) {
    return -1;
  }
  memcpy(plain_out, work, enc_data_len);
  return enc_data_len;
}

uint8_t RfDataLayer::select_best_channel(const int8_t rssi[4]) {
  // The meter picks the quietest channel
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
