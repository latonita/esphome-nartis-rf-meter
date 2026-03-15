/*
 * Nartis RF Data Layer
 *
 * Layer 2: Packet framing, CRC-16/DNP block coding, AES-128-CCM encryption,
 * and frequency channel management.
 *
 * Knows RF frame structure but NOT DLMS protocol.
 * Produces/consumes raw byte buffers that Layer 3 (DLMS) fills.
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include "helpers.h"

namespace esphome::nartis_rf_meter {

class RfDataLayer {
 public:
  RfDataLayer() = default;

  /// Set the 8-byte RF address for this device.
  void set_address(const RfAddress &addr);

  /// Set AES-128 key (16 bytes). E.g. "ZCZfuT666iRdgPNH"
  void set_aes_key(const uint8_t key[AES_KEY_SIZE]);

  /* ---- TX Path ---- */

  /// Build a complete RF frame from a DLMS payload.
  /// Returns total frame size, or 0 on error.
  ///
  /// For RF-encrypted frames (ACK/BEACON): payload → AES-CCM encrypt → CRC block insert → RF header
  /// For DLMS-wrapped frames (DATA/PLAIN_DATA): payload → CRC block insert → RF header
  size_t build_frame(uint8_t *out, size_t out_max,
                     RfFrameType type, uint8_t src_id,
                     uint8_t sequence,
                     const uint8_t *payload, size_t payload_len);

  /* ---- RX Path ---- */

  /// Parse a received RF frame, extracting DLMS payload.
  /// Returns payload length, or -1 on error (CRC fail, decrypt fail, address mismatch).
  int parse_frame(const uint8_t *in, size_t in_len,
                  uint8_t *payload_out, size_t payload_max,
                  RfFrameType *type_out);

  /* ---- Channel Management ---- */

  /// Select the best channel from 4 RSSI measurements.
  /// Returns channel index 0-3 (picks the one with strongest signal, or quietest for TX).
  static uint8_t select_best_channel(const int8_t rssi[4]);

  /// Get/set current channel index.
  void set_channel(uint8_t ch) { current_channel_ = ch; }
  uint8_t get_channel() const { return current_channel_; }

  /* ---- Frame Counter ---- */

  uint32_t get_frame_counter() const { return frame_counter_; }
  void set_frame_counter(uint32_t counter) { frame_counter_ = counter; }
  void increment_frame_counter() { frame_counter_++; }

  /* ---- CRC-16/DNP (public for testing) ---- */

  /// Calculate CRC-16/DNP over a byte buffer.
  static uint16_t crc16_calc(const uint8_t *data, size_t len);

  /// Verify CRC-16/DNP: compute CRC over data[0..len-3], compare with data[len-2..len-1] (big-endian).
  static bool crc16_verify(const uint8_t *data, size_t len);

 private:
  /* ---- CRC Block Framing ---- */

  /// Insert CRC-16 blocks into TX payload (plaintext mode).
  /// Returns total size after CRC insertion.
  static size_t crc_block_insert_plain(uint8_t *out, size_t out_max,
                                       const uint8_t *data, size_t len);

  /// Insert CRC-16 for encrypted payload (single or double CRC block).
  /// Returns total size after CRC insertion.
  static size_t crc_block_insert_encrypted(uint8_t *out, size_t out_max,
                                           const uint8_t *data, size_t len);

  /// Strip CRC blocks from RX plaintext payload. Returns data length after stripping, or -1 on CRC error.
  static int crc_block_strip_plain(uint8_t *data, size_t len);

  /// Strip CRC from RX encrypted payload. Returns data length after stripping, or -1 on CRC error.
  static int crc_block_strip_encrypted(uint8_t *data, size_t len);

  /* ---- AES-128-CCM ---- */

  /// Encrypt payload in-place. Appends 12-byte MIC tag.
  /// Returns new length (payload_len + AES_TAG_SIZE), or 0 on error.
  size_t aes_ccm_encrypt(uint8_t *data, size_t data_len, size_t buf_max, uint32_t counter);

  /// Decrypt payload in-place. Verifies and removes 12-byte MIC tag.
  /// Returns new length (data_len - AES_TAG_SIZE), or -1 on auth failure.
  int aes_ccm_decrypt(uint8_t *data, size_t data_len, uint32_t counter);

  /// Build 12-byte nonce: 8-byte address + 4-byte counter (big-endian).
  void build_nonce(uint8_t nonce[AES_NONCE_SIZE], uint32_t counter) const;

  /* ---- CRC Lookup Table ---- */
  static const uint16_t crc_table_[256];

  /* ---- State ---- */
  RfAddress address_{};
  uint8_t aes_key_[AES_KEY_SIZE]{};
  uint32_t frame_counter_{0};
  uint8_t current_channel_{0};
  uint8_t sequence_{0};
};

}  // namespace esphome::nartis_rf_meter
