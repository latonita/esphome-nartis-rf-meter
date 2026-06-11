/*
 * Nartis RF Data Layer
 *
 * Layer 2: Packet framing, CRC-16/DNP, AES-128-GCM, channel management.
 *
 * Structure (matches firmware rf_build_frame @ 0xBB88 and rf_parse_frame
 * @ 0xBF5E, verified against 208 SPI-extracted frames from Pair-B):
 *
 *   TX (CIU→meter) layout:
 *     [0]     length-1
 *     [1]     flags (0x44 ACK | 0x46 DATA | 0x08 BEACON | 0x00 PLAIN)
 *     [2..9]  CIU address (CD 2C + hash[4] + group + type)
 *     [10]    mode_marker (0x7A normal | 0x8A special)
 *     [11]    sequence
 *     [12]    channel_byte = (chan_idx << 6) | (quality & 0x3F)
 *     [13]    enc_flag (0x01 encrypted)
 *     [14]    sec_flag (0x29 = fixed security/AAD header byte)
 *     [15]    enc_data_len = L
 *     [16]    0x00
 *     [17..20] frame_counter (BE u32)
 *     [21..20+L]      ciphertext
 *     [21+L..32+L]    AES-GCM tag (12 bytes)
 *     trailing CRC(s): single if total ≤ 128, dual otherwise (CRC1 at 0x7E)
 *
 *   RX (meter→CIU) plain layout:
 *     [0]     length-1
 *     [1]     RX frame_type (0x40 | 0x43 | 0x53 | 0x5B | 0x06)
 *     [2..9]  meter address
 *     [10]    passthrough byte
 *     [11..12] mode flags (both 0 ⇒ plain)
 *     [13..end-2-CRC] payload
 *     trailing CRC(s): same rule as TX
 *
 * Knows RF frame structure but NOT DLMS protocol.
 * Produces/consumes raw byte buffers that Layer 3 (DLMS) fills.
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>
#include "helpers.h"

namespace esphome::nartis_rf_meter {

class RfDataLayer {
 public:
  RfDataLayer() = default;

  /* ---- Configuration ---- */

  /// Set the local (CIU) 8-byte RF address used for TX address field
  /// AND for the AES-GCM nonce prefix.
  void set_address(const RfAddress &addr);

  /// Set the expected meter address (used to filter incoming frames).
  /// If `group_id`/`device_type` are zero in `addr`, address matching skips them.
  void set_meter_address(const RfAddress &addr);
  const RfAddress &get_meter_address() const { return meter_address_; }

  /// Set AES-128 key (16 bytes), e.g. "ZCZfuT666iRdgPNH"
  void set_aes_key(const uint8_t key[AES_KEY_SIZE]);
  /// Read back the active AES-128 data key (16 bytes).
  void get_aes_key(uint8_t key[AES_KEY_SIZE]) const { memcpy(key, aes_key_, AES_KEY_SIZE); }

  /// Extract the 16-byte data key the meter delivers inside a 0x53
  /// SESSION_SETUP key-install blob. The blob is GCM-encrypted with the
  /// factory default key (0x22 x16); the nonce is perm(CIU address) + the
  /// blob's inner counter (firmware key_install_verify @0x12DF0). The
  /// plaintext is [slot][len=0x10][16-byte key = ASCII(meter_SN)[12] +
  /// meter-assigned 4-byte suffix]. Returns true and fills `out_key` on a
  /// well-formed blob (len byte == 0x10). CTR is symmetric so no separate
  /// tag check is needed — the caller validates the embedded serial.
  bool extract_session_key(const uint8_t *payload, size_t len, uint8_t out_key[AES_KEY_SIZE]) const;

  /// Set the channel index (0..3) and a representative RSSI (dBm) for
  /// outgoing frames' channel_byte at [12]. RSSI is mapped via the
  /// firmware formula `quality = clamp((rssi+130)/2, 1, 62)`.
  void set_channel_quality(uint8_t channel_idx, int8_t rssi_dbm) {
    current_channel_ = channel_idx & 0x3;
    last_rssi_dbm_   = rssi_dbm;
  }
  uint8_t get_channel() const { return current_channel_; }

  /* ---- TX Path ---- */

  /// Build a complete on-air RF frame.
  ///   `payload` / `payload_len`: plaintext DLMS payload.
  /// For encrypted modes (ACK/BEACON/DATA), the payload is AES-GCM encrypted
  /// and the frame includes the enc_data_len/counter/tag layout per spec.
  /// For PLAIN_DATA mode, the payload is written verbatim.
  /// Returns total frame size, or 0 on error.
  ///
  /// `sequence` is the value to write at [11]. Increment per mode externally.
  size_t build_frame(uint8_t *out, size_t out_max,
                     RfFrameType type, uint8_t sequence,
                     const uint8_t *payload, size_t payload_len);

  /// Legacy-compat overload (src_id is ignored — used to be written at the wrong offset).
  size_t build_frame(uint8_t *out, size_t out_max,
                     RfFrameType type, uint8_t /*src_id*/, uint8_t sequence,
                     const uint8_t *payload, size_t payload_len) {
    return build_frame(out, out_max, type, sequence, payload, payload_len);
  }

  /* ---- RX Path ---- */

  /// Result codes mirroring firmware parser return values + replay reject.
  enum class ParseResult : int {
    OK              =  0,
    ERR_LEN         = -1,
    ERR_CRC         = -2,
    ERR_ADDR        = -3,
    ERR_MIC         = -4,
    ERR_MUSTBEZERO  = -5,  // matches firmware return 6
    ERR_LEN_MATH    = -6,  // matches firmware return 4 (length math)
    ERR_REPLAY      = -7,  // counter rewound (firmware return 9)
  };

  /// Parse a received RF frame, extracting payload.
  /// Validates CRC(s) (single or dual), verifies sender address matches
  /// the configured meter address, and (if encrypted) AES-GCM decrypts
  /// AND rejects replayed counters (counter must be > last_rx_counter_).
  ///
  /// On RX type 0x43 plain frames, the payload contains a NESTED encrypted
  /// inner frame; this function returns the plain transport payload as-is —
  /// caller may then call parse_nested_encrypted() on it.
  ///
  /// Returns payload length on success, or a negative ParseResult value
  /// (cast to int) on error. The `type_out` is set to byte[1] cast to
  /// RfFrameType.
  int parse_frame(const uint8_t *in, size_t in_len,
                  uint8_t *payload_out, size_t payload_max,
                  RfFrameType *type_out);

  /// Decrypt a nested encrypted frame found inside an RX 0x43 plain payload.
  /// Layout: 8 bytes prefix, then enc_flag=0x01, ccm=0x29, enc_data_len,
  /// 0x00, counter[4], ciphertext, MIC.
  /// Validates byte[11]==0 and rejects rewound nested counters.
  /// Returns plaintext length, or negative ParseResult on failure.
  int parse_nested_encrypted(const uint8_t *payload, size_t payload_len,
                              uint8_t *plain_out, size_t plain_max);

  /// Diagnostic-only, NON-mutating decrypt of a nested encrypted body (same
  /// layout as parse_nested_encrypted). Skips replay checks and does NOT
  /// touch any counter state, so it is safe to call for logging before the
  /// real handler parses the frame. Returns plaintext length, or <0.
  int peek_nested_plain(const uint8_t *payload, size_t payload_len,
                        uint8_t *plain_out, size_t plain_max) const;

  /* ---- Channel Management ---- */

  /// Select the quietest channel (1..3) from 4 RSSI readings.
  /// Channel 0 is scanned but never selected by firmware.
  static uint8_t select_best_channel(const int8_t rssi[4]);

  void set_channel(uint8_t ch) { current_channel_ = ch & 0x3; }

  /* ---- Frame Counter ---- */

  /// TX-side counter (used in the AES-GCM nonce; auto-incremented per encrypted TX).
  uint32_t get_frame_counter() const { return frame_counter_; }
  /// Set the TX counter — caller should call on startup with NVS-persisted value
  /// + safety margin (e.g. +16) to avoid replay rejection by the meter after reboot.
  void set_frame_counter(uint32_t counter) { frame_counter_ = counter; }
  void increment_frame_counter() { frame_counter_++; }

  /// RX-side replay-protection counter (highest counter ever accepted in an
  /// encrypted RX frame). Matches firmware ctx[+8] in aes_counter_mgr.
  /// MUST be persisted to NVS across reboots.
  uint32_t get_last_rx_counter() const { return last_rx_counter_; }
  void set_last_rx_counter(uint32_t v) { last_rx_counter_ = v; }
  /// Reset replay-counter state (firmware does this for frame[6]==0x80
  /// session-reset messages). Use sparingly.
  void reset_rx_counter() { last_rx_counter_ = 0; }

  /// Nested-frame replay-protection counter (separate from top-level encrypted
  /// counter — applies to inner encrypted frames inside RX 0x43 envelopes).
  uint32_t get_last_nested_rx_counter() const { return last_nested_rx_counter_; }
  void set_last_nested_rx_counter(uint32_t v) { last_nested_rx_counter_ = v; }

  /* ---- CRC-16/DNP (public for testing) ---- */

  /// Compute CRC-16/DNP over a byte buffer.
  /// Polynomial 0x3D65, init 0x0000, no reflection, xorout 0xFFFF.
  static uint16_t crc16_calc(const uint8_t *data, size_t len);

  /// Verify CRC: compute over data[0..len-3], compare with BE u16 at [len-2..len-1].
  static bool crc16_verify(const uint8_t *data, size_t len);

  /// Compute channel_byte from channel index and RSSI (firmware formula).
  static uint8_t compose_channel_byte(uint8_t channel_idx, int8_t rssi_dbm);

 private:
  /* ---- CRC insertion / stripping ---- */

  /// Insert CRC(s) on the buffer in-place; returns final size.
  /// Single CRC if `body_size + 2 <= CRC_DUAL_THRESHOLD`, else dual CRC
  /// (CRC1 at offset 0x7E, body bytes [0x7E..body_size-1] shifted right
  /// by 2, CRC2 appended at end). `out_max` must accommodate +2 or +4.
  static size_t crc_insert(uint8_t *buf, size_t body_size, size_t out_max);

  /// Verify and strip CRC(s) on the buffer in-place; returns final body size,
  /// or -1 on CRC error.
  static int crc_strip(uint8_t *buf, size_t frame_size);

  /* ---- AES-128-GCM ---- */

  /// Encrypt payload in-place, appending 12-byte tag. AAD = [01 29 len 00].
  /// Returns new length.
  size_t aes_gcm_encrypt(uint8_t *data, size_t data_len, size_t buf_max, uint32_t counter);

  /// Decrypt payload in-place, verifying/removing tag. Returns new length, or -1.
  int aes_gcm_decrypt(uint8_t *data, size_t data_len, uint32_t counter);

  /// Build 12-byte nonce: permuted 8-byte CIU address + 4-byte counter (BE).
  void build_nonce(uint8_t nonce[AES_NONCE_SIZE], const RfAddress &addr, uint32_t counter) const;

  /* ---- CRC Lookup Table ---- */
  static const uint16_t crc_table_[256];

  /* ---- State ---- */
  RfAddress address_{};        // CIU's own address
  RfAddress meter_address_{};  // expected meter address (RX filter)
  bool meter_address_set_{false};
  uint8_t aes_key_[AES_KEY_SIZE]{};
  uint32_t frame_counter_{0};
  uint32_t last_rx_counter_{0};
  uint32_t last_nested_rx_counter_{0};
  // Firmware rf_prepare_and_queue tracks last-sent sequence so the AES counter
  // is bumped only when the sequence byte changes — beacons retransmitted with
  // identical content reuse the previous counter.
  uint8_t  last_tx_sequence_{0};
  bool     has_sent_first_tx_{false};
  uint8_t  current_channel_{1};
  int8_t   last_rssi_dbm_{-70};
};

}  // namespace esphome::nartis_rf_meter
