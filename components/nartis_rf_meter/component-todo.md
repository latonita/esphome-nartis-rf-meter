# Nartis RF Meter Component — TODO

Gaps between our ESPHome component and the original firmware.

## Critical (no communication without these)

- [x] **#1 `derive_rf_address_()` not implemented** — Declared but never defined or called. `address_` stays at `serial_hash=0`. Every RX frame fails address check. No communication possible.
- [x] **#2 Beacon payload empty** — Firmware sends a 29-byte struct (meter address, timestamp, protocol version, channel config). We send zero-length payload. Meter won't respond.
- [x] **#3 No ACK after GET response** — Firmware sends ACK frame (0x44) after each GET response. We skip it. Meter likely rejects subsequent requests.

## High (will fail in practice)

- [x] **#4 TX > 64 bytes truncated** — `write_fifo()` caps at 64B. AARQ is ~80-100B after CRC framing. Silently corrupt.
- [x] **#5 AARQ format wrong** — Firmware uses NO standard DLMS AARQ. Each register read is a proprietary request: `C0 01 C1 00 [class_id] [obis] [attr_id] 00`. Rewrote DLMS client, eliminated AARQ/AARE/Release phases.
- [ ] **#6 Frame counter not persisted** — Resets to 0 on reboot. Meter rejects as replay. Needs NVS.
- [x] **#7 HLS auth stubbed** — Closed: firmware uses NO DLMS-level auth (no HLS, no LLS). Security is RF-layer AES-CCM only. Password "123456" is for local serial-console pairing, never sent over RF. No class 15 (Association LN) in firmware OBIS tables. Removed HLS stubs.
- [x] **#8 RSSI channel selection inverted** — Fixed: picks quietest (lowest RSSI) from channels 1-3 only (ch0 skipped). 7 readings per channel with min/max outlier removal, matching firmware.
- [x] **#18 FIFO_MERGE_EN never set** — Fixed: `FIFO_CTL (0x69) |= 0x02` added to `apply_runtime_overrides()`. This bit merges the separate 32B TX + 32B RX FIFOs into a single 64B buffer. Without it, `transmit_chunked()` would overflow the 32B TX FIFO when writing up to `FIFO_SIZE_MERGED` (64) bytes. The 12-byte FIFO threshold (ISR chunk size) is unrelated — it controls how often `RX_FIFO_TH` fires during reception.
- [x] **#19 EN_CTL (0x62) not configured** — Fixed: `EN_CTL |= 0x20` (LFOSC_RECAL) added to `init()`. Register 0x62 is in the control bank (0x60+), not covered by the 6-bank config write (0x00-0x5F).
- [x] **#20 FIFO direction not set before TX** — Fixed: `FIFO_CTL |= 0x05` (FIFO_RX_TX_SEL + SPI_FIFO_RD_WR_SEL) added to `transmit_chunked()`. Firmware switches direction before every TX/RX; we only had the RX direction set in `prepare_fifo_read()`.

## Medium (wrong data / partial failures)

- [ ] **#9 No scaler/unit for register values** — Raw integer published without 10^N scaling. Readings off by orders of magnitude.
- [x] **#10 GET invoke-id encoding** — Not a bug: byte[3] is hardcoded `0x00` in firmware (not a counter). In single-address mode (byte[1]=0x01), firmware writes literal zero. Response parser doesn't validate invoke-id. Our code matches exactly.
- [x] **#11 INT1 source not restored after RX** — Fixed: `transmit_chunked` sets INT1=TX_FIFO_TH before TX, `start_rx_` sets INT1=RX_FIFO_TH before RX.
- [ ] **#12 INT64/UINT64 truncated to 32-bit** — Energy counters > 2^31 will wrap.
- [ ] **#13 DATE_TIME, STRUCTURE, ARRAY types unhandled** — `parse_typed_value()` returns NONE for these.

## Low (robustness / edge cases)

- [x] **#14 `is_chip_connected()` runs before config write** — Not a bug: REG_CMT2 (0x01) = 0x66 is a hardwired product ID, readable after soft reset regardless of config state.
- [x] **#15 `gpio_install_isr_service()` return unchecked** — Fixed: now uses ESPHome's `InternalGPIOPin::attach_interrupt()` which handles ISR service installation internally.
- [x] **#16 AES key only accepts ASCII string, no hex** — Closed: Nartis uses ASCII passphrase keys, hex encoding not needed.
- [x] **#17 Blocking `wait_for_state()` in loop (up to 20ms)** — Not a risk: worst case ~30ms (3 chained transitions), ESP32 WDT threshold is ~3.5s. ESPHome components routinely block for similar durations.

## Verified OK (firmware differences by design)

- **FIFO threshold 12 vs firmware 15** — Our ISR reads `FIFO_TH_VALUE` (12) bytes per chunk. Self-consistent with `FifoChunk` buffer size. Just more frequent ISR calls. Not a bug.
- **INT_EN 0x21 vs firmware 0x39** — We don't use PREAM_OK/SYNC_OK flags. INT_EN doesn't gate INT1 pin source (FIFO events fire regardless).
- **GPIO1=INT1 vs firmware GPIO3=INT1** — Different hardware wiring (ESP32 vs HT6027). Same logical function.
