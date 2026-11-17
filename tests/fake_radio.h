// In-memory RadioHal for host tests. Nothing is ever received through it: the
// tests place a complete frame in the component's RX buffer and step the state
// machine. Transmissions are recorded so tests can check what went on air.
#pragma once
#include "radio_hal.h"
#include <vector>

namespace esphome::nartis_rf_meter {

class FakeRadio : public RadioHal {
 public:
  const char *radio_name() const override { return "fake"; }
  void set_frequency(uint32_t) override {}
  void set_rx_frequency(uint32_t) override {}
  bool init() override { return true; }
  bool transmit(const uint8_t *frame, size_t len) override {
    tx_frames.emplace_back(frame, frame + len);
    return true;
  }
  bool begin_rx() override { return true; }
  size_t drain_rx(uint8_t *, size_t, RxStatus &status) override {
    status = RxStatus::RX_STATUS_NO_DATA;
    return 0;
  }
  size_t read_rx_exact(uint8_t *, size_t) override { return 0; }
  size_t rx_chunk_size() const override { return 1; }
  size_t max_drain_chunk() const override { return 64; }
  int8_t get_rssi_dbm() override { return -100; }
  bool go_standby() override { return true; }
  void dump_config() override {}

  /// Frame type byte of the last transmission (byte 1), or -1 if none.
  int last_tx_type() const { return tx_frames.empty() || tx_frames.back().size() < 2 ? -1 : tx_frames.back()[1]; }

  std::vector<std::vector<uint8_t>> tx_frames;
};

}  // namespace esphome::nartis_rf_meter
