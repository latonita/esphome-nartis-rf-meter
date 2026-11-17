// Actual RF framing/CRC/AES-GCM, DLMS parser and orchestrator; only the ESPHome
// runtime and physical radio are stubbed. Frames are generated independently.
#include "nartis_rf_meter.h"
#include "fake_radio.h"
#include "rf_vectors.h"
#include <cassert>
#include <iostream>
#include <algorithm>
using namespace esphome;
using namespace esphome::nartis_rf_meter;
using Bytes = std::vector<uint8_t>;
static int scenarios = 0;

static void configure(RfDataLayer &rf) {
  rf.set_address(RfAddress::from_bytes(fixtures::CIU));
  rf.set_meter_address(RfAddress::from_bytes(fixtures::METER));
  rf.set_aes_key(fixtures::KEY);
}
static void recalc_crc(Bytes &b) {
  auto put = [&](size_t begin, size_t end) {
    uint16_t c = RfDataLayer::crc16_calc(b.data() + begin, end - begin);
    b[end] = c >> 8; b[end + 1] = c;
  };
  if (b.size() <= 128) put(0, b.size() - 2);
  else { put(0, 126); put(128, b.size() - 2); }
}
static void test_real_crc_aes_replay_and_peek() {
  RfDataLayer rf; configure(rf); RfFrameType type;
  uint8_t payload[512]{}, plain[512]{};
  auto parse = [&](const Bytes &f) { return rf.parse_frame(f.data(), f.size(), payload, sizeof(payload), &type); };
  assert(parse(fixtures::ACK40) == 0 && uint8_t(type) == RX_TYPE_SHORT_ACK);
  assert(parse(fixtures::ACK5B) == 0 && uint8_t(type) == RX_TYPE_REQUEST_ACK);
  assert(fixtures::NORMAL1.size() < 128 && fixtures::BATCH10.size() > 128);
  for (const Bytes *packet : {&fixtures::NORMAL1, &fixtures::BATCH10}) {
    const int n = parse(*packet); assert(n > 0);
    const auto counter = rf.get_last_nested_rx_counter();
    const int len = rf.peek_nested_plain(payload, n, plain, sizeof(plain)); assert(len > 0);
    assert(rf.get_last_nested_rx_counter() == counter);
    assert(rf.parse_nested_encrypted(payload, n, plain, sizeof(plain)) == len);
    assert(rf.get_last_nested_rx_counter() > counter);
    assert(rf.parse_nested_encrypted(payload, n, plain, sizeof(plain)) == int(RfDataLayer::ParseResult::ERR_REPLAY));
    assert(rf.peek_nested_plain(payload, n, plain, sizeof(plain)) == len);
  }
  auto bad = fixtures::BATCH6; bad.back() ^= 1;
  assert(parse(bad) == int(RfDataLayer::ParseResult::ERR_CRC));
  bad = fixtures::BATCH10; bad[126] ^= 1;
  assert(parse(bad) == int(RfDataLayer::ParseResult::ERR_CRC));
  bad = fixtures::ACK40; bad[2] ^= 1; recalc_crc(bad);
  assert(parse(bad) == int(RfDataLayer::ParseResult::ERR_ADDR));
  // Valid outer CRC, corrupt nested authentication tag: neither parser may
  // accept it or move the replay watermark.
  bad = fixtures::BATCH6; bad[bad.size() - 3] ^= 1; recalc_crc(bad);
  int n = parse(bad); assert(n > 0);
  auto counter = rf.get_last_nested_rx_counter();
  assert(rf.peek_nested_plain(payload, n, plain, sizeof(plain)) < 0);
  assert(rf.parse_nested_encrypted(payload, n, plain, sizeof(plain)) == int(RfDataLayer::ParseResult::ERR_MIC));
  assert(rf.get_last_nested_rx_counter() == counter);
  // All truncations of a genuine multi-CRC frame must fail safely.
  for (size_t len = 0; len < fixtures::BATCH10.size(); ++len)
    assert(rf.parse_frame(fixtures::BATCH10.data(), len, payload, sizeof(payload), &type) < 0);
  ++scenarios;
}

class Meter : public NartisRfMeterComponent {
 public:
  using NartisRfMeterComponent::handle_get_tx_;
  using NartisRfMeterComponent::handle_state_;
  using NartisRfMeterComponent::start_cycle_;
  using NartisRfMeterComponent::validate_close_response_;
  Meter() {
    radio_ = &radio;
    paired_ = true; session_primed_ = true; user_batch_size_ = 10;
    sensors_.resize(16); sensors_[4].sensor = &power;
    sensors_[15].text_sensor = &clock; sensors_[15].class_id = 8;
    configure(rf_);
  }
  void feed(State s, const Bytes &b) {
    assert(b.size() <= sizeof(rx_accum_buf_));
    memcpy(rx_accum_buf_, b.data(), b.size()); rx_accum_len_ = b.size();
    rx_expected_len_ = b.size(); rx_active_ = true;
    state_ = s; state_entered_ms_ = test_now;
  }
  void receive(State s, const Bytes &b) { feed(s, b); handle_state_(); }
  void close(const Bytes &b) { receive(State::READ_WAIT_CLOSE_ACK, b); }
  uint8_t invoke() const { return get_invoke_id_; }
  uint32_t rx_counter() const { return rf_.get_last_nested_rx_counter(); }
  size_t start() const { return batch_start_idx_; }
  bool valid(size_t i) const { return sensors_[i].last_value.has_value(); }
  bool primed() const { return session_primed_; }
  State state() const { return state_; }
  sensor::Sensor power;
  text_sensor::TextSensor clock;
  FakeRadio radio;
};
static void test_two_encrypted_cycles_and_stale_data() {
  Meter m; m.start_cycle_(); assert(!m.primed()); m.handle_get_tx_(); assert(m.invoke() == 1);
  m.receive(Meter::State::READ_WAIT_REQUEST_ACK, fixtures::NORMAL1);
  assert(!m.valid(0)); m.close(fixtures::NORMAL1); assert(m.primed());
  m.handle_get_tx_(); assert(m.invoke() == 2);
  m.receive(Meter::State::READ_WAIT_REQUEST_ACK, fixtures::STALE_SAME_COUNT);
  assert(m.start() == 0 && !m.valid(0)); assert(m.state() == Meter::State::READ_REQUEST_TX);
  m.handle_get_tx_(); assert(m.invoke() == 2);
  m.receive(Meter::State::READ_WAIT_REQUEST_ACK, fixtures::BATCH10);
  assert(m.start() == 10); const auto rx = m.rx_counter();
  // Close may receive the exact encrypted duplicate. It must not republish or
  // move the replay counter backwards, and wrong-id duplicates cannot close.
  m.feed(Meter::State::READ_WAIT_CLOSE_ACK, fixtures::STALE_SAME_COUNT);
  assert(!m.validate_close_response_()); assert(m.rx_counter() == rx);
  m.close(fixtures::BATCH10); assert(m.rx_counter() == rx); assert(m.power.states.empty());
  m.handle_get_tx_(); assert(m.invoke() == 3);
  m.receive(Meter::State::READ_WAIT_REQUEST_ACK, fixtures::STALE_WRONG_COUNT);
  assert(m.start() == 10 && !m.valid(10)); m.handle_get_tx_();
  m.receive(Meter::State::READ_WAIT_REQUEST_ACK, fixtures::BATCH6); m.close(fixtures::ACK40);
  assert(m.state() == Meter::State::PUBLISH); m.handle_state_();
  assert(m.power.states.size() == 1 && m.power.states[0] == 200);
  assert(m.clock.states[0] == "2026-11-08 01:34:47");
  for (int i = 0; i < 16; ++i) assert(m.valid(i));

  test_now += 300000; m.start_cycle_(); assert(!m.primed()); assert(m.rx_counter() == 5);
  m.handle_get_tx_(); assert(m.invoke() == 4);
  m.receive(Meter::State::READ_WAIT_REQUEST_ACK, fixtures::ACK5B);
  for (int i = 0; i < 2; ++i) m.receive(Meter::State::READ_WAIT_DATA, fixtures::ACK40);
  assert(m.primed()); m.handle_get_tx_(); assert(m.invoke() == 5);
  m.receive(Meter::State::READ_WAIT_REQUEST_ACK, fixtures::NORMAL4);
  assert(m.start() == 0 && !m.valid(0)); m.handle_get_tx_();
  m.receive(Meter::State::READ_WAIT_REQUEST_ACK, fixtures::NEXT_BATCH10); m.close(fixtures::ACK40);
  m.handle_get_tx_(); assert(m.invoke() == 6);
  m.receive(Meter::State::READ_WAIT_REQUEST_ACK, fixtures::NEXT_BATCH6); m.close(fixtures::ACK40);
  assert(m.state() == Meter::State::PUBLISH); m.handle_state_();
  assert(m.power.states.size() == 2 && m.power.states.back() == 200);
  assert(m.clock.states.size() == 2); ++scenarios;
}
static void test_corrupt_control_frames_cannot_advance() {
  for (auto s : {Meter::State::READ_WAIT_REQUEST_ACK, Meter::State::READ_WAIT_DATA}) {
    Meter m; m.start_cycle_(); m.handle_get_tx_();
    auto b = s == Meter::State::READ_WAIT_DATA ? fixtures::ACK40 : fixtures::ACK5B;
    b.back() ^= 1; m.receive(s, b);
    assert(m.state() == Meter::State::READ_REQUEST_TX && !m.primed());
    m.handle_get_tx_(); b = fixtures::ACK40; b[3] ^= 1; recalc_crc(b);
    m.receive(Meter::State::READ_WAIT_DATA, b);
    assert(m.state() == Meter::State::READ_REQUEST_TX && !m.primed());
  }
  // Wake: a genuine 0x5B means awake; a 0x5B with a broken CRC does not.
  {
    Meter m; m.start_cycle_(); m.receive(Meter::State::WAKE_WAIT_ACK, fixtures::ACK5B);
    assert(m.state() == Meter::State::READ_REQUEST_TX);
    Meter c; c.start_cycle_(); auto b = fixtures::ACK5B; b.back() ^= 1;
    c.receive(Meter::State::WAKE_WAIT_ACK, b);
    assert(c.state() != Meter::State::READ_REQUEST_TX);
  }
  ++scenarios;
}
int main() {
  test_real_crc_aes_replay_and_peek();
  test_two_encrypted_cycles_and_stale_data();
  test_corrupt_control_frames_cannot_advance();
  std::cout << scenarios << " RF/AES integration scenarios passed\n";
}
