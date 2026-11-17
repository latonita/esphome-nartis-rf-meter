// Exercises the actual orchestrator and DLMS parser. Only ESPHome's runtime,
// radio I/O (FakeRadio) and RF encryption are mocked; there is no simulated
// copy of the FSM. Based on the regression suite from denis-isaev's
// fix/dlms-response-recovery branch, adapted to the RadioHal interface, the
// poll-then-re-send pacing, and extended with re-key and wake cases.
#include "nartis_rf_meter.h"
#include "fake_radio.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>
using namespace esphome;
using namespace esphome::nartis_rf_meter;
using Bytes = std::vector<uint8_t>;
static Bytes last_tx;
static int tests = 0;
static uint32_t serial_counter = 1;
static uint32_t be32(const uint8_t *p) {
  return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | p[3];
}
static Bytes response(uint8_t id, const std::vector<uint32_t> &values) {
  Bytes apdu{0xC4, 0x03, uint8_t(0xC0 | id), uint8_t(values.size())};
  for (uint32_t v : values) {
    apdu.insert(apdu.end(), {0, 0x06, uint8_t(v >> 24), uint8_t(v >> 16), uint8_t(v >> 8), uint8_t(v)});
  }
  Bytes out{0, 1, 0, 1, 0, 0x66, uint8_t(apdu.size() >> 8), uint8_t(apdu.size())};
  out.insert(out.end(), apdu.begin(), apdu.end()); return out;
}
static Bytes normal_response(uint8_t id, uint32_t value = 123) {
  auto p = response(id, {value});
  p[9] = 0x01; p.erase(p.begin() + 11); --p[7];
  return p;
}
static Bytes frame(uint8_t type, const Bytes &apdu = {}, uint32_t counter = 0) {
  Bytes b{0, type};
  if (type == RX_TYPE_DATA) {
    if (!counter) counter = serial_counter++;
    b.insert(b.end(), {uint8_t(counter >> 24), uint8_t(counter >> 16), uint8_t(counter >> 8), uint8_t(counter)});
  }
  b.insert(b.end(), apdu.begin(), apdu.end()); b[0] = b.size() - 1; return b;
}
static bool logged(const char *needle) {
  return std::any_of(test_logs.begin(), test_logs.end(),
                     [&](const auto &s) { return s.find(needle) != std::string::npos; });
}
namespace esphome::nartis_rf_meter {
void RfDataLayer::set_address(const RfAddress &a) { address_ = a; }
void RfDataLayer::set_meter_address(const RfAddress &a) { meter_address_ = a; }
void RfDataLayer::set_aes_key(const uint8_t *p) { memcpy(aes_key_, p, 16); }
bool RfDataLayer::extract_session_key(const uint8_t *, size_t, uint8_t *) const { return false; }
size_t RfDataLayer::build_frame(uint8_t *out, size_t cap, RfFrameType type, uint8_t,
                               const uint8_t *p, size_t len) {
  last_tx.assign(p, p + len); if (cap < 2) return 0;
  out[0] = 1; out[1] = uint8_t(type); ++frame_counter_; return 2;
}
int RfDataLayer::parse_frame(const uint8_t *in, size_t len, uint8_t *out, size_t cap, RfFrameType *type) {
  if (len < 2 || in[1] == 0xFF || len - 2 > cap) return -1;
  *type = RfFrameType(in[1]); memcpy(out, in + 2, len - 2); return len - 2;
}
int RfDataLayer::peek_nested_plain(const uint8_t *p, size_t len, uint8_t *out, size_t cap) const {
  if (len < 4 || len - 4 > cap) return -1;
  memcpy(out, p + 4, len - 4); return len - 4;
}
int RfDataLayer::parse_nested_encrypted(const uint8_t *p, size_t len, uint8_t *out, size_t cap) {
  if (len < 4) return -1;
  if (be32(p) <= last_nested_rx_counter_) return int(ParseResult::ERR_REPLAY);
  int n = peek_nested_plain(p, len, out, cap);
  if (n >= 0) last_nested_rx_counter_ = be32(p);
  return n;
}
}
class Meter : public NartisRfMeterComponent {
 public:
  using NartisRfMeterComponent::handle_get_tx_;
  using NartisRfMeterComponent::handle_state_;
  using NartisRfMeterComponent::handle_data_response_;
  using NartisRfMeterComponent::retry_get_;
  using NartisRfMeterComponent::validate_close_response_;
  using NartisRfMeterComponent::start_cycle_;
  using NartisRfMeterComponent::advance_after_get_;
  using NartisRfMeterComponent::handle_publish_;
  Meter(size_t n, int batch = 5) {
    radio_ = &radio;
    session_primed_ = true; paired_ = true; user_batch_size_ = batch;
    sensors_.resize(n); state_ = State::READ_REQUEST_TX;
  }
  void feed(State s, const Bytes &b) {
    assert(b.size() <= sizeof(rx_accum_buf_));
    if (!b.empty()) memcpy(rx_accum_buf_, b.data(), b.size());
    rx_accum_len_ = b.size();
    rx_expected_len_ = b.size(); rx_active_ = true;
    state_ = s; state_entered_ms_ = test_now;
  }
  void reply(const Bytes &b) { feed(State::READ_WAIT_REQUEST_ACK, b); handle_state_(); }
  void inject(const Bytes &b) {
    const auto entered = state_entered_ms_; feed(state_, b); state_entered_ms_ = entered;
  }
  // One attempt that gets only "not ready" (0x40) answers to every poll beacon.
  void no_data_polls() {
    for (int p = 0; p < 3; ++p) { feed(State::READ_WAIT_DATA, frame(RX_TYPE_SHORT_ACK)); handle_state_(); }
  }
  State state() const { return state_; }
  uint8_t invoke() const { return get_invoke_id_; }
  size_t start() const { return batch_start_idx_; }
  bool valid(size_t i) const { return sensors_[i].last_value.has_value(); }
  float value(size_t i) const {
    const auto &v = sensors_[i].last_value;
    return DlmsClient::data_as_float(DlmsDataType(v.dtype), v.raw, v.raw_len);
  }
  void set_sensor(size_t i, sensor::Sensor *s) { sensors_[i].sensor = s; }
  void unprime() { session_primed_ = false; }
  bool primed() const { return session_primed_; }
  uint32_t tx_counter() const { return rf_.get_frame_counter(); }
  uint32_t nested_rx() const { return rf_.get_last_nested_rx_counter(); }
  bool paired() const { return paired_; }
  void accept(const std::vector<uint32_t> &v) { reply(frame(RX_TYPE_DATA, response(invoke(), v))); }
  FakeRadio radio;
};
static void test_same_size_stale_response() {
  Meter m(10); m.handle_get_tx_(); const uint8_t first = m.invoke();
  m.accept({29331176, 19571884, 9701030, 58262, 200});
  assert(m.start() == 5); assert(m.valid(0));
  m.advance_after_get_(); m.handle_get_tx_(); assert(m.invoke() != first);
  // A previously unseen, fresh-counter frame still belongs to the OLD GET.
  m.reply(frame(RX_TYPE_DATA, response(first, {29331176, 19571884, 9701030, 58262, 200})));
  assert(m.state() == Meter::State::READ_REQUEST_TX); assert(m.start() == 5);
  for (size_t i = 5; i < 10; ++i) assert(!m.valid(i));
  const auto current = m.invoke(); m.handle_get_tx_(); assert(m.invoke() == current);
  m.accept({13, 0, 185, 2230, 2200});
  assert(m.value(5) == 13); assert(m.value(8) == 2230); ++tests;
}
static void test_no_data_polls_then_resends_current_get() {
  Meter m(10); m.handle_get_tx_(); const auto id = m.invoke();
  // "Not ready" answers poll again first, then the request itself is re-sent.
  m.feed(Meter::State::READ_WAIT_DATA, frame(RX_TYPE_SHORT_ACK)); m.handle_state_();
  assert(m.state() == Meter::State::READ_POLL_BEACON_TX);
  m.no_data_polls();
  assert(m.state() == Meter::State::READ_REQUEST_TX); assert(m.start() == 0);
  m.handle_get_tx_(); assert(m.invoke() == id);
  m.accept({29331176, 19571884, 9701030, 58262, 200});
  assert(m.valid(0)); assert(m.start() == 5); ++tests;
}
static void test_failure_does_not_skip_or_publish() {
  Meter m(10); sensor::Sensor s; m.set_sensor(0, &s); m.handle_get_tx_();
  m.accept({100, 30, 30, 40, 200}); m.advance_after_get_(); m.handle_get_tx_();
  for (int i = 0; i < 6; ++i) {
    m.no_data_polls();
    if (i < 5) m.handle_get_tx_();
  }
  assert(m.state() == Meter::State::ERROR_RECOVERY); assert(m.start() == 5);
  assert(s.states.empty()); m.handle_state_(); assert(m.state() == Meter::State::IDLE); ++tests;
}
static void test_retry_budget_cannot_reset_on_keepalive() {
  Meter m(5); m.handle_get_tx_();
  for (int i = 0; i < 6; ++i) {
    m.reply(frame(RX_TYPE_REQUEST_ACK));
    m.no_data_polls();
    if (i < 5) m.handle_get_tx_();
  }
  assert(m.state() == Meter::State::ERROR_RECOVERY); assert(m.start() == 0); ++tests;
}
static void test_close_packet_matching() {
  Meter m(5); m.handle_get_tx_(); const auto id = m.invoke();
  m.accept({1, 2, 3, 4, 5});
  m.feed(Meter::State::READ_WAIT_CLOSE_ACK, frame(RX_TYPE_DATA, response((id + 1) & 15, {1, 2, 3, 4, 5})));
  assert(!m.validate_close_response_());
  m.feed(Meter::State::READ_WAIT_CLOSE_ACK, frame(RX_TYPE_DATA, response(id, {1, 2, 3, 4, 5})));
  assert(m.validate_close_response_());
  m.feed(Meter::State::READ_WAIT_CLOSE_ACK, frame(RX_TYPE_SHORT_ACK)); assert(m.validate_close_response_());
  m.feed(Meter::State::READ_WAIT_CLOSE_ACK, frame(0xFF)); assert(!m.validate_close_response_()); ++tests;
}
static void test_close_repeat_is_not_stored_twice() {
  Meter m(5); sensor::Sensor s; m.set_sensor(0, &s); m.handle_get_tx_();
  m.accept({7, 2, 3, 4, 5}); assert(m.state() == Meter::State::READ_CLOSE_BEACON_TX);
  const auto rx = m.nested_rx();
  // The D101 closes with a repeat of the same reply instead of 0x40.
  m.feed(Meter::State::READ_WAIT_CLOSE_ACK, frame(RX_TYPE_DATA, response(m.invoke(), {9, 9, 9, 9, 9})));
  m.handle_state_();
  assert(m.state() == Meter::State::PUBLISH); assert(m.value(0) == 7); assert(m.nested_rx() == rx);
  m.handle_state_(); assert(s.states.size() == 1 && s.states[0] == 7); ++tests;
}
static void test_10_plus_6_regression() {
  Meter m(16, 10); m.handle_get_tx_(); const auto first = m.invoke();
  m.accept({1,2,3,4,5,6,7,8,9,10}); m.advance_after_get_(); m.handle_get_tx_();
  m.reply(frame(RX_TYPE_DATA, response(first, {1,2,3,4,5,6,7,8,9,10})));
  assert(m.start() == 10); assert(!m.valid(10)); m.handle_get_tx_();
  m.accept({11,12,13,14,15,16});
  assert(m.start() == 16); assert(m.value(15) == 16); ++tests;
}
static void test_parser_bounds_and_builder() {
  DlmsClient d; DlmsValue values[10]{}; uint8_t count = 0; int got_id = -2;
  auto p = response(3, {1,2,3,4,5});
  assert(d.parse_read_response_list(p.data(), p.size(), values, 10, &count, nullptr, 3, 5, false, &got_id));
  assert(count == 5 && got_id == 3);
  assert(!d.parse_read_response_list(p.data(), p.size(), values, 10, &count, nullptr, 4, 5, false, &got_id));
  assert(count == 0 && got_id == 3);
  assert(!d.parse_read_response_list(p.data(), p.size(), values, 10, &count, nullptr, 3, 6));
  p.pop_back(); p[7]--;
  assert(!d.parse_read_response_list(p.data(), p.size(), values, 10, &count, nullptr, 3, 5));
  p = response(3, {1}); p[7]++;
  assert(!d.parse_read_response_list(p.data(), p.size(), values, 10, &count));
  p = response(3, {1}); p[9] = 1;
  assert(!d.parse_read_response_list(p.data(), p.size(), values, 10, &count));
  DlmsClient::AttrSpec a{}; uint8_t request[32]{};
  for (int id = 0; id < 16; ++id) {
    assert(d.build_get_request_with_list(request, sizeof(request), &a, 1, id) == 14);
    assert(request[2] == uint8_t(0xC0 | id));
    assert(d.build_get_request_normal(request, sizeof(request), a.obis, 3, 2, id) == 13);
    assert(request[2] == uint8_t(0xC0 | id));
  }
  ++tests;
}
static void test_invoke_lifecycle_and_reporting() {
  Meter m(1, 1); int prev = -1;
  for (int i = 0; i < 35; ++i) {
    m.start_cycle_(); m.handle_get_tx_(); assert(m.invoke() != prev);
    m.reply(frame(RX_TYPE_DATA, normal_response(m.invoke()))); m.advance_after_get_();
    const auto priming = m.invoke();
    m.handle_get_tx_(); assert(m.invoke() != priming); prev = m.invoke();
    m.accept({uint32_t(i)}); m.advance_after_get_();
  }
  test_logs.clear(); m.handle_publish_();
  assert(logged("complete: 1/1"));
  Meter partial(2); partial.handle_get_tx_();
  partial.accept({1,2}); // both numeric, then test an empty new read cycle
  partial.start_cycle_(); test_logs.clear(); partial.handle_publish_();
  assert(logged("incomplete: 0/2")); ++tests;
}
static void test_priming_normal_is_discarded() {
  Meter m(5); m.unprime(); m.handle_get_tx_(); const auto priming_id = m.invoke();
  auto p = normal_response(priming_id);
  m.reply(frame(RX_TYPE_DATA, p));
  assert(m.state() == Meter::State::READ_CLOSE_BEACON_TX); assert(m.start() == 0);
  for (int i = 0; i < 5; ++i) assert(!m.valid(i));
  m.feed(Meter::State::READ_WAIT_CLOSE_ACK, frame(RX_TYPE_DATA, p));
  assert(m.validate_close_response_());
  m.advance_after_get_(); assert(m.primed()); m.handle_get_tx_();
  assert(m.invoke() != priming_id);
  // The priming answer arriving late, on the first batch, is never stored.
  m.reply(frame(RX_TYPE_DATA, p)); assert(m.start() == 0); assert(!m.valid(0));
  assert(m.state() == Meter::State::READ_REQUEST_TX); ++tests;
}
static void test_stale_reply_at_priming_is_discarded() {
  // Any authenticated reply to the priming GET is thrown away, whatever its
  // form: here a with-list answer left over from an earlier GET.
  Meter m(1, 1); m.start_cycle_(); m.handle_get_tx_();
  m.accept({999});
  assert(!m.valid(0)); assert(m.state() == Meter::State::READ_CLOSE_BEACON_TX);
  m.advance_after_get_(); assert(m.primed());
  // An unrelated control frame must not count toward the two priming ACKs.
  Meter n(1, 1); n.start_cycle_(); n.handle_get_tx_();
  n.feed(Meter::State::READ_WAIT_DATA, frame(RX_TYPE_REQUEST_ACK)); n.handle_state_();
  assert(!n.primed()); assert(n.state() == Meter::State::READ_REQUEST_TX);
  ++tests;
}
static void test_timeout_retries_and_retry_pacing() {
  Meter m(5); m.handle_get_tx_(); const auto id = m.invoke();
  m.feed(Meter::State::READ_WAIT_REQUEST_ACK, {}); test_now += 5001; m.handle_state_();
  assert(m.state() == Meter::State::READ_REQUEST_TX); assert(m.start() == 0);
  auto tx = last_tx; m.handle_state_(); assert(m.state() == Meter::State::READ_REQUEST_TX);
  assert(last_tx == tx); test_now += 1000; m.handle_state_();
  assert(m.state() == Meter::State::READ_REQUEST_WAIT_TX_DONE); assert(m.invoke() == id);
  assert(last_tx == tx);  // same APDU, same invoke-id
  ++tests;
}
static void test_close_wait_is_bounded() {
  Meter m(5); m.handle_get_tx_(); m.accept({1,2,3,4,5});
  m.feed(Meter::State::READ_WAIT_CLOSE_ACK, frame(RX_TYPE_DATA, response((m.invoke() + 1) & 15, {1,2,3,4,5})));
  m.handle_state_(); assert(m.state() == Meter::State::READ_WAIT_CLOSE_ACK);
  test_now += 5001; m.handle_state_(); assert(m.state() == Meter::State::PUBLISH); ++tests;
}
static void test_close_noise_cannot_extend_deadline() {
  Meter m(5); m.handle_get_tx_(); m.accept({1,2,3,4,5});
  auto unrelated = frame(RX_TYPE_DATA, response((m.invoke() + 1) & 15, {1,2,3,4,5}));
  m.feed(Meter::State::READ_WAIT_CLOSE_ACK, unrelated); m.handle_state_();
  assert(m.state() == Meter::State::READ_WAIT_CLOSE_ACK);
  // poll_rx_ gives a complete packet precedence over its timeout. A stream
  // of unrelated complete frames must still obey the closing deadline.
  test_now += 5001; m.inject(unrelated); m.handle_state_();
  assert(m.state() == Meter::State::PUBLISH); ++tests;
}
static void test_repeated_sessions_with_buffered_priming(int batch) {
  Meter m(16, batch); sensor::Sensor power; m.set_sensor(4, &power);
  // One ESP boot, repeated polls. Model the observed meter: the first GET is
  // buffered, and appears only after the next GET is sent.
  for (int cycle = 0; cycle < 3; ++cycle) {
    const auto tx_counter = m.tx_counter();
    m.start_cycle_(); assert(!m.primed()); assert(m.paired());
    assert(m.tx_counter() == tx_counter);  // do not reset pairing/RF counters
    for (int i = 0; i < 16; ++i) assert(!m.valid(i));
    m.handle_get_tx_(); const auto opener = m.invoke();
    assert(last_tx[8] == 0xC0 && last_tx[9] == 0x01);
    m.reply(frame(RX_TYPE_REQUEST_ACK));
    for (int i = 0; i < 2; ++i) {
      m.feed(Meter::State::READ_WAIT_DATA, frame(RX_TYPE_SHORT_ACK)); m.handle_state_();
    }
    assert(m.primed()); assert(m.start() == 0);
    m.handle_get_tx_(); const auto first = m.invoke();
    assert(first != opener); assert(last_tx[9] == 0x03);
    m.reply(frame(RX_TYPE_DATA, normal_response(opener)));
    assert(m.state() == Meter::State::READ_REQUEST_TX && m.start() == 0);
    m.handle_get_tx_(); assert(m.invoke() == first);
    while (m.start() < 16) {
      std::vector<uint32_t> values;
      for (size_t i = m.start(); i < std::min(m.start() + batch, size_t(16)); ++i)
        values.push_back(1000 * cycle + i + 1);
      m.accept(values); m.advance_after_get_();
      if (m.start() < 16) m.handle_get_tx_();
    }
    assert(m.state() == Meter::State::PUBLISH);
    m.handle_state_(); assert(m.state() == Meter::State::IDLE);
    for (int i = 0; i < 16; ++i) assert(m.value(i) == 1000 * cycle + i + 1);
    assert(power.states.size() == size_t(cycle + 1));
    assert(power.states.back() == 1000 * cycle + 5);
    test_now += 300000;
  }
  ++tests;
}
static void test_timer_wrap_and_abort_reprime() {
  test_now = UINT32_MAX - 500;
  Meter m(5); m.handle_get_tx_(); m.retry_get_("test");
  test_now += 999; m.handle_state_(); assert(m.state() == Meter::State::READ_REQUEST_TX);
  ++test_now; m.handle_state_(); assert(m.state() == Meter::State::READ_REQUEST_WAIT_TX_DONE);
  for (int i = 0; i < 5; ++i) m.retry_get_("test");
  assert(m.state() == Meter::State::ERROR_RECOVERY); m.handle_state_();
  m.start_cycle_(); assert(!m.primed()); assert(m.start() == 0);
  m.handle_get_tx_(); assert(last_tx[9] == 0x01); ++tests;
}
static void test_counters_saved_after_failed_cycle() {
  Meter m(5); m.handle_get_tx_();
  for (int i = 0; i < 6; ++i) m.retry_get_("test");
  assert(m.state() == Meter::State::ERROR_RECOVERY);
  const int saves = test_pref_saves; m.handle_state_();
  assert(m.state() == Meter::State::IDLE); assert(test_pref_saves > saves); ++tests;
}
static void test_mid_read_rekey_completes_with_mode6() {
  Meter m(10); m.handle_get_tx_(); m.accept({1, 2, 3, 4, 5}); m.advance_after_get_();
  m.handle_get_tx_(); const auto second = m.invoke();
  const int saves = test_pref_saves;
  // The meter starts a new session in the middle of the second batch.
  m.feed(Meter::State::READ_WAIT_DATA, frame(RX_TYPE_SESSION_SETUP, {0x11, 0x22, 0x33}));
  m.handle_state_();
  assert(m.state() == Meter::State::PAIR_MODE6_TX);
  assert(!m.primed()); assert(m.start() == 5); assert(m.valid(0) && m.value(0) == 1);
  assert(m.tx_counter() == 0); assert(test_pref_saves > saves);
  // Mode-6 waits the meter's key-install time, then goes out as a plain frame.
  const auto sent = m.radio.tx_frames.size();
  m.handle_state_(); assert(m.state() == Meter::State::PAIR_MODE6_TX);
  assert(m.radio.tx_frames.size() == sent);
  test_now += 5500; m.handle_state_();
  assert(m.state() == Meter::State::PAIR_MODE6_WAIT_TX_DONE);
  assert(m.radio.last_tx_type() == uint8_t(RfFrameType::PLAIN_DATA));
  m.handle_state_(); assert(m.state() == Meter::State::PAIR_WAIT_READY);
  m.feed(Meter::State::PAIR_WAIT_READY, frame(RX_TYPE_REQUEST_ACK)); m.handle_state_();
  assert(m.state() == Meter::State::WAKE_BEACON_TX);
  // Restart from the wake: beacon, 0x40, then a new priming GET.
  m.handle_state_(); assert(m.radio.last_tx_type() == uint8_t(RfFrameType::BEACON));
  m.handle_state_(); assert(m.state() == Meter::State::WAKE_WAIT_ACK);
  m.feed(Meter::State::WAKE_WAIT_ACK, frame(RX_TYPE_SHORT_ACK)); m.handle_state_();
  assert(m.state() == Meter::State::READ_REQUEST_TX);
  m.handle_get_tx_(); assert(last_tx[9] == 0x01); assert(m.invoke() != second);
  const auto priming = m.invoke();
  m.reply(frame(RX_TYPE_DATA, normal_response(priming))); m.advance_after_get_();
  m.handle_get_tx_(); assert(m.start() == 5);  // resumes where it was
  m.accept({6, 7, 8, 9, 10}); m.advance_after_get_();
  assert(m.state() == Meter::State::PUBLISH); assert(m.value(0) == 1 && m.value(9) == 10);
  ++tests;
}
static void test_rekeys_are_bounded_and_wake_rekey() {
  Meter m(5); m.handle_get_tx_();
  for (int i = 0; i < 2; ++i) {
    m.feed(Meter::State::READ_WAIT_REQUEST_ACK, frame(RX_TYPE_SESSION_SETUP, {1}));
    m.handle_state_(); assert(m.state() == Meter::State::PAIR_MODE6_TX);
  }
  m.feed(Meter::State::READ_WAIT_CLOSE_ACK, frame(RX_TYPE_SESSION_SETUP, {1}));
  m.handle_state_(); assert(m.state() == Meter::State::ERROR_RECOVERY);
  // A 0x53 answering the wake beacon is completed the same way.
  Meter w(5); w.start_cycle_();
  w.feed(Meter::State::WAKE_WAIT_ACK, frame(RX_TYPE_SESSION_SETUP, {1}));
  w.handle_state_(); assert(w.state() == Meter::State::PAIR_MODE6_TX);
  // A frame that does not parse is never treated as a session setup: it takes
  // the plain re-send path. (The CRC case is covered with real frames in
  // test_rf_integration.cpp's corrupt-control-frame scenario.)
  Meter c(5); c.handle_get_tx_();
  Bytes broken = frame(RX_TYPE_SESSION_SETUP, {1}); broken[1] = 0xFF;
  c.feed(Meter::State::READ_WAIT_REQUEST_ACK, broken); c.handle_state_();
  assert(c.state() == Meter::State::READ_REQUEST_TX);
  ++tests;
}
static void test_redelivered_data_at_wake_pauses_then_recovers() {
  Meter m(5); m.start_cycle_();
  const auto rx = m.nested_rx();
  // ~30 s after a read the meter answers the wake with its last data frame.
  m.feed(Meter::State::WAKE_WAIT_ACK, frame(RX_TYPE_DATA, response(7, {1, 2, 3, 4, 5})));
  m.handle_state_();
  assert(m.state() == Meter::State::WAKE_BEACON_TX); assert(m.nested_rx() == rx);
  for (int i = 0; i < 5; ++i) assert(!m.valid(i));
  const auto sent = m.radio.tx_frames.size();
  m.handle_state_(); test_now += 4499; m.handle_state_();
  assert(m.state() == Meter::State::WAKE_BEACON_TX); assert(m.radio.tx_frames.size() == sent);
  test_now += 1; m.handle_state_();
  assert(m.state() == Meter::State::WAKE_BEACON_WAIT_TX_DONE);
  assert(m.radio.last_tx_type() == uint8_t(RfFrameType::BEACON));
  m.handle_state_(); m.feed(Meter::State::WAKE_WAIT_ACK, frame(RX_TYPE_SHORT_ACK)); m.handle_state_();
  assert(m.state() == Meter::State::READ_REQUEST_TX); ++tests;
}
static void test_request_ack_at_wake_is_awake() {
  // A meter already in a session answers the wake beacon with 0x5B: that is
  // awake, the same as 0x40, and the cycle goes on to the priming GET.
  Meter m(5); m.start_cycle_();
  const auto sent = m.radio.tx_frames.size();
  m.feed(Meter::State::WAKE_WAIT_ACK, frame(RX_TYPE_REQUEST_ACK)); m.handle_state_();
  assert(m.state() == Meter::State::READ_REQUEST_TX); assert(m.radio.tx_frames.size() == sent);
  m.handle_get_tx_(); assert(last_tx[9] == 0x01);
  // Same right after a mid-read re-key: mode-6 answered 0x5B, then the wake
  // beacon is answered 0x5B again.
  Meter r(5); r.handle_get_tx_();
  r.feed(Meter::State::READ_WAIT_DATA, frame(RX_TYPE_SESSION_SETUP, {1})); r.handle_state_();
  test_now += 5500; r.handle_state_(); r.handle_state_();
  r.feed(Meter::State::PAIR_WAIT_READY, frame(RX_TYPE_REQUEST_ACK)); r.handle_state_();
  r.handle_state_(); r.handle_state_(); assert(r.state() == Meter::State::WAKE_WAIT_ACK);
  r.feed(Meter::State::WAKE_WAIT_ACK, frame(RX_TYPE_REQUEST_ACK)); r.handle_state_();
  assert(r.state() == Meter::State::READ_REQUEST_TX); assert(!r.primed());
  // A frame that does not parse is not "awake".
  Meter n(5); n.start_cycle_();
  n.feed(Meter::State::WAKE_WAIT_ACK, frame(0xFF)); n.handle_state_();
  assert(n.state() != Meter::State::READ_REQUEST_TX);
  ++tests;
}
static void test_invoke_check_off_logs_only() {
  Meter m(5); m.set_invoke_id_check(false); m.handle_get_tx_(); const auto id = m.invoke();
  test_logs.clear();
  m.reply(frame(RX_TYPE_DATA, response((id + 1) & 15, {1, 2, 3, 4, 5})));
  assert(m.start() == 5 && m.valid(0)); assert(logged("match=NO"));
  // Form and count are still enforced with the check off.
  Meter n(5); n.set_invoke_id_check(false); n.handle_get_tx_();
  n.reply(frame(RX_TYPE_DATA, normal_response(n.invoke())));
  assert(n.start() == 0 && !n.valid(0));
  n.handle_get_tx_(); n.reply(frame(RX_TYPE_DATA, response(n.invoke(), {1, 2, 3})));
  assert(n.start() == 0 && !n.valid(0)); ++tests;
}
int main() {
  // The next poll session needs a throwaway opener too. The original code
  // kept this flag set for the entire boot.
  { Meter m(16, 10); m.start_cycle_(); assert(!m.primed()); }
  test_same_size_stale_response(); test_no_data_polls_then_resends_current_get();
  test_failure_does_not_skip_or_publish(); test_retry_budget_cannot_reset_on_keepalive();
  test_close_packet_matching(); test_close_repeat_is_not_stored_twice(); test_10_plus_6_regression();
  test_parser_bounds_and_builder(); test_invoke_lifecycle_and_reporting();
  test_priming_normal_is_discarded(); test_stale_reply_at_priming_is_discarded();
  test_timeout_retries_and_retry_pacing(); test_close_wait_is_bounded();
  test_repeated_sessions_with_buffered_priming(10);
  test_repeated_sessions_with_buffered_priming(5);
  test_repeated_sessions_with_buffered_priming(1);
  test_close_noise_cannot_extend_deadline();
  test_counters_saved_after_failed_cycle();
  test_mid_read_rekey_completes_with_mode6(); test_rekeys_are_bounded_and_wake_rekey();
  test_redelivered_data_at_wake_pauses_then_recovers(); test_request_ack_at_wake_is_awake();
  test_invoke_check_off_logs_only();
  test_timer_wrap_and_abort_reprime();
  std::cout << tests << " regression scenarios passed\n";
}
