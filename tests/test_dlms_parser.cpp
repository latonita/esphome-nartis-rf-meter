#include "dlms_client.h"
#include "esphome/test_runtime.h"
#include <cassert>
#include <iostream>
#include <vector>
using namespace esphome::nartis_rf_meter;
using Bytes = std::vector<uint8_t>;
static Bytes wrap(const Bytes &apdu) {
  Bytes out{0,1,0,1,0,0x66,uint8_t(apdu.size() >> 8),uint8_t(apdu.size())};
  out.insert(out.end(), apdu.begin(), apdu.end()); return out;
}
static bool parse(const Bytes &b, DlmsValue *v, uint8_t *count, int expected = 6) {
  const uint16_t classes[10] = {3,3,3,3,8,3};
  return DlmsClient().parse_read_response_list(b.data(), b.size(), v, 10, count, classes, 7, expected);
}
int main() {
  // Access errors, nulls and compound results must occupy their original
  // slots; the scalar after them must never shift to the wrong sensor.
  const auto original = wrap({0xC4,3,0xC7,6,
    1,3,                              // access denied
    0,0,                              // null
    0,2,2,0x11,7,0x09,1,'x',         // structure of two values
    0,0x10,0xFF,0xEC,                 // signed -20
    0,9,12,7,0xEA,11,8,7,1,22,47,0xFF,0x80,0,0, // clock octet-string
    0,0x17,0x43,0x66,0,0});           // float32 230
  DlmsValue values[10]{}; uint8_t count = 0;
  assert(parse(original, values, &count)); assert(count == 6);
  for (int i = 0; i < 3; ++i) assert(!values[i].has_value());
  assert(DlmsClient::data_as_float(DlmsDataType(values[3].dtype), values[3].raw, values[3].raw_len) == -20);
  assert(values[4].dtype == uint8_t(DlmsDataType::DATE_TIME));
  char date[64]{}; DlmsClient::data_to_string(DlmsDataType(values[4].dtype), values[4].raw, values[4].raw_len, date, sizeof(date));
  assert(std::string(date) == "2026-11-08 01:22:47");
  assert(DlmsClient::data_as_float(DlmsDataType(values[5].dtype), values[5].raw, values[5].raw_len) == 230);
  for (size_t n = 0; n < original.size(); ++n) {
    auto b = original; b.resize(n); assert(!parse(b, values, &count) && count == 0);
    // Also test internally truncated but self-consistent IEC envelopes.
    if (n >= 8) { b[6] = 0; b[7] = n - 8; assert(!parse(b, values, &count) && count == 0); }
  }
  auto b = original; b.push_back(0); ++b[7]; assert(!parse(b, values, &count) && count == 0);
  for (int n = 0; n < 256; ++n) {
    b = original; b[11] = n;
    assert(parse(b, values, &count) == (n == 6));
    if (n != 6) assert(count == 0);
  }
  for (int invoke = 0; invoke < 16; ++invoke) {
    b = original; b[10] = 0xC0 | invoke;
    assert(parse(b, values, &count) == (invoke == 7));
  }
  // Deterministic mutations exercise nested lengths/type tags and malformed
  // result lists under ASan/UBSan. A successful parse must consume exactly
  // six slots; every rejection reports count=0 to prevent partial publication.
  uint32_t seed = 0x4E415254;
  auto next = [&]() { seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5; return seed; };
  for (int i = 0; i < 20000; ++i) {
    b = original;
    for (uint32_t j = 0, n = 1 + next() % 8; j < n; ++j) b[next() % b.size()] = next();
    if (i % 3 == 0) b.resize(next() % 256, 0);
    const bool ok = parse(b, values, &count);
    assert(count == (ok ? 6 : 0));
    esphome::test_logs.clear();
  }
  std::cout << "DLMS typed results, truncations, IDs/counts and 20000 mutations passed\n";
}
