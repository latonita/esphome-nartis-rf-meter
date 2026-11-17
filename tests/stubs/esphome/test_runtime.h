#pragma once
#include <cstdint>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <vector>
namespace esphome {
struct ESPTime {
  int year = 2026, month = 11, day_of_month = 7, hour = 1, minute = 0, second = 0, day_of_week = 7;
  static ESPTime from_epoch_utc(long) { return {}; }
  bool is_valid() const { return true; }
};
inline uint32_t test_now = 10000;
inline std::vector<std::string> test_logs;
inline uint32_t millis() { return test_now; }
inline void delay(uint32_t n) { test_now += n; }
inline void delayMicroseconds(uint32_t) {}
inline void get_mac_address_raw(uint8_t *p) { std::memset(p, 0, 6); }
inline uint32_t fnv1_hash(const std::string &) { return 1; }
using LogString = char;
inline std::string format_hex_pretty(const uint8_t *, size_t) { return "hex"; }
inline void test_log(const char *, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
inline void test_log(const char *, const char *fmt, ...) {
  char buf[512]; va_list args; va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args); va_end(args); test_logs.emplace_back(buf);
}
namespace setup_priority { inline constexpr float DATA = 600; }
class PollingComponent {
 public:
  virtual ~PollingComponent() = default;
  virtual void setup() {} virtual void dump_config() {} virtual void update() {}
  virtual void loop() {} virtual float get_setup_priority() const { return 0; }
  void mark_failed() {} bool is_failed() const { return false; }
  void set_timeout(uint32_t, std::function<void()>) {}
};
class ISRInternalGPIOPin {
 public: void digital_write(bool) {} bool digital_read() { return false; }
};
class InternalGPIOPin {
 public: int get_pin() const { return 0; }
};
namespace gpio { enum Flags { FLAG_INPUT = 1, FLAG_OUTPUT = 2 }; }
inline int test_pref_saves = 0;  // number of NVS session saves requested
class ESPPreferenceObject {
 public: template<class T> bool load(T *) { return false; }
  template<class T> bool save(const T *) { ++test_pref_saves; return true; }
};
class TestPreferences {
 public: template<class T> ESPPreferenceObject make_preference(uint32_t) { return {}; }
  bool sync() { return true; }
};
inline TestPreferences test_preferences;
inline TestPreferences *global_preferences = &test_preferences;
namespace sensor {
class Sensor {
 public: std::string get_name() const { return "test"; }
  void publish_state(float v) { states.push_back(v); }
  std::vector<float> states;
};
}
namespace text_sensor {
class TextSensor {
 public: std::string get_name() const { return "text"; }
  void publish_state(const std::string &v) { states.push_back(v); }
  std::vector<std::string> states;
};
}
namespace button { class Button { protected: virtual void press_action() {} }; }
}
#define ESP_LOGE(...) esphome::test_log(__VA_ARGS__)
#define ESP_LOGW(...) esphome::test_log(__VA_ARGS__)
#define ESP_LOGI(...) esphome::test_log(__VA_ARGS__)
#define ESP_LOGD(...) esphome::test_log(__VA_ARGS__)
#define ESP_LOGV(...) esphome::test_log(__VA_ARGS__)
#define ESP_LOGVV(...) esphome::test_log(__VA_ARGS__)
#define ESP_LOGCONFIG(...) esphome::test_log(__VA_ARGS__)
#define LOG_STR(x) (x)
#define LOG_STR_ARG(x) (x)
