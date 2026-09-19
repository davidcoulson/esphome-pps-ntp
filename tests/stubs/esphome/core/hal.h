#pragma once
#include <cstdint>
extern int64_t g_now_us;
namespace esphome {
inline uint32_t millis() { return static_cast<uint32_t>(g_now_us / 1000); }
namespace gpio { enum InterruptType { INTERRUPT_RISING_EDGE = 1 }; }
class InternalGPIOPin {
 public:
  void setup() {}
  uint8_t get_pin() const { return 15; }
  bool is_inverted() const { return false; }
  template<typename T> void attach_interrupt(void (*)(T *), T *, gpio::InterruptType) const {}
};
}  // namespace esphome
