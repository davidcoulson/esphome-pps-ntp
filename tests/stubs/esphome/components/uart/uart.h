#pragma once
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>
namespace esphome::uart {
struct UARTComponent {
  uint32_t baud{9600};
  std::deque<uint8_t> rx;
  std::vector<uint8_t> tx;
  std::vector<uint32_t> tx_baud;  // the baud each byte was sent at
  uint32_t get_baud_rate() const { return baud; }
  void set_baud_rate(uint32_t b) { baud = b; }
  void load_settings(bool) {}
};
class UARTDevice {
 public:
  UARTComponent *parent_{nullptr};
  size_t available() { return parent_->rx.size(); }
  bool read_byte(uint8_t *b) { if (parent_->rx.empty()) return false; *b = parent_->rx.front(); parent_->rx.pop_front(); return true; }
  void write_array(const uint8_t *d, size_t n) {
    parent_->tx.insert(parent_->tx.end(), d, d + n);
    parent_->tx_baud.insert(parent_->tx_baud.end(), n, parent_->baud);
  }
  int flush() { return 0; }
};
}  // namespace esphome::uart
