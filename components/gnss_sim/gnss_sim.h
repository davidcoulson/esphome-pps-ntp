#pragma once

#include <atomic>
#include <cstdint>
#include <string>

#include <esp_timer.h>

#include "esphome/core/component.h"

namespace esphome::gnss_sim {

class GNSSSim : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  // After pps_ntp and the uart: components, so the pads they configured can be re-shared
  float get_setup_priority() const override { return setup_priority::LATE; }

  void set_pins(int pps, int tx, int rx) {
    this->pps_pin_ = pps;
    this->tx_pin_ = tx;
    this->rx_pin_ = rx;
  }
  void set_uart_num(int num) { this->uart_num_ = num; }
  void set_baud_rate(uint32_t baud) { this->baud_ = baud; }
  void set_ppm(int ppm) { this->ppm_ = ppm; }
  void set_pulse_width_ms(uint32_t ms) { this->pulse_width_us_ = ms * 1000; }
  void set_nmea_delay_ms(uint32_t ms) { this->nmea_delay_us_ = ms * 1000; }
  void set_satellites(int sats) { this->satellites_ = sats; }

  // Fault injection, for template switches and buttons
  void set_pps_enabled(bool on) { this->pps_enabled_ = on; }
  void set_nmea_enabled(bool on) { this->nmea_enabled_ = on; }
  void set_fix(bool on) { this->fix_ = on; }
  void set_utc_valid(bool on) { this->utc_valid_ = on; }
  void set_answer_ubx(bool on) { this->answer_ubx_ = on; }
  void set_week_rollover(bool on) { this->week_rollover_ = on; }
  void inject_glitch() { this->glitch_requested_ = true; }  // a stray edge 50 ms after the next pulse
  void step_phase_us(int32_t us) { this->phase_step_us_ += us; }  // every later pulse moves by this much
  void step_time_s(int32_t s) { this->epoch_offset_s_ += s; }  // the reported time jumps; pulses don't

 protected:
  static void pulse_cb(void *arg);
  void start_();
  void send_epoch_(int64_t utc_s);
  void send_nmea_(const std::string &body);
  void send_ubx_(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len);
  void read_commands_();
  void handle_ubx_(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len);

  int pps_pin_{-1}, tx_pin_{-1}, rx_pin_{-1};
  int uart_num_{2};
  uint32_t baud_{9600};
  int ppm_{0};
  uint32_t pulse_width_us_{10000};
  uint32_t nmea_delay_us_{100000};
  int satellites_{9};

  bool pps_enabled_{true}, nmea_enabled_{true}, fix_{true}, utc_valid_{true}, answer_ubx_{true};
  bool week_rollover_{false};
  bool glitch_requested_{false};
  std::atomic<int32_t> phase_step_us_{0};
  int64_t deadline_us_{0};
  int32_t epoch_offset_s_{0};

  bool started_{false};
  esp_timer_handle_t timer_{nullptr};
  int64_t first_epoch_s_{0};

  // Written by the timer callback
  std::atomic<uint32_t> pulse_count_{0};
  std::atomic<int64_t> pulse_us_{0};
  uint32_t handled_pulse_{0};
  bool pin_high_{false};
  bool epoch_pending_{false};
  int64_t epoch_due_us_{0};
  int64_t glitch_due_us_{0};
  int64_t glitch_end_us_{0};

  // UBX command parser
  uint8_t rx_[64];
  uint16_t rx_len_{0};
  uint32_t saves_{0};
};

}  // namespace esphome::gnss_sim
