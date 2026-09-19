#pragma once

#include <atomic>
#include <cstdint>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"

namespace esphome::pps_ntp {

// Maps the local esp_timer clock (µs since boot) onto UTC, re-fitted on every accepted PPS pulse
struct ClockModel {
  bool valid{false};
  int64_t anchor_local_us{0};  // fitted local time of the most recent accepted pulse
  int64_t anchor_utc_s{0};     // UTC (unix seconds) of that pulse
  double local_us_per_s{1e6};  // local microseconds per true second (crystal rate)
  int64_t last_pulse_local_us{0};  // raw local time of the most recent accepted pulse
  bool utc_trusted{false};     // receiver confirmed UTC (leap seconds known)
};

class PPSNTPServer : public PollingComponent, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;

  void set_pps_pin(InternalGPIOPin *pin) { this->pps_pin_ = pin; }
  void set_port(uint16_t port) { this->port_ = port; }
  void set_holdover_s(uint32_t seconds) { this->holdover_us_ = static_cast<int64_t>(seconds) * 1000000LL; }
  void set_gnss_baud_rate(uint32_t baud) { this->gnss_baud_rate_ = baud; }

  void set_satellites_sensor(sensor::Sensor *s) { this->satellites_sensor_ = s; }
  void set_frequency_offset_sensor(sensor::Sensor *s) { this->frequency_offset_sensor_ = s; }
  void set_pps_jitter_sensor(sensor::Sensor *s) { this->pps_jitter_sensor_ = s; }
  void set_requests_sensor(sensor::Sensor *s) { this->requests_sensor_ = s; }
  void set_synced_binary_sensor(binary_sensor::BinarySensor *s) { this->synced_binary_sensor_ = s; }

 protected:
  static void pps_isr(PPSNTPServer *self);
  static void ntp_task(void *arg);
  void ntp_loop_();

  // GNSS input
  void feed_byte_(uint8_t byte);
  void handle_nmea_(char *line);
  void handle_rmc_(char **fields, int count);
  void handle_ubx_(uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t len);
  void send_ubx_(uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t len);
  void start_baud_switch_();
  void service_baud_switch_();

  // Clock discipline
  void handle_pulse_(int64_t local_us);
  void accept_pulse_(int64_t local_us, int64_t utc_s);
  void reset_discipline_(const char *reason);
  void publish_model_();
  ClockModel get_model_();
  bool is_synced_(const ClockModel &model, int64_t now_local_us) const;

  InternalGPIOPin *pps_pin_{nullptr};
  uint16_t port_{123};
  int64_t holdover_us_{900LL * 1000000LL};
  uint32_t gnss_baud_rate_{0};

  // Written by the PPS ISR
  volatile int64_t isr_pulse_us_{0};
  volatile uint32_t isr_pulse_count_{0};
  uint32_t seen_pulse_count_{0};

  // Pulses and labels
  int64_t last_pulse_us_{0};  // most recent raw pulse, labelled or not
  bool last_pulse_labelled_{false};
  int64_t last_rmc_valid_us_{0};  // local time of the last RMC with an 'A' fix
  uint8_t label_mismatches_{0};
  uint8_t outliers_{0};

  static constexpr int HISTORY_SIZE = 64;
  int64_t hist_local_[HISTORY_SIZE];
  int64_t hist_utc_[HISTORY_SIZE];
  int hist_count_{0};
  int hist_head_{0};  // index of the next write
  int64_t last_accepted_local_us_{0};
  int64_t last_accepted_utc_s_{0};
  double jitter_sq_us_{0};

  // Model shared with the NTP task
  portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;
  ClockModel model_;
  bool utc_trusted_{false};
  bool ubx_seen_{false};
  uint32_t boot_ms_{0};
  uint32_t last_ubx_poll_ms_{0};
  bool last_synced_{false};
  std::atomic<uint32_t> requests_{0};
  TaskHandle_t task_{nullptr};

  // Receiver parsing
  enum class RxState : uint8_t { IDLE, NMEA, UBX_SYNC2, UBX_HEADER, UBX_PAYLOAD };
  RxState rx_state_{RxState::IDLE};
  char nmea_[96];
  uint8_t nmea_len_{0};
  uint8_t ubx_[4 + 256 + 2];
  uint16_t ubx_len_{0};
  uint16_t ubx_expected_{0};
  int satellites_{-1};

  // Baud switching for legacy u-blox modules
  enum class BaudState : uint8_t { OFF, VERIFY, RETRY_WAIT, DONE };
  BaudState baud_state_{BaudState::OFF};
  uint32_t original_baud_{0};
  uint32_t baud_deadline_ms_{0};
  uint8_t baud_attempts_{0};
  bool nmea_ok_since_switch_{false};

  sensor::Sensor *satellites_sensor_{nullptr};
  sensor::Sensor *frequency_offset_sensor_{nullptr};
  sensor::Sensor *pps_jitter_sensor_{nullptr};
  sensor::Sensor *requests_sensor_{nullptr};
  binary_sensor::BinarySensor *synced_binary_sensor_{nullptr};
};

}  // namespace esphome::pps_ntp
