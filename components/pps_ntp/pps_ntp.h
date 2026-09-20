#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"

#ifdef USE_PPS_NTP_MCPWM
#include <driver/mcpwm_cap.h>
#endif

#ifdef USE_PPS_NTP_RAW_UDP
struct udp_pcb;
struct pbuf;
#include <lwip/ip_addr.h>
#endif

namespace esphome::pps_ntp {

// Maps the local esp_timer clock (µs since boot) onto UTC, re-fitted on every accepted PPS pulse
struct ClockModel {
  bool valid{false};
  int64_t anchor_local_us{0};  // fitted local time of the most recent accepted pulse
  int64_t anchor_utc_s{0};     // UTC (unix seconds) of that pulse
  double local_us_per_s{1e6};  // local microseconds per true second (crystal rate)
  int64_t last_pulse_local_us{0};  // raw local time of the most recent accepted pulse
  bool utc_trusted{false};     // receiver confirmed UTC (leap seconds known)
  // Leap seconds. The fit runs on a continuous count of seconds ("internal" time) so that history stays
  // linear across a leap. UTC = internal - leap_adj_s, and leap_adj_s changes by leap_change at leap_at_s.
  int32_t leap_adj_s{0};
  int8_t leap_change{0};      // +1 insertion, -1 deletion, 0 none pending
  int64_t leap_at_s{0};       // internal second at which the adjustment takes effect
  bool leap_announce{false};  // known in advance (UBX-NAV-TIMELS) and due within a day: set LI for clients
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
  void set_fit_window(int pulses) { this->fit_window_ = pulses; }
  void set_max_residual_us(double us) { this->max_residual_us_ = us; }
  void set_refid(const char *refid) { strncpy(this->refid_, refid, sizeof(this->refid_)); }
  void set_require_utc_valid(bool require) { this->require_utc_valid_ = require; }

  void set_satellites_sensor(sensor::Sensor *s) { this->satellites_sensor_ = s; }
  void set_signal_strength_sensor(sensor::Sensor *s) { this->signal_strength_sensor_ = s; }
  void set_strong_satellites_sensor(sensor::Sensor *s) { this->strong_satellites_sensor_ = s; }
  void set_hdop_sensor(sensor::Sensor *s) { this->hdop_sensor_ = s; }
  void set_rejected_pulses_sensor(sensor::Sensor *s) { this->rejected_pulses_sensor_ = s; }
  void set_nmea_errors_sensor(sensor::Sensor *s) { this->nmea_errors_sensor_ = s; }
  void set_pulse_age_sensor(sensor::Sensor *s) { this->pulse_age_sensor_ = s; }
  void set_strong_threshold(int dbhz) { this->strong_threshold_ = dbhz; }
  void set_stationary(bool on) { this->stationary_ = on; }
  void set_trim_nmea(bool on) { this->trim_nmea_ = on; }
  void set_frequency_offset_sensor(sensor::Sensor *s) { this->frequency_offset_sensor_ = s; }
  void set_pps_jitter_sensor(sensor::Sensor *s) { this->pps_jitter_sensor_ = s; }
  void set_requests_sensor(sensor::Sensor *s) { this->requests_sensor_ = s; }
  void set_synced_binary_sensor(binary_sensor::BinarySensor *s) { this->synced_binary_sensor_ = s; }

 protected:
  static void pps_isr(PPSNTPServer *self);
  static void ntp_task(void *arg);
  void ntp_loop_();
  bool start_server_();
  // Fills a 48-byte reply for a 48-byte (or longer) request; false means stay silent
  bool build_reply_(const uint8_t *request, int64_t rx_local_us, uint8_t *reply);
#ifdef USE_PPS_NTP_RAW_UDP
  // EXPERIMENTAL: serve from lwIP's tcpip thread, skipping the socket mailbox and task wake-up
  bool start_raw_udp_();
  static void raw_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, uint16_t port);
  struct udp_pcb *pcb_{nullptr};
#endif

  // PPS capture
  bool setup_capture_();
  void poll_capture_();
  void poll_isr_();

  // GNSS input
  void feed_byte_(uint8_t byte);
  void handle_nmea_(char *line);
  void handle_rmc_(char **fields, int count);
  void handle_gsv_(char **fields, int count);
  void finalize_cno_();
  void handle_ubx_(uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t len);
  void handle_cfg_gnss_(const uint8_t *payload, uint16_t len);
  void configure_receiver_();
  void send_ubx_(uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t len);
  void start_baud_switch_();
  void service_baud_switch_();

  // Clock discipline
  void handle_pulse_(int64_t local_us);
  void accept_pulse_(int64_t local_us, int64_t utc_s);
  void reset_discipline_(const char *reason);
  void log_status_();
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

  // MCPWM hardware capture: the edge time is latched by the peripheral, so interrupt latency and
  // flash-write stalls can't move it
  bool hw_capture_{false};
#ifdef USE_PPS_NTP_MCPWM
  mcpwm_cap_timer_handle_t cap_timer_{nullptr};
  mcpwm_cap_channel_handle_t cap_pps_{nullptr};  // latches on the PPS rising edge
  mcpwm_cap_channel_handle_t cap_ref_{nullptr};  // software-latched to tie capture ticks to esp_timer
  double cap_ticks_per_us_{80.0};
  uint32_t last_cap_value_{0};
  int cap_group_{-1};
  portMUX_TYPE ref_lock_ = portMUX_INITIALIZER_UNLOCKED;
#endif

  // Pulses and labels
  int64_t last_pulse_us_{0};     // most recent plausible pulse: accepted, or a candidate waiting for its RMC
  int64_t last_pulse_utc_s_{0};  // the label given to that pulse
  bool last_pulse_labelled_{false};
  uint8_t off_second_edges_{0};     // consecutive edges that weren't a whole number of seconds after the last pulse
  uint8_t label_confirmations_{0};  // RMC sentences that agreed with the pulse count since the last reset
  int64_t last_rmc_valid_us_{0};  // local time of the last RMC with an 'A' fix
  uint8_t label_mismatches_{0};
  uint8_t outliers_{0};

  int fit_window_{64};
  double max_residual_us_{1000.0};
  char refid_[4]{'G', 'P', 'S', '\0'};  // NTP refid: up to 4 ASCII chars, zero-padded, not NUL-terminated
  std::vector<int64_t> hist_local_;
  std::vector<int64_t> hist_utc_;
  int hist_count_{0};
  int hist_head_{0};  // index of the next write
  int64_t last_accepted_local_us_{0};
  int64_t last_accepted_utc_s_{0};
  double jitter_sq_us_{0};
  double last_residual_us_{0};

  // Diagnostics, reported by update()
  uint32_t edges_seen_{0};
  uint32_t pulses_accepted_{0};
  uint32_t nmea_ok_{0};
  uint32_t nmea_bad_{0};
  uint32_t ubx_frames_{0};
  uint32_t status_edges_{0};  // edges_seen_ / nmea_ok_ at the previous update(), to spot a dead input
  uint32_t status_nmea_{0};
  bool rollover_warned_{false};

  // Model shared with the NTP task
  portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;
  ClockModel model_;
  bool utc_trusted_{false};
  bool ubx_seen_{false};       // any UBX frame at all, for diagnostics
  bool timeutc_seen_{false};   // a NAV-TIMEUTC reply specifically: the only message that can confirm UTC
  // Leap second state, in the loop task; copied into the model on every publish
  int32_t leap_adj_s_{0};
  int8_t leap_change_{0};
  int64_t leap_midnight_unix_{0};  // the UTC midnight the leap belongs to (the 00:00:00 that follows it)
  bool leap_from_ubx_{false};
  uint32_t last_timels_poll_ms_{0};
  int64_t leap_at_internal_() const {
    return this->leap_midnight_unix_ + this->leap_adj_s_ - (this->leap_change_ < 0 ? 1 : 0);
  }
  void fill_leap_(ClockModel &model) const;
  bool require_utc_valid_{false};
  bool stationary_{true};   // tell the receiver it isn't moving (UBX-CFG-NAV5)
  bool trim_nmea_{false};   // silence the sentences we don't read (UBX-CFG-MSG)
  bool receiver_configured_{false};
  bool ubx_absent_warned_{false};
  bool server_started_{false};
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
  uint32_t ubx_len_{0};
  uint32_t ubx_expected_{0};  // 32-bit: 4 + a 16-bit length + 2 doesn't fit in 16
  int satellites_{-1};
  // Mean carrier-to-noise density over the satellites the receiver is tracking, accumulated across one
  // epoch's GSV sentences (every constellation) and finalised when the next RMC starts the following epoch
  float cno_mean_{0};
  bool cno_valid_{false};
  uint32_t cno_sum_{0};
  uint16_t cno_count_{0};
  bool cno_saw_gsv_{false};
  // Satellites at or above strong_threshold_: a better guide to antenna placement than the mean, which
  // marginal satellites drag down
  int strong_threshold_{35};
  uint16_t strong_accum_{0};
  int strong_satellites_{-1};
  float hdop_{0};  // horizontal dilution of precision, from GGA: the geometry half of fix quality
  bool hdop_valid_{false};

  // Baud switching for legacy u-blox modules
  enum class BaudState : uint8_t { OFF, PROBE, VERIFY, RETRY_WAIT, DONE };
  BaudState baud_state_{BaudState::OFF};
  uint32_t original_baud_{0};
  uint32_t baud_deadline_ms_{0};
  uint8_t baud_attempts_{0};
  bool nmea_ok_since_switch_{false};

  sensor::Sensor *satellites_sensor_{nullptr};
  sensor::Sensor *signal_strength_sensor_{nullptr};
  sensor::Sensor *strong_satellites_sensor_{nullptr};
  sensor::Sensor *hdop_sensor_{nullptr};
  sensor::Sensor *rejected_pulses_sensor_{nullptr};
  sensor::Sensor *nmea_errors_sensor_{nullptr};
  sensor::Sensor *pulse_age_sensor_{nullptr};
  sensor::Sensor *frequency_offset_sensor_{nullptr};
  sensor::Sensor *pps_jitter_sensor_{nullptr};
  sensor::Sensor *requests_sensor_{nullptr};
  binary_sensor::BinarySensor *synced_binary_sensor_{nullptr};
};

}  // namespace esphome::pps_ntp
