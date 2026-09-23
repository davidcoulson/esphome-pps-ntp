#include "pps_ntp.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

#include <sdkconfig.h>
#include <esp_attr.h>
#include <esp_timer.h>

#ifdef USE_PPS_NTP_MCPWM
// esp_timer's own counter, before its divide down to microseconds: 16 ticks/µs on the S3 and P4
extern "C" uint64_t esp_timer_impl_get_counter_reg(void);
#endif
#include <lwip/sockets.h>
#include <unistd.h>

#include "esphome/components/network/util.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#ifdef USE_PPS_NTP_RAW_UDP
#include <lwip/pbuf.h>
#include <lwip/udp.h>
#endif

#ifdef USE_ESP32
#include <lwip/etharp.h>
#include <lwip/ip4_addr.h>
#include <lwip/netif.h>
#endif

#ifdef USE_ETHERNET
#include "esphome/components/ethernet/ethernet_component.h"
#endif

#if defined(USE_PPS_NTP_TASK_CORE) && CONFIG_FREERTOS_NUMBER_OF_CORES < 2
#error "pps_ntp: task_core is set, but this build has a single FreeRTOS core"
#endif

namespace esphome::pps_ntp {

static const char *const TAG = "pps_ntp";

static constexpr int MIN_PULSES_FOR_SYNC = 4;
static constexpr uint8_t MIN_LABEL_CONFIRMATIONS = 3;  // RMCs that must agree with the pulse count before serving
static constexpr uint8_t MAX_OFF_SECOND_EDGES = 3;     // consecutive misaligned edges before re-labelling
static constexpr int64_t MIN_VALID_UTC_S = 1767225600;       // 2026-01-01: nothing older can be a live fix
static constexpr int64_t GPS_WEEK_ROLLOVER_S = 619315200;    // 1024 weeks
static constexpr uint32_t BAUD_PROBE_MS = 1500;
static constexpr double MAX_RATE_ERROR_PPM = 500.0;   // reject fits that imply a broken crystal
static constexpr int64_t MAX_PULSE_GAP_S = 600;       // longer gaps are relabelled from NMEA
static constexpr int64_t FIX_STALE_US = 3000000;      // pulses only count while the receiver reports a fix
static constexpr int64_t RMC_MAX_DELAY_US = 950000;   // RMC must follow its pulse within this window
static constexpr uint32_t UBX_POLL_INTERVAL_MS = 10000;
static constexpr uint32_t UBX_POLL_FAST_MS = 2000;
static constexpr uint32_t TIMELS_POLL_MS = 60000;
static constexpr int64_t LEAP_ANNOUNCE_S = 86400;  // NTP convention: set LI during the day that ends with the leap
static constexpr uint32_t UBX_ABSENT_TIMEOUT_MS = 60000;
static constexpr uint32_t BAUD_VERIFY_MS = 4000;
static constexpr uint32_t BAUD_RETRY_MS = 5000;
static constexpr uint8_t BAUD_MAX_ATTEMPTS = 3;
static constexpr uint64_t NTP_UNIX_OFFSET = 2208988800ULL;  // seconds from 1900 to 1970
static constexpr double HOLDOVER_DRIFT_PPM = 5.0;
static constexpr int64_t RX_STAMP_MAX_AGE_US = 50000;  // a driver stamp older than this belongs to someone else
static constexpr uint32_t ARP_REFRESH_MS = 120000;     // lwIP drops ARP entries after 300 s
static constexpr uint32_t ARP_CLIENT_IDLE_MS = 7200000;  // stop priming a client silent this long (> max poll 1024 s)

// ---------------------------------------------------------------------------
// Frame parsing for the driver-level receive hook

static uint16_t get_be16(const uint8_t *p) { return static_cast<uint16_t>((p[0] << 8) | p[1]); }

bool parse_ntp_request_frame(const uint8_t *frame, uint32_t length, uint16_t port, uint8_t *key) {
  static constexpr uint32_t NTP_LEN = 48;
  if (length < 14)
    return false;
  uint32_t off = 12;
  uint16_t ethertype = get_be16(frame + off);
  if (ethertype == 0x8100) {  // one 802.1Q tag
    off += 4;
    if (length < off + 2)
      return false;
    ethertype = get_be16(frame + off);
  }
  off += 2;

  const uint8_t *udp;
  if (ethertype == 0x0800) {
    if (length < off + 20)
      return false;
    const uint8_t *ip = frame + off;
    uint32_t ihl = (ip[0] & 0x0F) * 4u;
    if ((ip[0] >> 4) != 4 || ihl < 20 || ip[9] != 17)
      return false;
    if ((get_be16(ip + 6) & 0x3FFF) != 0)
      return false;  // a fragment: only the first carries the UDP header, and NTP requests are never this big
    off += ihl;
  } else if (ethertype == 0x86DD) {
    if (length < off + 40)
      return false;
    const uint8_t *ip = frame + off;
    if ((ip[0] >> 4) != 6 || ip[6] != 17)
      return false;  // extension headers: not worth walking for NTP
    off += 40;
  } else {
    return false;
  }
  if (length < off + 8 + NTP_LEN)
    return false;
  udp = frame + off;
  if (get_be16(udp + 2) != port)
    return false;
  const uint8_t *ntp = udp + 8;
  if ((ntp[0] & 0x07) != 3)
    return false;
  memcpy(key, ntp + 40, 8);
  key[8] = udp[0];  // source port, as sent
  key[9] = udp[1];
  return true;
}

// Days since 1970-01-01 for a proleptic Gregorian date (Howard Hinnant's algorithm)
static int64_t days_from_civil(int y, int m, int d) {
  y -= m <= 2;
  const int era = (y >= 0 ? y : y - 399) / 400;
  const int yoe = y - era * 400;
  const int doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return static_cast<int64_t>(era) * 146097 + doe - 719468;
}

static int two_digits(const char *p) { return (p[0] - '0') * 10 + (p[1] - '0'); }

static bool all_digits(const char *p, int n) {
  for (int i = 0; i < n; i++) {
    if (p[i] < '0' || p[i] > '9')
      return false;
  }
  return true;
}

// Internal (continuous) microseconds -> UTC. During an inserted second this repeats 23:59:59, which is
// how NTP servers conventionally represent 23:59:60.
static int64_t internal_to_utc_us(const ClockModel &model, int64_t internal_us) {
  int64_t adj = model.leap_adj_s;
  if (model.leap_change != 0 && internal_us >= model.leap_at_s * 1000000LL)
    adj += model.leap_change;
  return internal_us - adj * 1000000LL;
}

static int64_t internal_us_at(const ClockModel &model, int64_t local_us) {
  double elapsed = (static_cast<double>(local_us) - model.anchor_local_us) * (1e6 / model.local_us_per_s);
  return model.anchor_utc_s * 1000000LL + llround(elapsed);
}

static int64_t utc_us_at(const ClockModel &model, int64_t local_us) {
  return internal_to_utc_us(model, internal_us_at(model, local_us));
}

static void put_be32(uint8_t *p, uint32_t v) {
  p[0] = v >> 24;
  p[1] = v >> 16;
  p[2] = v >> 8;
  p[3] = v;
}

static void put_ntp_timestamp(uint8_t *p, int64_t unix_us) {
  uint64_t seconds = static_cast<uint64_t>(unix_us / 1000000) + NTP_UNIX_OFFSET;
  uint64_t micros = static_cast<uint64_t>(unix_us % 1000000);
  put_be32(p, static_cast<uint32_t>(seconds));
  put_be32(p + 4, static_cast<uint32_t>((micros << 32) / 1000000));
}

void IRAM_ATTR PPSNTPServer::pps_isr(PPSNTPServer *self) {
  self->isr_pulse_us_ = esp_timer_get_time();
  self->isr_pulse_count_ = self->isr_pulse_count_ + 1;
}

void PPSNTPServer::setup() {
  this->boot_ms_ = millis();
  this->measure_precision_();
  this->hist_local_.assign(this->fit_window_, 0);
  this->hist_utc_.assign(this->fit_window_, 0);
  // Always apply the YAML pin mode (pull-up/down); MCPWM only routes the pad to its capture input
  this->pps_pin_->setup();
  this->hw_capture_ = this->setup_capture_();
#ifdef USE_PPS_NTP_MCPWM
  if (this->hw_capture_) {
    // Ratio of esp_timer's raw counter to its microseconds (an exact integer): lets the reference latch
    // be placed to a fraction of a microsecond instead of the ±0.5 µs of a whole-µs read
    uint64_t raw0 = esp_timer_impl_get_counter_reg();
    int64_t us0 = esp_timer_get_time();
    delay(20);
    uint64_t raw1 = esp_timer_impl_get_counter_reg();
    int64_t us1 = esp_timer_get_time();
    this->systimer_ticks_per_us_ = std::round(static_cast<double>(raw1 - raw0) / static_cast<double>(us1 - us0));
  }
#endif
  if (!this->hw_capture_)
    this->pps_pin_->attach_interrupt(PPSNTPServer::pps_isr, this, gpio::INTERRUPT_RISING_EDGE);

  this->original_baud_ = this->parent_->get_baud_rate();
  if (this->gnss_baud_rate_ != 0 && this->gnss_baud_rate_ != this->original_baud_) {
    // The receiver keeps its baud across our reboots (and in BBR/flash), so listen at the target first.
    // Only if that stays quiet do we send CFG-PRT at the original baud and save the result.
    this->parent_->set_baud_rate(this->gnss_baud_rate_);
    this->parent_->load_settings(false);
    this->nmea_ok_since_switch_ = false;
    this->baud_deadline_ms_ = millis() + BAUD_PROBE_MS;
    this->baud_state_ = BaudState::PROBE;
  }
}

void PPSNTPServer::loop() {
  // The socket API needs lwIP's tcpip thread, which the network component only brings up after
  // our setup(); calling socket() earlier asserts on an uninitialised lwIP mutex
  if (!this->server_started_ && network::is_connected()) {
    if (!this->start_server_()) {
      this->mark_failed();
      return;
    }
    this->server_started_ = true;
#ifdef USE_ETHERNET
    if (this->install_rx_hook_requested_)
      this->rx_hook_active_ = this->install_rx_hook_();
#endif
  }
  if (this->server_started_ && millis() - this->arp_last_refresh_ms_ >= ARP_REFRESH_MS) {
    this->arp_last_refresh_ms_ = millis();
    this->refresh_arp_();
  }

  // Pulses before NMEA: the RMC that labels a pulse must find that pulse already recorded
  if (this->hw_capture_) {
    this->poll_capture_();
  } else {
    this->poll_isr_();
  }

  uint8_t byte;
  while (this->available() > 0 && this->read_byte(&byte))
    this->feed_byte_(byte);

  this->service_baud_switch_();

  // Once the link is settled and sentences are arriving, set the receiver up for a fixed installation
  if (!this->receiver_configured_ && this->nmea_ok_ > 0 &&
      (this->baud_state_ == BaudState::OFF || this->baud_state_ == BaudState::DONE))
    this->configure_receiver_();

  uint32_t now_ms = millis();
  // Poll quickly until UTC is confirmed, so a warm receiver isn't held at stratum 16 for a full interval
  if (now_ms - this->last_ubx_poll_ms_ >= (this->utc_trusted_ ? UBX_POLL_INTERVAL_MS : UBX_POLL_FAST_MS)) {
    this->last_ubx_poll_ms_ = now_ms;
    this->send_ubx_(0x01, 0x21, nullptr, 0);  // poll UBX-NAV-TIMEUTC for the validUTC flag
  }
  // Leap-second schedule (u-blox 8 and later; older receivers don't answer, and the leap is then taken from NMEA)
  if (this->utc_trusted_ && now_ms - this->last_timels_poll_ms_ >= TIMELS_POLL_MS) {
    this->last_timels_poll_ms_ = now_ms;
    this->send_ubx_(0x01, 0x26, nullptr, 0);
  }
  // Once a leap is well behind us, fold it into the running adjustment
  if (this->leap_change_ != 0 && this->hist_count_ > 0 && this->last_accepted_utc_s_ > this->leap_at_internal_() + 10) {
    this->leap_adj_s_ += this->leap_change_;
    this->leap_change_ = 0;
    this->leap_from_ubx_ = false;
    this->publish_model_();
    ESP_LOGI(TAG, "Leap second complete");
  }

  if (!this->timeutc_seen_ && !this->utc_trusted_ && !this->ubx_absent_warned_ &&
      now_ms - this->boot_ms_ >= UBX_ABSENT_TIMEOUT_MS) {
    this->ubx_absent_warned_ = true;
    if (this->require_utc_valid_) {
      ESP_LOGW(TAG, "Receiver does not answer NAV-TIMEUTC and require_utc_valid is set; staying unsynchronised");
    } else {
      ESP_LOGW(TAG, "Receiver does not answer NAV-TIMEUTC; trusting NMEA UTC without leap-second confirmation");
      this->utc_trusted_ = true;
      this->publish_model_();
    }
  }

  bool synced = this->is_synced_(this->model_, esp_timer_get_time());
  if (synced != this->last_synced_) {
    this->last_synced_ = synced;
    ESP_LOGI(TAG, "%s", synced ? "Synchronised to GNSS PPS; serving stratum 1" : "Lost synchronisation; serving stratum 16");
    if (this->synced_binary_sensor_ != nullptr)
      this->synced_binary_sensor_->publish_state(synced);
  }
}

// ---------------------------------------------------------------------------
// PPS capture

bool PPSNTPServer::setup_capture_() {
#ifdef USE_PPS_NTP_MCPWM
  mcpwm_capture_timer_config_t timer_config = {};
  timer_config.clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT;
  for (int group = 0; group < 2 && this->cap_timer_ == nullptr; group++) {
    timer_config.group_id = group;
    if (mcpwm_new_capture_timer(&timer_config, &this->cap_timer_) == ESP_OK)
      this->cap_group_ = group;
  }
  if (this->cap_timer_ == nullptr) {
    ESP_LOGW(TAG, "No free MCPWM capture timer; falling back to the GPIO interrupt");
    return false;
  }

  mcpwm_capture_channel_config_t channel_config = {};
  channel_config.gpio_num = this->pps_pin_->get_pin();
  channel_config.prescale = 1;
  channel_config.flags.pos_edge = true;
  channel_config.flags.invert_cap_signal = this->pps_pin_->is_inverted();  // `inverted: true` = active-low PPS
  esp_err_t err = mcpwm_new_capture_channel(this->cap_timer_, &channel_config, &this->cap_pps_);
  if (err == ESP_OK) {
    channel_config.gpio_num = -1;  // software trigger only
    channel_config.flags.invert_cap_signal = false;
    err = mcpwm_new_capture_channel(this->cap_timer_, &channel_config, &this->cap_ref_);
  }
  if (err == ESP_OK && this->rx_reference_pin_ >= 0) {
    // The Ethernet chip's interrupt line (active low), read through the GPIO matrix alongside the
    // driver's own interrupt on the same pad. Optional: a failure here only loses the diagnostic.
    mcpwm_capture_channel_config_t int_config = {};
    int_config.gpio_num = this->rx_reference_pin_;
    int_config.prescale = 1;
    int_config.flags.neg_edge = true;
    if (mcpwm_new_capture_channel(this->cap_timer_, &int_config, &this->cap_int_) != ESP_OK) {
      ESP_LOGW(TAG, "No MCPWM capture channel for rx_reference_pin; interrupt lead unavailable");
      this->cap_int_ = nullptr;
    } else if (mcpwm_capture_channel_enable(this->cap_int_) != ESP_OK) {
      mcpwm_del_capture_channel(this->cap_int_);
      this->cap_int_ = nullptr;
    }
  }
  // No callbacks are registered, so the driver never installs an interrupt; loop() polls the latches
  bool pps_enabled = false, ref_enabled = false, timer_enabled = false, timer_started = false;
  if (err == ESP_OK)
    pps_enabled = (err = mcpwm_capture_channel_enable(this->cap_pps_)) == ESP_OK;
  if (err == ESP_OK)
    ref_enabled = (err = mcpwm_capture_channel_enable(this->cap_ref_)) == ESP_OK;
  if (err == ESP_OK)
    timer_enabled = (err = mcpwm_capture_timer_enable(this->cap_timer_)) == ESP_OK;
  if (err == ESP_OK)
    timer_started = (err = mcpwm_capture_timer_start(this->cap_timer_)) == ESP_OK;
  uint32_t resolution_hz = 0;
  if (err == ESP_OK)
    err = mcpwm_capture_timer_get_resolution(this->cap_timer_, &resolution_hz);
  if (err != ESP_OK || resolution_hz == 0) {
    ESP_LOGW(TAG, "MCPWM capture setup failed (%s); falling back to the GPIO interrupt", esp_err_to_name(err));
    // The driver refuses to delete anything that is still enabled
    if (timer_started)
      mcpwm_capture_timer_stop(this->cap_timer_);
    if (timer_enabled)
      mcpwm_capture_timer_disable(this->cap_timer_);
    if (ref_enabled)
      mcpwm_capture_channel_disable(this->cap_ref_);
    if (pps_enabled)
      mcpwm_capture_channel_disable(this->cap_pps_);
    if (this->cap_pps_ != nullptr)
      mcpwm_del_capture_channel(this->cap_pps_);
    if (this->cap_ref_ != nullptr)
      mcpwm_del_capture_channel(this->cap_ref_);
    if (this->cap_int_ != nullptr) {
      mcpwm_capture_channel_disable(this->cap_int_);
      mcpwm_del_capture_channel(this->cap_int_);
    }
    mcpwm_del_capture_timer(this->cap_timer_);
    this->cap_pps_ = this->cap_ref_ = this->cap_int_ = nullptr;
    this->cap_timer_ = nullptr;
    return false;
  }
  this->cap_ticks_per_us_ = resolution_hz / 1e6;
  mcpwm_capture_get_latched_value(this->cap_pps_, &this->last_cap_value_);
  if (this->cap_int_ != nullptr)
    mcpwm_capture_get_latched_value(this->cap_int_, &this->last_int_value_);
  return true;
#else
  return false;
#endif
}

void PPSNTPServer::poll_capture_() {
#ifdef USE_PPS_NTP_MCPWM
  uint32_t pulse_ticks;
  mcpwm_capture_get_latched_value(this->cap_pps_, &pulse_ticks);
  if (pulse_ticks == this->last_cap_value_)
    return;
  this->last_cap_value_ = pulse_ticks;

  uint32_t ref_ticks;
  double ref_us;
  this->sample_ref_(&ref_ticks, &ref_us);

  // The 32-bit counter wraps every ~53 s (80 MHz); the signed difference is valid for ~26 s
  int32_t ticks_since_edge = static_cast<int32_t>(ref_ticks - pulse_ticks);
  this->handle_pulse_(ref_us - ticks_since_edge / this->cap_ticks_per_us_);
#endif
}

void PPSNTPServer::sample_ref_(uint32_t *ticks, double *us) {
#ifdef USE_PPS_NTP_MCPWM
  // Latch the capture timer "now" between two reads of esp_timer's raw counter, with nothing able to
  // preempt us, to place captured edges on the esp_timer timeline. The raw counter (16 MHz) places the
  // latch to ~60 ns; esp_timer_get_time() would round each read to a whole microsecond. The lock also
  // keeps the loop and the Ethernet driver task from latching over each other's reading.
  portENTER_CRITICAL(&this->ref_lock_);
  uint64_t before = esp_timer_impl_get_counter_reg();
  mcpwm_capture_channel_trigger_soft_catch(this->cap_ref_);
  uint64_t after = esp_timer_impl_get_counter_reg();
  mcpwm_capture_get_latched_value(this->cap_ref_, ticks);
  portEXIT_CRITICAL(&this->ref_lock_);
  *us = static_cast<double>(before + after) * 0.5 / this->systimer_ticks_per_us_;
#endif
}

// Driver task. Places the most recent falling edge of the Ethernet chip's interrupt line on the esp_timer
// timeline and records how long before the driver hook it happened: the part of the receive path (SPI
// read-out, task wake-up) that even the driver-level stamp can't see.
void PPSNTPServer::note_interrupt_lead_(int64_t hook_us) {
#ifdef USE_PPS_NTP_MCPWM
  if (this->cap_int_ == nullptr)
    return;
  uint32_t int_ticks;
  mcpwm_capture_get_latched_value(this->cap_int_, &int_ticks);
  if (int_ticks == this->last_int_value_)
    return;  // no new edge: the line was already low for an earlier frame
  this->last_int_value_ = int_ticks;
  uint32_t ref_ticks;
  double ref_us;
  this->sample_ref_(&ref_ticks, &ref_us);
  double edge_us = ref_us - static_cast<int32_t>(ref_ticks - int_ticks) / this->cap_ticks_per_us_;
  double lead = static_cast<double>(hook_us) - edge_us;
  if (lead < 0 || lead > 20000)
    return;  // not this frame's edge
  this->int_lead_sum_us_ += static_cast<int32_t>(lead);
  this->int_lead_count_++;
#endif
}

void PPSNTPServer::poll_isr_() {
  // Take the latest pulse from the ISR; re-read if one landed while copying the 64-bit timestamp
  uint32_t count = this->isr_pulse_count_;
  if (count == this->seen_pulse_count_)
    return;
  int64_t pulse_us = this->isr_pulse_us_;
  while (count != this->isr_pulse_count_) {
    count = this->isr_pulse_count_;
    pulse_us = this->isr_pulse_us_;
  }
  this->seen_pulse_count_ = count;
  this->handle_pulse_(pulse_us);
}

// RFC 5905 precision: log2 of the time it takes to read the clock, taken as the smallest step between two
// consecutive reads that differ (esp_timer counts whole microseconds, so this is normally 2^-20 s)
void PPSNTPServer::measure_precision_() {
  int64_t smallest = INT64_MAX;
  int64_t previous = esp_timer_get_time();
  for (int i = 0; i < 2000; i++) {
    int64_t now = esp_timer_get_time();
    if (now != previous && now - previous < smallest)
      smallest = now - previous;
    previous = now;
  }
  if (smallest == INT64_MAX)
    smallest = 1;
  this->precision_ = static_cast<int8_t>(std::floor(std::log2(static_cast<double>(smallest) * 1e-6)));
}

void PPSNTPServer::update() {
  ClockModel model = this->get_model_();
  if (this->satellites_sensor_ != nullptr && this->satellites_ >= 0)
    this->satellites_sensor_->publish_state(this->satellites_);
  if (this->signal_strength_sensor_ != nullptr && this->cno_valid_)
    this->signal_strength_sensor_->publish_state(this->cno_mean_);
  if (this->strong_satellites_sensor_ != nullptr && this->strong_satellites_ >= 0)
    this->strong_satellites_sensor_->publish_state(this->strong_satellites_);
  if (this->hdop_sensor_ != nullptr && this->hdop_valid_)
    this->hdop_sensor_->publish_state(this->hdop_);
  if (this->rejected_pulses_sensor_ != nullptr)
    this->rejected_pulses_sensor_->publish_state(
        this->edges_seen_ >= this->pulses_accepted_ ? this->edges_seen_ - this->pulses_accepted_ : 0);
  if (this->nmea_errors_sensor_ != nullptr)
    this->nmea_errors_sensor_->publish_state(this->nmea_bad_);
  if (this->pulse_age_sensor_ != nullptr && model.valid) {
    this->pulse_age_sensor_->publish_state((esp_timer_get_time() - model.last_pulse_local_us) / 1e6);
  }
  if (this->frequency_offset_sensor_ != nullptr && model.valid)
    this->frequency_offset_sensor_->publish_state(model.local_us_per_s - 1e6);
  if (this->pps_jitter_sensor_ != nullptr && model.valid)
    this->pps_jitter_sensor_->publish_state(std::sqrt(this->jitter_sq_us_));
  // A float holds integers exactly up to 2^24; wrap there and let total_increasing treat it as a meter reset
  if (this->requests_sensor_ != nullptr)
    this->requests_sensor_->publish_state(this->requests_.load() & 0xFFFFFF);
  if (this->synced_binary_sensor_ != nullptr)
    this->synced_binary_sensor_->publish_state(this->last_synced_);
  // Interval averages; a quiet interval publishes nothing rather than a misleading zero
  uint32_t gain_n = this->rx_gain_count_.exchange(0);
  int32_t gain_sum = this->rx_gain_sum_us_.exchange(0);
  if (this->rx_timestamp_gain_sensor_ != nullptr && gain_n > 0)
    this->rx_timestamp_gain_sensor_->publish_state(static_cast<float>(gain_sum) / gain_n);
  uint32_t lead_n = this->int_lead_count_.exchange(0);
  int32_t lead_sum = this->int_lead_sum_us_.exchange(0);
  if (this->rx_interrupt_lead_sensor_ != nullptr && lead_n > 0)
    this->rx_interrupt_lead_sensor_->publish_state(static_cast<float>(lead_sum) / lead_n);
  if (this->arp_clients_sensor_ != nullptr) {
    int active = 0;
    for (auto &client : this->arp_clients_)
      active += client.load() != 0;
    this->arp_clients_sensor_->publish_state(active);
  }
  this->log_status_();
}

void PPSNTPServer::log_status_() {
  // A dead input is otherwise silent: nothing arrives, so nothing else would ever log
  if (this->edges_seen_ == this->status_edges_)
    ESP_LOGW(TAG, "No PPS edges since the last update; check the PPS wiring and that the receiver has a fix");
  if (this->nmea_ok_ == this->status_nmea_ && this->baud_state_ != BaudState::PROBE &&
      this->baud_state_ != BaudState::VERIFY && this->baud_state_ != BaudState::RETRY_WAIT) {
    ESP_LOGW(TAG, "No valid NMEA since the last update (%u checksum failures so far); check TX/RX wiring and baud",
             static_cast<unsigned>(this->nmea_bad_));
  }
  this->status_edges_ = this->edges_seen_;
  this->status_nmea_ = this->nmea_ok_;

  char stack_free[12];
  if (this->task_ != nullptr) {
    snprintf(stack_free, sizeof(stack_free), "%u", static_cast<unsigned>(uxTaskGetStackHighWaterMark(this->task_)));
  } else {
    strcpy(stack_free, "n/a");  // the raw lwIP transport runs in the tcpip thread, with no task of ours
  }
  ESP_LOGD(TAG,
           "edges=%u accepted=%u nmea=%u/%u bad ubx=%u utc_valid=%s fit=%d/%d confirmed=%u residual=%.1fus "
           "jitter=%.1fus sats=%d cno=%.1f strong=%d hdop=%.1f stack_free=%s rx_hook=%u/%u arp=%u",
           static_cast<unsigned>(this->edges_seen_), static_cast<unsigned>(this->pulses_accepted_),
           static_cast<unsigned>(this->nmea_ok_), static_cast<unsigned>(this->nmea_bad_),
           static_cast<unsigned>(this->ubx_frames_), YESNO(this->utc_trusted_), this->hist_count_, this->fit_window_,
           static_cast<unsigned>(this->label_confirmations_), this->last_residual_us_, std::sqrt(this->jitter_sq_us_),
           this->satellites_, this->cno_mean_, this->strong_satellites_, this->hdop_, stack_free,
           static_cast<unsigned>(this->rx_hook_hits_.load()), static_cast<unsigned>(this->rx_hook_misses_.load()),
           static_cast<unsigned>(this->arp_requests_));
}

void PPSNTPServer::dump_config() {
  ESP_LOGCONFIG(TAG, "PPS NTP Server:");
  LOG_PIN("  PPS Pin: ", this->pps_pin_);
#ifdef USE_PPS_NTP_MCPWM
  if (this->hw_capture_) {
    ESP_LOGCONFIG(TAG, "  PPS capture: MCPWM hardware (group %d, %.0f MHz)", this->cap_group_, this->cap_ticks_per_us_);
  } else {
    ESP_LOGCONFIG(TAG, "  PPS capture: GPIO interrupt (MCPWM unavailable)");
  }
#else
  ESP_LOGCONFIG(TAG, "  PPS capture: GPIO interrupt");
#endif
  ESP_LOGCONFIG(TAG,
                "  Port: %u\n"
                "  Holdover: %us\n"
                "  Fit window: %d pulses\n"
                "  Max residual: %.0f µs\n"
                "  Strong signal threshold: %d dB-Hz\n"
                "  Stationary model: %s\n"
                "  Trim NMEA output: %s\n"
                "  Refid: %.4s\n"
                "  Require UTC valid: %s\n"
                "  Root dispersion: %.0f µs\n"
                "  Rx/Tx delay compensation: %.0f / %.0f µs\n"
                "  Precision: 2^%d s",
                this->port_, static_cast<unsigned>(this->holdover_us_ / 1000000), this->fit_window_,
                this->max_residual_us_, this->strong_threshold_, YESNO(this->stationary_),
                YESNO(this->trim_nmea_), this->refid_, YESNO(this->require_utc_valid_), this->root_dispersion_us_,
                this->rx_delay_us_, this->tx_delay_us_, this->precision_);
#ifdef USE_ETHERNET
  if (!this->install_rx_hook_requested_) {
    ESP_LOGCONFIG(TAG, "  Receive timestamp: network stack (driver hook disabled)");
  } else {
    ESP_LOGCONFIG(TAG, "  Receive timestamp: %s",
                  this->rx_hook_active_ ? "Ethernet driver" : "network stack (driver hook not installed yet)");
  }
#else
  ESP_LOGCONFIG(TAG, "  Receive timestamp: network stack");
#endif
#ifdef USE_PPS_NTP_MCPWM
  if (this->rx_reference_pin_ >= 0) {
    ESP_LOGCONFIG(TAG, "  Rx reference pin: GPIO%d (%s)", this->rx_reference_pin_,
                  this->cap_int_ != nullptr ? "MCPWM capture" : "unavailable");
  }
#endif
#ifdef USE_PPS_NTP_RAW_UDP
  ESP_LOGCONFIG(TAG, "  Transport: raw lwIP (experimental)");
#else
  ESP_LOGCONFIG(TAG, "  Transport: socket task");
#endif
#ifdef USE_PPS_NTP_TASK_CORE
  ESP_LOGCONFIG(TAG, "  Task core: %d (pinned)", USE_PPS_NTP_TASK_CORE);
#endif
  if (this->gnss_baud_rate_ != 0) {
    ESP_LOGCONFIG(TAG, "  GNSS baud rate: %u (from %u)", static_cast<unsigned>(this->gnss_baud_rate_),
                  static_cast<unsigned>(this->original_baud_));
  }
  if (this->constellations_[0] != '\0') {
    ESP_LOGCONFIG(TAG, "  Receiver constellations: %s\n  Hardware tracking channels: %u", this->constellations_,
                  this->tracking_channels_);
  }
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Satellites", this->satellites_sensor_);
  LOG_SENSOR("  ", "Signal Strength", this->signal_strength_sensor_);
  LOG_SENSOR("  ", "Strong Satellites", this->strong_satellites_sensor_);
  LOG_SENSOR("  ", "HDOP", this->hdop_sensor_);
  LOG_SENSOR("  ", "Rejected Pulses", this->rejected_pulses_sensor_);
  LOG_SENSOR("  ", "NMEA Errors", this->nmea_errors_sensor_);
  LOG_SENSOR("  ", "Pulse Age", this->pulse_age_sensor_);
  LOG_SENSOR("  ", "Frequency Offset", this->frequency_offset_sensor_);
  LOG_SENSOR("  ", "PPS Jitter", this->pps_jitter_sensor_);
  LOG_SENSOR("  ", "Requests", this->requests_sensor_);
  LOG_SENSOR("  ", "Rx Timestamp Gain", this->rx_timestamp_gain_sensor_);
  LOG_SENSOR("  ", "Rx Interrupt Lead", this->rx_interrupt_lead_sensor_);
  LOG_SENSOR("  ", "ARP Clients", this->arp_clients_sensor_);
  LOG_BINARY_SENSOR("  ", "Synced", this->synced_binary_sensor_);
}

// ---------------------------------------------------------------------------
// Clock discipline

void PPSNTPServer::handle_pulse_(double local_us) {
  this->edges_seen_++;

  // Without a fix the receiver's pulse is free-running; hold over on the local crystal instead
  bool fix_recent = this->last_rmc_valid_us_ != 0 && local_us - this->last_rmc_valid_us_ < FIX_STALE_US;
  if (this->hist_count_ > 0 && fix_recent) {
    // Label by counting whole seconds since the last accepted pulse
    double rate = this->model_.valid ? this->model_.local_us_per_s : 1e6;
    double elapsed = local_us - this->last_accepted_local_us_;
    int64_t seconds = llround(elapsed / rate);
    if (seconds < 1)
      return;  // glitch just after a pulse: leave the recorded pulse alone so its RMC still finds it
    if (seconds > MAX_PULSE_GAP_S) {
      this->reset_discipline_("PPS gap too long", true);  // the model is only as stale as its holdover
    } else {
      double error = elapsed - seconds * rate;
      double tolerance = 300.0 + 100e-6 * elapsed;
      if (std::fabs(error) <= tolerance) {
        this->off_second_edges_ = 0;
        this->last_pulse_us_ = llround(local_us);
        this->last_pulse_utc_s_ = this->last_accepted_utc_s_ + seconds;
        this->last_pulse_labelled_ = true;
        this->accept_pulse_(local_us, this->last_pulse_utc_s_);
        return;
      }
      // One stray edge is noise; a run of them means the pulse train itself has moved
      if (++this->off_second_edges_ < MAX_OFF_SECOND_EDGES) {
        ESP_LOGD(TAG, "Ignoring PPS edge %.0f us off the second", error);
        return;
      }
      this->reset_discipline_("PPS no longer on the second", false);
    }
  }

  // Candidate: the next valid RMC labels it
  this->last_pulse_us_ = llround(local_us);
  this->last_pulse_labelled_ = false;
}

void PPSNTPServer::accept_pulse_(double local_us, int64_t utc_s) {
  if (this->hist_count_ > 0 && utc_s <= this->last_accepted_utc_s_)
    this->reset_discipline_("GNSS time went backwards", false);

  // While re-locking on a held-over model the new pulses are the truth, not the model
  if (this->model_.valid && !this->relocking_) {
    double predicted = this->model_.anchor_local_us + (utc_s - this->model_.anchor_utc_s) * this->model_.local_us_per_s;
    double residual = local_us - predicted;
    this->last_residual_us_ = residual;
    if (std::fabs(residual) > this->max_residual_us_) {
      if (++this->outliers_ < 3) {
        ESP_LOGW(TAG, "PPS pulse %.0f µs from prediction; ignoring", residual);
        return;
      }
      this->reset_discipline_("PPS phase stepped", false);
    } else {
      this->outliers_ = 0;
      this->jitter_sq_us_ = this->hist_count_ < 8 ? residual * residual
                                                  : 0.95 * this->jitter_sq_us_ + 0.05 * residual * residual;
    }
  }

  this->hist_local_[this->hist_head_] = local_us;
  this->hist_utc_[this->hist_head_] = utc_s;
  this->hist_head_ = (this->hist_head_ + 1) % this->fit_window_;
  if (this->hist_count_ < this->fit_window_)
    this->hist_count_++;
  this->last_accepted_local_us_ = local_us;
  this->last_accepted_utc_s_ = utc_s;
  // A reset re-labels the pulse it happened on, which can then be accepted a second time: count edges, not calls
  if (local_us != this->last_counted_pulse_us_) {
    this->last_counted_pulse_us_ = local_us;
    this->pulses_accepted_++;
  }

  // Least-squares fit of local time against UTC seconds, relative to the newest pulse
  double rate = 1e6;
  double fitted_offset = 0;
  if (this->hist_count_ >= 2) {
    double sum_x = 0, sum_y = 0;
    for (int i = 0; i < this->hist_count_; i++) {
      sum_x += static_cast<double>(this->hist_utc_[i] - utc_s);
      sum_y += this->hist_local_[i] - local_us;
    }
    double mean_x = sum_x / this->hist_count_;
    double mean_y = sum_y / this->hist_count_;
    double sxx = 0, sxy = 0;
    for (int i = 0; i < this->hist_count_; i++) {
      double dx = static_cast<double>(this->hist_utc_[i] - utc_s) - mean_x;
      double dy = (this->hist_local_[i] - local_us) - mean_y;
      sxx += dx * dx;
      sxy += dx * dy;
    }
    rate = sxy / sxx;
    fitted_offset = mean_y - rate * mean_x;
    if (std::fabs(rate - 1e6) > MAX_RATE_ERROR_PPM) {
      this->reset_discipline_("implausible crystal rate", false);
      return;
    }
  }

  ClockModel next;
  // The first label comes from a single RMC, which a stalled loop can pair with the wrong pulse (off by a
  // whole second). Don't serve until later RMCs have independently agreed with the pulse count.
  next.valid = this->hist_count_ >= MIN_PULSES_FOR_SYNC && this->label_confirmations_ >= MIN_LABEL_CONFIRMATIONS;
  // Not locked yet, but a held-over model is still serving: leave it until the new fit is ready
  if (!next.valid && this->relocking_)
    return;
  this->relocking_ = false;
  next.anchor_local_us = local_us + fitted_offset;
  next.anchor_utc_s = utc_s;
  next.local_us_per_s = rate;
  next.last_pulse_local_us = llround(local_us);
  next.utc_trusted = this->utc_trusted_;
  this->fill_leap_(next);
  if (next.valid && next.utc_trusted)
    this->ever_synced_ = true;
  portENTER_CRITICAL(&this->lock_);
  this->model_ = next;
  portEXIT_CRITICAL(&this->lock_);
}

// keep_time: the old model's time is still trustworthy (it just has no fresh pulses), so it keeps
// serving in holdover while the fit rebuilds. Otherwise the model is dropped and, if the server had
// ever been synchronised, clients get stratum 16 rather than silence until the new fit is ready.
void PPSNTPServer::reset_discipline_(const char *reason, bool keep_time) {
  ESP_LOGW(TAG, "Resetting clock discipline: %s%s", reason, keep_time ? " (serving on holdover meanwhile)" : "");
  this->hist_count_ = 0;
  this->hist_head_ = 0;
  this->outliers_ = 0;
  this->label_mismatches_ = 0;
  this->label_confirmations_ = 0;
  this->off_second_edges_ = 0;
  this->jitter_sq_us_ = 0;
  if (this->leap_change_ == 0)
    this->leap_adj_s_ = 0;  // no history left that was labelled with it
  this->relocking_ = keep_time && this->model_.valid;
  if (!this->relocking_) {
    portENTER_CRITICAL(&this->lock_);
    this->model_.valid = false;
    portEXIT_CRITICAL(&this->lock_);
  }
}

void PPSNTPServer::fill_leap_(ClockModel &model) const {
  model.leap_adj_s = this->leap_adj_s_;
  model.leap_change = this->leap_change_;
  model.leap_at_s = this->leap_change_ != 0 ? this->leap_at_internal_() : 0;
  int64_t now_unix = this->last_accepted_utc_s_ - this->leap_adj_s_;
  model.leap_announce = this->leap_change_ != 0 && this->leap_from_ubx_ &&
                        this->leap_midnight_unix_ - now_unix <= LEAP_ANNOUNCE_S;
}

void PPSNTPServer::publish_model_() {
  ClockModel next = this->model_;
  next.utc_trusted = this->utc_trusted_;
  this->fill_leap_(next);
  portENTER_CRITICAL(&this->lock_);
  this->model_ = next;
  portEXIT_CRITICAL(&this->lock_);
}

ClockModel PPSNTPServer::get_model_() {
  portENTER_CRITICAL(&this->lock_);
  ClockModel copy = this->model_;
  portEXIT_CRITICAL(&this->lock_);
  return copy;
}

bool PPSNTPServer::is_synced_(const ClockModel &model, int64_t now_local_us) const {
  return model.valid && model.utc_trusted && now_local_us - model.last_pulse_local_us < this->holdover_us_;
}

// ---------------------------------------------------------------------------
// Receiver input: interleaved NMEA and UBX

void PPSNTPServer::feed_byte_(uint8_t byte) {
  switch (this->rx_state_) {
    case RxState::IDLE:
      if (byte == '$') {
        this->nmea_len_ = 0;
        this->nmea_[this->nmea_len_++] = '$';
        this->rx_state_ = RxState::NMEA;
      } else if (byte == 0xB5) {
        this->rx_state_ = RxState::UBX_SYNC2;
      }
      break;

    case RxState::NMEA:
      if (byte == '\r' || byte == '\n') {
        this->nmea_[this->nmea_len_] = '\0';
        if (this->nmea_len_ > 6)
          this->handle_nmea_(this->nmea_);
        this->rx_state_ = RxState::IDLE;
      } else if (byte < 0x20 || byte > 0x7E || this->nmea_len_ >= sizeof(this->nmea_) - 1) {
        this->rx_state_ = byte == 0xB5 ? RxState::UBX_SYNC2 : RxState::IDLE;
      } else {
        this->nmea_[this->nmea_len_++] = byte;
      }
      break;

    case RxState::UBX_SYNC2:
      if (byte == 0x62) {
        this->ubx_len_ = 0;
        this->rx_state_ = RxState::UBX_HEADER;
      } else {
        this->rx_state_ = RxState::IDLE;
        this->feed_byte_(byte);
      }
      break;

    case RxState::UBX_HEADER:
      this->ubx_[this->ubx_len_++] = byte;
      if (this->ubx_len_ == 4) {
        this->ubx_expected_ = 4 + (this->ubx_[2] | (this->ubx_[3] << 8)) + 2;
        this->rx_state_ = this->ubx_expected_ > sizeof(this->ubx_) ? RxState::IDLE : RxState::UBX_PAYLOAD;
      }
      break;

    case RxState::UBX_PAYLOAD:
      this->ubx_[this->ubx_len_++] = byte;
      if (this->ubx_len_ == this->ubx_expected_) {
        uint8_t ck_a = 0, ck_b = 0;
        for (uint32_t i = 0; i < this->ubx_len_ - 2; i++) {
          ck_a += this->ubx_[i];
          ck_b += ck_a;
        }
        if (ck_a == this->ubx_[this->ubx_len_ - 2] && ck_b == this->ubx_[this->ubx_len_ - 1])
          this->handle_ubx_(this->ubx_[0], this->ubx_[1], this->ubx_ + 4, this->ubx_len_ - 6);
        this->rx_state_ = RxState::IDLE;
      }
      break;
  }
}

void PPSNTPServer::handle_nmea_(char *line) {
  char *star = strchr(line, '*');
  uint8_t checksum = 0;
  for (char *p = line + 1; star != nullptr && p < star; p++)
    checksum ^= static_cast<uint8_t>(*p);
  if (star == nullptr || strlen(star) < 3 || checksum != static_cast<uint8_t>(strtoul(star + 1, nullptr, 16))) {
    this->nmea_bad_++;
    return;
  }
  *star = '\0';
  this->nmea_ok_++;
  this->nmea_ok_since_switch_ = true;

  char *fields[24];
  int count = 0;
  char *p = line;
  fields[count++] = p;
  while ((p = strchr(p, ',')) != nullptr && count < 24) {
    *p++ = '\0';
    fields[count++] = p;
  }

  // Address is $ttSSS; the talker (GP, GN, GL...) is ignored
  if (strlen(fields[0]) != 6)
    return;
  const char *type = fields[0] + 3;
  if (strcmp(type, "RMC") == 0) {
    // RMC opens each epoch's burst, so the GSV sentences accumulated since the last one are complete
    this->finalize_cno_();
    this->handle_rmc_(fields, count);
  } else if (strcmp(type, "GGA") == 0) {
    if (count > 7 && fields[7][0] != '\0')
      this->satellites_ = atoi(fields[7]);
    if (count > 8 && fields[8][0] != '\0') {
      this->hdop_ = atof(fields[8]);
      this->hdop_valid_ = true;
    }
  } else if (strcmp(type, "GSV") == 0) {
    this->handle_gsv_(fields, count);
  }
}

void PPSNTPServer::handle_rmc_(char **fields, int count) {
  int64_t now_us = esp_timer_get_time();
  if (count < 10 || fields[2][0] != 'A')
    return;

  const char *time = fields[1];
  const char *date = fields[9];
  if (strlen(time) < 6 || !all_digits(time, 6) || strlen(date) != 6 || !all_digits(date, 6))
    return;
  // Only whole-second epochs line up with a pulse
  if (time[6] == '.') {
    for (const char *p = time + 7; *p != '\0'; p++) {
      if (*p != '0')
        return;
    }
  }

  int day = two_digits(date), month = two_digits(date + 2), year = 2000 + two_digits(date + 4);
  int hour = two_digits(time), minute = two_digits(time + 2), second = two_digits(time + 4);
  if (day < 1 || day > 31 || month < 1 || month > 12 || hour > 23 || minute > 59 || second > 60)
    return;
  bool leap_second_now = second == 60;  // 23:59:60, an inserted leap second in progress
  if (leap_second_now && (hour != 23 || minute != 59))
    return;
  int64_t utc_s = days_from_civil(year, month, day) * 86400 + hour * 3600 + minute * 60 + second;

  // Old and clone receivers that mishandle the 10-bit GPS week number report a date exactly a multiple of
  // 1024 weeks in the past. Anything before this firmware existed can't be a live fix: move it forward by
  // whole rollovers, and drop it if that still isn't plausible.
  if (utc_s < MIN_VALID_UTC_S) {
    int64_t reported = utc_s;
    for (int i = 0; i < 3 && utc_s < MIN_VALID_UTC_S; i++)
      utc_s += GPS_WEEK_ROLLOVER_S;
    if (utc_s < MIN_VALID_UTC_S)
      return;
    if (!this->rollover_warned_) {
      this->rollover_warned_ = true;
      ESP_LOGW(TAG, "Receiver date is in the past (unix %lld); assuming GPS week rollover and using %lld",
               static_cast<long long>(reported), static_cast<long long>(utc_s));
    }
  }
  this->last_rmc_valid_us_ = now_us;

  // From here on utc_s is "internal" time: UTC plus the leap seconds that have gone by while we've been
  // running, so that it keeps counting one per pulse across a leap.
  if (leap_second_now && this->leap_change_ == 0) {
    // No advance notice (u-blox 6/7 have no NAV-TIMELS): take it from the receiver's own 23:59:60
    this->leap_change_ = 1;
    this->leap_midnight_unix_ = utc_s;  // 23:59:60 converts to the following midnight
    this->leap_from_ubx_ = false;
    this->publish_model_();
    ESP_LOGW(TAG, "Receiver reports 23:59:60: inserting a leap second now (no advance notice was available)");
  }
  if (this->leap_change_ != 0 && !leap_second_now && utc_s >= this->leap_midnight_unix_) {
    utc_s += this->leap_adj_s_ + this->leap_change_;
  } else {
    utc_s += this->leap_adj_s_;
  }

  // A u-blox receiver sends the RMC for an epoch shortly after that epoch's pulse
  int64_t age = now_us - this->last_pulse_us_;
  if (this->last_pulse_us_ == 0 || age <= 0 || age > RMC_MAX_DELAY_US)
    return;

  if (this->last_pulse_labelled_) {
    // Compare against the label the pulse was given, not the last accepted one: an outlier pulse is
    // labelled correctly but never accepted
    if (this->last_pulse_utc_s_ == utc_s) {
      this->label_mismatches_ = 0;
      if (this->label_confirmations_ < UINT8_MAX)
        this->label_confirmations_++;
      return;
    }
    if (++this->label_mismatches_ < 3)
      return;
    this->reset_discipline_("NMEA time disagrees with PPS count", false);
  }

  ESP_LOGD(TAG, "Labelling the pulse from %lld ms ago as unix %lld (from RMC)", static_cast<long long>(age / 1000),
           static_cast<long long>(utc_s));
  this->last_pulse_labelled_ = true;
  this->last_pulse_utc_s_ = utc_s;
  this->accept_pulse_(this->last_pulse_us_, utc_s);
}

// $ttGSV,totalMsgs,msgNum,satsInView,{prn,elevation,azimuth,cno} x up to 4[,signalId]
void PPSNTPServer::handle_gsv_(char **fields, int count) {
  this->cno_saw_gsv_ = true;
  for (int i = 7; i < count; i += 4) {  // the first C/N0 is field 7, then every fourth
    if (fields[i][0] == '\0')
      continue;  // in view but not tracked
    int cno = atoi(fields[i]);
    if (cno <= 0 || cno > 99)
      continue;
    this->cno_sum_ += cno;
    this->cno_count_++;
    if (cno >= this->strong_threshold_)
      this->strong_accum_++;
  }
}

void PPSNTPServer::finalize_cno_() {
  // Zero tracked satellites is a real reading (a disconnected antenna), but only once GSV has been seen
  if (this->cno_saw_gsv_) {
    this->cno_mean_ = this->cno_count_ > 0 ? static_cast<float>(this->cno_sum_) / this->cno_count_ : 0.0f;
    this->cno_valid_ = true;
    this->strong_satellites_ = this->strong_accum_;
  }
  this->cno_sum_ = 0;
  this->cno_count_ = 0;
  this->strong_accum_ = 0;
  this->cno_saw_gsv_ = false;
}

// UBX-CFG-GNSS: which constellations the receiver has and which are switched on. Reported once so the
// installer can see whether, for example, GLONASS is available to add alongside GPS.
void PPSNTPServer::handle_cfg_gnss_(const uint8_t *payload, uint16_t len) {
  if (len < 4)
    return;
  uint8_t blocks = payload[3];
  if (len < 4u + blocks * 8u)
    return;
  static const char *const NAMES[] = {"GPS", "SBAS", "Galileo", "BeiDou", "IMES", "QZSS", "GLONASS", "NavIC"};
  char *list = this->constellations_;
  const size_t capacity = sizeof(this->constellations_);
  size_t pos = 0;
  for (uint8_t i = 0; i < blocks; i++) {
    const uint8_t *block = payload + 4 + i * 8;
    uint8_t id = block[0];
    const char *name = id < sizeof(NAMES) / sizeof(NAMES[0]) ? NAMES[id] : "?";
    int written = snprintf(list + pos, capacity - pos, "%s%s(%s, max %u ch)", pos > 0 ? ", " : "", name,
                           (block[4] & 0x01) ? "on" : "off", block[2]);
    if (written < 0 || pos + static_cast<size_t>(written) >= capacity)
      break;
    pos += written;
  }
  this->tracking_channels_ = payload[1];
  ESP_LOGI(TAG, "Receiver constellations: %s; %u hardware tracking channels", list, payload[1]);
}

void PPSNTPServer::handle_ubx_(uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t len) {
  this->ubx_seen_ = true;
  this->ubx_frames_++;
  if (msg_class == 0x05 && msg_id == 0x00 && len >= 2) {  // UBX-ACK-NAK
    ESP_LOGW(TAG, "Receiver rejected UBX command 0x%02X 0x%02X", payload[0], payload[1]);
    return;
  }
  if (msg_class == 0x06 && msg_id == 0x3E) {
    this->handle_cfg_gnss_(payload, len);
    return;
  }
  if (msg_class == 0x01 && msg_id == 0x26 && len >= 24) {  // NAV-TIMELS
    int8_t change = static_cast<int8_t>(payload[11]);
    int32_t to_event = static_cast<int32_t>(payload[12] | (payload[13] << 8) | (payload[14] << 16) |
                                            (static_cast<uint32_t>(payload[15]) << 24));
    bool valid = (payload[23] & 0x02) != 0;  // validTimeToLsEvent
    if (valid && (change == 1 || change == -1) && to_event > 0 && this->hist_count_ > 0 && this->leap_change_ == 0) {
      // The event is a UTC midnight; rounding removes any doubt about which edge of the leap it counts to
      int64_t now_unix = this->last_accepted_utc_s_ - this->leap_adj_s_;
      this->leap_midnight_unix_ = ((now_unix + to_event + 43200) / 86400) * 86400;
      this->leap_change_ = change;
      this->leap_from_ubx_ = true;
      this->publish_model_();
      ESP_LOGW(TAG, "Leap second scheduled: %+d at unix %lld (in %ld s)", change,
               static_cast<long long>(this->leap_midnight_unix_), static_cast<long>(to_event));
    }
    return;
  }
  if (msg_class != 0x01 || msg_id != 0x21 || len < 20)
    return;
  this->timeutc_seen_ = true;
  bool valid_utc = (payload[19] & 0x04) != 0;  // NAV-TIMEUTC valid.validUTC: leap seconds are known
  if (valid_utc != this->utc_trusted_) {
    ESP_LOGI(TAG, "Receiver UTC %s", valid_utc ? "confirmed (leap seconds known)" : "not yet valid");
    this->utc_trusted_ = valid_utc;
    this->publish_model_();
  }
}

void PPSNTPServer::send_ubx_(uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t len) {
  uint8_t header[6] = {0xB5, 0x62, msg_class, msg_id, static_cast<uint8_t>(len & 0xFF), static_cast<uint8_t>(len >> 8)};
  uint8_t ck_a = 0, ck_b = 0;
  for (int i = 2; i < 6; i++) {
    ck_a += header[i];
    ck_b += ck_a;
  }
  for (uint16_t i = 0; i < len; i++) {
    ck_a += payload[i];
    ck_b += ck_a;
  }
  uint8_t checksum[2] = {ck_a, ck_b};
  this->write_array(header, sizeof(header));
  if (len > 0)
    this->write_array(payload, len);
  this->write_array(checksum, sizeof(checksum));
  this->flush();
}

// Applied once per boot, after the baud rate has settled. These are RAM-only settings: they are not
// saved to the receiver, so a receiver power cycle reverts them and our next boot re-applies them.
void PPSNTPServer::configure_receiver_() {
  this->receiver_configured_ = true;

  if (this->stationary_) {
    // UBX-CFG-NAV5 with only the dynamic-model bit set, model 2 = stationary. A receiver that knows it
    // cannot be moving constrains its solution, which steadies the time when signals are marginal.
    uint8_t cfg_nav5[36] = {0};
    cfg_nav5[0] = 0x01;  // mask: apply dynModel
    cfg_nav5[2] = 0x02;  // dynModel: stationary
    this->send_ubx_(0x06, 0x24, cfg_nav5, sizeof(cfg_nav5));
    ESP_LOGI(TAG, "Set the receiver to the stationary dynamic model");
  }

  if (this->trim_nmea_) {
    // Legacy UBX-CFG-MSG, 3-byte form: set one NMEA message's rate on the port it arrives on
    static constexpr uint8_t NMEA_CLASS = 0xF0;
    static constexpr uint8_t KEEP[] = {0x04, 0x00, 0x03};  // RMC (time), GGA (satellites, HDOP), GSV (C/N0)
    static constexpr uint8_t DROP[] = {0x01, 0x02, 0x05, 0x06,
                                       0x07, 0x08, 0x09, 0x0A};  // GLL GSA VTG GRS GST ZDA GBS DTM
    for (uint8_t id : KEEP) {
      uint8_t payload[3] = {NMEA_CLASS, id, 1};
      this->send_ubx_(0x06, 0x01, payload, sizeof(payload));
    }
    for (uint8_t id : DROP) {
      uint8_t payload[3] = {NMEA_CLASS, id, 0};
      this->send_ubx_(0x06, 0x01, payload, sizeof(payload));
    }
    ESP_LOGI(TAG, "Trimmed the receiver's NMEA output to RMC, GGA and GSV");
  }

  this->send_ubx_(0x06, 0x3E, nullptr, 0);  // poll CFG-GNSS, to report which constellations exist
}

// Older u-blox receivers only accept the legacy UBX-CFG-PRT message for port settings
void PPSNTPServer::start_baud_switch_() {
  this->parent_->set_baud_rate(this->original_baud_);
  this->parent_->load_settings(false);

  uint8_t cfg_prt[20] = {0};
  cfg_prt[0] = 1;     // portID = UART1
  cfg_prt[4] = 0xD0;  // mode = 8N1
  cfg_prt[5] = 0x08;
  cfg_prt[8] = this->gnss_baud_rate_ & 0xFF;
  cfg_prt[9] = (this->gnss_baud_rate_ >> 8) & 0xFF;
  cfg_prt[10] = (this->gnss_baud_rate_ >> 16) & 0xFF;
  cfg_prt[11] = (this->gnss_baud_rate_ >> 24) & 0xFF;
  cfg_prt[12] = 0x07;  // inProtoMask = UBX + NMEA + RTCM
  cfg_prt[14] = 0x03;  // outProtoMask = UBX + NMEA
  this->send_ubx_(0x06, 0x00, cfg_prt, sizeof(cfg_prt));

  this->parent_->set_baud_rate(this->gnss_baud_rate_);
  this->parent_->load_settings(false);
  this->rx_state_ = RxState::IDLE;
  this->nmea_ok_since_switch_ = false;
  this->baud_attempts_++;
  this->baud_deadline_ms_ = millis() + BAUD_VERIFY_MS;
  this->baud_state_ = BaudState::VERIFY;
}

void PPSNTPServer::service_baud_switch_() {
  uint32_t now_ms = millis();
  switch (this->baud_state_) {
    case BaudState::PROBE:
      if (this->nmea_ok_since_switch_) {
        ESP_LOGI(TAG, "GNSS receiver already at %u baud", static_cast<unsigned>(this->gnss_baud_rate_));
        this->baud_state_ = BaudState::DONE;  // nothing changed, so nothing to save
      } else if (static_cast<int32_t>(now_ms - this->baud_deadline_ms_) >= 0) {
        this->start_baud_switch_();
      }
      break;
    case BaudState::VERIFY:
      if (this->nmea_ok_since_switch_) {
        // Persist to BBR/flash/EEPROM where the module has them, so a power cycle keeps the new baud
        uint8_t cfg_cfg[13] = {0};
        cfg_cfg[4] = 0x1F;
        cfg_cfg[5] = 0x1F;
        cfg_cfg[12] = 0x17;
        this->send_ubx_(0x06, 0x09, cfg_cfg, sizeof(cfg_cfg));
        ESP_LOGI(TAG, "GNSS receiver now at %u baud", static_cast<unsigned>(this->gnss_baud_rate_));
        this->baud_state_ = BaudState::DONE;
      } else if (static_cast<int32_t>(now_ms - this->baud_deadline_ms_) >= 0) {
        this->parent_->set_baud_rate(this->original_baud_);
        this->parent_->load_settings(false);
        this->rx_state_ = RxState::IDLE;
        if (this->baud_attempts_ < BAUD_MAX_ATTEMPTS) {
          ESP_LOGW(TAG, "No NMEA at %u baud; retrying", static_cast<unsigned>(this->gnss_baud_rate_));
          this->baud_deadline_ms_ = now_ms + BAUD_RETRY_MS;
          this->baud_state_ = BaudState::RETRY_WAIT;
        } else {
          ESP_LOGW(TAG, "Receiver did not switch baud; staying at %u", static_cast<unsigned>(this->original_baud_));
          this->baud_state_ = BaudState::DONE;
        }
      }
      break;
    case BaudState::RETRY_WAIT:
      if (static_cast<int32_t>(now_ms - this->baud_deadline_ms_) >= 0)
        this->start_baud_switch_();
      break;
    default:
      break;
  }
}

// ---------------------------------------------------------------------------
// NTP server

bool PPSNTPServer::build_reply_(const uint8_t *request, int64_t rx_local_us, uint16_t src_port, uint8_t *reply) {
  uint8_t version = (request[0] >> 3) & 0x07;
  uint8_t mode = request[0] & 0x07;
  if (mode != 3 || version < 1 || version > 4)
    return false;
  rx_local_us = this->choose_rx_stamp_(request, src_port, rx_local_us);

  ClockModel model = this->get_model_();
  if (!model.valid && !this->ever_synced_)
    return false;  // never synchronised: stay silent rather than hand out a bogus time
  // Lost lock after having been synchronised: answer with stratum 16 (clients discard the time but
  // don't time out) using the last model, until the new fit is ready
  bool synced = model.valid && this->is_synced_(model, rx_local_us);
  rx_local_us -= llround(this->rx_delay_us_);  // the stamp is taken this long after the frame arrived
  double age_s = static_cast<double>(rx_local_us - model.last_pulse_local_us) / 1e6;
  // The base covers what no local measurement can see: network asymmetry and the receive/transmit paths
  double dispersion_us = this->root_dispersion_us_ + HOLDOVER_DRIFT_PPM * age_s;

  memset(reply, 0, 48);
  // LI: 3 = unsynchronised; 1/2 = the last minute of today has 61/59 seconds
  uint8_t li = 0;
  if (!synced) {
    li = 3;
  } else if (model.leap_announce && internal_us_at(model, rx_local_us) < model.leap_at_s * 1000000LL) {
    li = model.leap_change > 0 ? 1 : 2;
  }
  reply[0] = (li << 6) | (version << 3) | 4;  // LI, VN, mode = server
  reply[1] = synced ? 1 : 16;
  reply[2] = request[2];  // poll
  reply[3] = static_cast<uint8_t>(this->precision_);
  put_be32(reply + 8, static_cast<uint32_t>(std::min(dispersion_us * 65536.0 / 1e6, 4294967295.0)));
  memcpy(reply + 12, this->refid_, 4);
  put_ntp_timestamp(reply + 16, internal_to_utc_us(model, model.anchor_utc_s * 1000000LL));
  memcpy(reply + 24, request + 40, 8);  // originate = client's transmit
  put_ntp_timestamp(reply + 32, utc_us_at(model, rx_local_us));
  // Last, as close to the send as we get; tx_delay is the measured path from here to the wire
  put_ntp_timestamp(reply + 40, utc_us_at(model, esp_timer_get_time() + llround(this->tx_delay_us_)));
  this->requests_++;
  return true;
}

// Single writer (the Ethernet driver task); readers retry nothing, they just miss on a torn read
void PPSNTPServer::rx_record_(const uint8_t *key, int64_t t) {
  RxStamp &slot = this->rx_ring_[this->rx_ring_next_];
  this->rx_ring_next_ = (this->rx_ring_next_ + 1) % RX_RING;
  uint32_t seq = slot.seq.load(std::memory_order_relaxed);
  slot.seq.store(seq + 1, std::memory_order_relaxed);
  std::atomic_thread_fence(std::memory_order_release);
  memcpy(slot.key, key, RX_KEY_LEN);
  slot.t = t;
  slot.seq.store(seq + 2, std::memory_order_release);
}

bool PPSNTPServer::rx_lookup_(const uint8_t *key, int64_t now_us, int64_t *t_out) {
  for (auto &slot : this->rx_ring_) {
    uint32_t before = slot.seq.load(std::memory_order_acquire);
    if (before == 0 || (before & 1) != 0)
      continue;
    uint8_t slot_key[RX_KEY_LEN];
    memcpy(slot_key, slot.key, RX_KEY_LEN);
    int64_t t = slot.t;
    std::atomic_thread_fence(std::memory_order_acquire);
    if (slot.seq.load(std::memory_order_relaxed) != before)
      continue;
    if (memcmp(slot_key, key, RX_KEY_LEN) == 0 && t <= now_us && now_us - t < RX_STAMP_MAX_AGE_US) {
      *t_out = t;
      return true;
    }
  }
  return false;
}

// Prefers the driver-level stamp for this request when there is one. The stack stamp is always the later
// of the two; the difference is what the driver hook buys, reported as rx_timestamp_gain.
int64_t PPSNTPServer::choose_rx_stamp_(const uint8_t *request, uint16_t src_port, int64_t stack_us) {
  if (!this->rx_hook_active_.load(std::memory_order_relaxed))
    return stack_us;
  uint8_t key[RX_KEY_LEN];
  memcpy(key, request + 40, 8);
  key[8] = src_port >> 8;
  key[9] = src_port & 0xFF;
  int64_t hook_us;
  if (!this->rx_lookup_(key, stack_us, &hook_us)) {
    this->rx_hook_misses_++;
    return stack_us;
  }
  this->rx_hook_hits_++;
  this->rx_gain_sum_us_ += static_cast<int32_t>(stack_us - hook_us);
  this->rx_gain_count_++;
  return this->driver_rx_timestamp_.load(std::memory_order_relaxed) ? hook_us : stack_us;
}

#ifdef USE_ETHERNET
PPSNTPServer *PPSNTPServer::rx_hook_owner_ = nullptr;

// Ethernet driver task, for every received frame. Stamp first, then hand the frame on exactly as the stock
// glue would; the frame belongs to the network stack from esp_netif_receive() on, so it's parsed before.
esp_err_t PPSNTPServer::rx_hook(esp_eth_handle_t eth, uint8_t *buffer, uint32_t length, void *priv, void *info) {
  int64_t now = esp_timer_get_time();
  PPSNTPServer *self = rx_hook_owner_;
  uint8_t key[RX_KEY_LEN];
  if (self != nullptr && parse_ntp_request_frame(buffer, length, self->port_, key)) {
    self->rx_record_(key, now);
    self->note_interrupt_lead_(now);
  }
  return esp_netif_receive(static_cast<esp_netif_t *>(priv), buffer, length, nullptr);
}

bool PPSNTPServer::install_rx_hook_() {
#if CONFIG_ESP_NETIF_L2_TAP
  ESP_LOGW(TAG, "L2 TAP is enabled; not replacing the Ethernet input path, so receive stamps come from the stack");
  return false;
#else
  auto *eth = ethernet::global_eth_component;
  if (eth == nullptr || eth->get_eth_handle() == nullptr || eth->get_esp_netif() == nullptr) {
    ESP_LOGW(TAG, "No Ethernet driver handle; receive stamps come from the network stack");
    return false;
  }
  rx_hook_owner_ = this;
  // priv stays the esp_netif the stock glue registered, so a frame caught mid-swap is handled either way
  esp_err_t err = esp_eth_update_input_path_info(eth->get_eth_handle(), PPSNTPServer::rx_hook, eth->get_esp_netif());
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Could not hook the Ethernet input path (%s)", esp_err_to_name(err));
    return false;
  }
  ESP_LOGI(TAG, "Receive timestamps taken in the Ethernet driver");
  return true;
#endif
}
#endif  // USE_ETHERNET

// Called per request from the serving context. Lock-free: a lost race only costs one missed priming.
void PPSNTPServer::note_client_ipv4_(uint32_t addr_be) {
  if (addr_be == 0)
    return;
  uint32_t now_ms = millis();
  for (int i = 0; i < ARP_SLOTS; i++) {
    if (this->arp_clients_[i].load(std::memory_order_relaxed) == addr_be) {
      this->arp_seen_ms_[i].store(now_ms, std::memory_order_relaxed);
      return;
    }
  }
  // New client: take an empty slot, else the least recently heard one
  int pick = 0;
  uint32_t oldest_age = 0;
  for (int i = 0; i < ARP_SLOTS; i++) {
    if (this->arp_clients_[i].load(std::memory_order_relaxed) == 0) {
      pick = i;
      break;
    }
    uint32_t age = now_ms - this->arp_seen_ms_[i].load(std::memory_order_relaxed);
    if (age > oldest_age) {
      oldest_age = age;
      pick = i;
    }
  }
  this->arp_seen_ms_[pick].store(now_ms, std::memory_order_relaxed);
  this->arp_clients_[pick].store(addr_be, std::memory_order_relaxed);
}

// lwIP never learns a sender's MAC from its IP traffic and forgets entries after 5 minutes, so a client
// polling every 17 minutes would otherwise find the cache cold on every request: the reply, already
// stamped, then waits a full ARP round trip. Asking each recent client (or, off-subnet, the gateway)
// every two minutes keeps its entry fresh.
void PPSNTPServer::refresh_arp_() {
#ifdef USE_ESP32
  uint32_t now_ms = millis();
  LwIPLock lock;
  uint32_t gateways[ARP_SLOTS];
  int gateway_count = 0;
  for (int i = 0; i < ARP_SLOTS; i++) {
    uint32_t addr_be = this->arp_clients_[i].load(std::memory_order_relaxed);
    if (addr_be == 0)
      continue;
    if (now_ms - this->arp_seen_ms_[i].load(std::memory_order_relaxed) > ARP_CLIENT_IDLE_MS) {
      this->arp_clients_[i].store(0, std::memory_order_relaxed);
      continue;
    }
    ip4_addr_t target;
    ip4_addr_set_u32(&target, addr_be);
    struct netif *nif = ip4_route(&target);
    if (nif == nullptr || (nif->flags & NETIF_FLAG_ETHARP) == 0)
      continue;
    if (!ip4_addr_net_eq(&target, netif_ip4_addr(nif), netif_ip4_netmask(nif))) {
      target = *netif_ip4_gw(nif);
      if (ip4_addr_isany_val(target))
        continue;
      uint32_t gw = ip4_addr_get_u32(&target);
      bool done = false;
      for (int g = 0; g < gateway_count; g++)
        done |= gateways[g] == gw;
      if (done)
        continue;
      gateways[gateway_count++] = gw;
    }
    if (etharp_request(nif, &target) == ERR_OK)
      this->arp_requests_++;
  }
#endif
}

bool PPSNTPServer::start_server_() {
#ifdef USE_PPS_NTP_RAW_UDP
  return this->start_raw_udp_();
#else
  // Run on the core the ESPHome loop isn't on, so request timestamps don't wait out its critical sections
#if defined(USE_PPS_NTP_TASK_CORE)
  BaseType_t core = USE_PPS_NTP_TASK_CORE;
#elif CONFIG_FREERTOS_NUMBER_OF_CORES > 1
  BaseType_t core = xPortGetCoreID() == 0 ? 1 : 0;
#else
  BaseType_t core = tskNO_AFFINITY;
#endif
  if (xTaskCreatePinnedToCore(PPSNTPServer::ntp_task, "pps_ntp", 4096, this, 10, &this->task_, core) != pdPASS) {
    ESP_LOGE(TAG, "Could not start NTP server task");
    return false;
  }
  ESP_LOGI(TAG, "Network up; serving NTP on UDP port %u (core %d)", this->port_, static_cast<int>(core));
  return true;
#endif
}

#ifdef USE_PPS_NTP_RAW_UDP
// EXPERIMENTAL, not yet run on hardware. Serves straight from lwIP's tcpip thread: the request is
// timestamped in the UDP input callback and answered there, with no socket mailbox, no task wake-up
// and no second copy. Select with `transport: raw_lwip`.
bool PPSNTPServer::start_raw_udp_() {
  LwIPLock lock;
  this->pcb_ = udp_new_ip_type(IPADDR_TYPE_ANY);
  if (this->pcb_ == nullptr) {
    ESP_LOGE(TAG, "Could not allocate a UDP PCB");
    return false;
  }
  err_t err = udp_bind(this->pcb_, IP_ANY_TYPE, this->port_);
  if (err != ERR_OK) {
    ESP_LOGE(TAG, "Could not bind UDP port %u (lwIP error %d)", this->port_, static_cast<int>(err));
    udp_remove(this->pcb_);
    this->pcb_ = nullptr;
    return false;
  }
  udp_recv(this->pcb_, PPSNTPServer::raw_recv, this);
  ESP_LOGI(TAG, "Network up; serving NTP on UDP port %u (raw lwIP)", this->port_);
  return true;
}

// Runs in the tcpip thread: keep it short, never block, never log
void PPSNTPServer::raw_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, uint16_t port) {
  int64_t rx_local = esp_timer_get_time();
  auto *self = static_cast<PPSNTPServer *>(arg);
  uint8_t request[48];
  if (p != nullptr && p->tot_len >= sizeof(request) &&
      pbuf_copy_partial(p, request, sizeof(request), 0) == sizeof(request)) {
    struct pbuf *out = pbuf_alloc(PBUF_TRANSPORT, 48, PBUF_RAM);
    if (out != nullptr) {
      if (self->build_reply_(request, rx_local, port, static_cast<uint8_t *>(out->payload))) {
        udp_sendto(pcb, out, addr, port);
#if LWIP_IPV4
        if (IP_IS_V4(addr))
          self->note_client_ipv4_(ip4_addr_get_u32(ip_2_ip4(addr)));
#endif
      }
      pbuf_free(out);
    }
  }
  if (p != nullptr)
    pbuf_free(p);
}
#endif  // USE_PPS_NTP_RAW_UDP

void PPSNTPServer::ntp_task(void *arg) { static_cast<PPSNTPServer *>(arg)->ntp_loop_(); }

void PPSNTPServer::ntp_loop_() {
  for (;;) {
    // One dual-stack socket when the build has IPv6 (`network: enable_ipv6: true`): IPv4 clients then
    // show up as v4-mapped addresses, and replying to the source address works for both.
#if LWIP_IPV6
    int sock = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
#else
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
#endif
    if (sock < 0) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
#if LWIP_IPV6
    int v6only = 0;
    setsockopt(sock, IPPROTO_IPV6, IPV6_V6ONLY, &v6only, sizeof(v6only));
    struct sockaddr_in6 addr {};
    addr.sin6_family = AF_INET6;
    addr.sin6_port = htons(this->port_);
    addr.sin6_addr = in6addr_any;
#else
    struct sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(this->port_);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
#endif
    if (bind(sock, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      close(sock);
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    for (;;) {
      uint8_t request[68];
      struct sockaddr_storage source {};  // large enough for either family
      socklen_t source_len = sizeof(source);
      int received = recvfrom(sock, request, sizeof(request), 0, reinterpret_cast<struct sockaddr *>(&source),
                              &source_len);
      int64_t rx_local = esp_timer_get_time();
      if (received < 0)
        break;
      if (received < 48)
        continue;
      uint8_t reply[48];
      uint16_t src_port;
      uint32_t src_v4 = 0;  // network byte order; 0 = not IPv4
#if LWIP_IPV6
      const auto *src6 = reinterpret_cast<const struct sockaddr_in6 *>(&source);
      src_port = ntohs(src6->sin6_port);
      const uint8_t *a = reinterpret_cast<const uint8_t *>(&src6->sin6_addr);
      static const uint8_t V4_MAPPED[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xFF, 0xFF};
      if (memcmp(a, V4_MAPPED, 12) == 0)
        memcpy(&src_v4, a + 12, 4);
#else
      const auto *src4 = reinterpret_cast<const struct sockaddr_in *>(&source);
      src_port = ntohs(src4->sin_port);
      src_v4 = src4->sin_addr.s_addr;
#endif
      if (this->build_reply_(request, rx_local, src_port, reply)) {
        sendto(sock, reply, sizeof(reply), 0, reinterpret_cast<struct sockaddr *>(&source), source_len);
        this->note_client_ipv4_(src_v4);
      }
    }
    close(sock);
  }
}

}  // namespace esphome::pps_ntp
