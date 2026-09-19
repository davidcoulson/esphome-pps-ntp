#include "gnss_sim.h"

#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <sys/time.h>

#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_rom_gpio.h>
#include <hal/uart_periph.h>

#include "esphome/core/log.h"

namespace esphome::gnss_sim {

static const char *const TAG = "gnss_sim";
static constexpr int64_t MIN_VALID_TIME_S = 1767225600;  // 2026-01-01: the system clock has been set
static constexpr int64_t GPS_WEEK_ROLLOVER_S = 619315200;

void GNSSSim::setup() {
  auto port = static_cast<uart_port_t>(this->uart_num_);
  uart_config_t config = {};
  config.baud_rate = this->baud_;
  config.data_bits = UART_DATA_8_BITS;
  config.parity = UART_PARITY_DISABLE;
  config.stop_bits = UART_STOP_BITS_1;
  config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  config.source_clk = UART_SCLK_DEFAULT;
  if (uart_driver_install(port, 512, 2048, 0, nullptr, 0) != ESP_OK || uart_param_config(port, &config) != ESP_OK) {
    ESP_LOGE(TAG, "Could not set up UART%d", this->uart_num_);
    this->mark_failed();
    return;
  }

  // Route the pads by hand rather than with uart_set_pin()/gpio_config(): each pad may already belong
  // to pps_ntp on this same chip. Outputs keep their input path enabled, and inputs are attached
  // without touching whatever is driving the pad, so both ends of each signal share one GPIO.
  auto tx = static_cast<gpio_num_t>(this->tx_pin_);
  auto rx = static_cast<gpio_num_t>(this->rx_pin_);
  auto pps = static_cast<gpio_num_t>(this->pps_pin_);

  gpio_set_direction(tx, GPIO_MODE_INPUT_OUTPUT);  // resets the pad's output to plain GPIO...
  esp_rom_gpio_connect_out_signal(tx, UART_PERIPH_SIGNAL(this->uart_num_, SOC_UART_PERIPH_SIGNAL_TX), false, false);

  gpio_input_enable(rx);
  esp_rom_gpio_connect_in_signal(rx, UART_PERIPH_SIGNAL(this->uart_num_, SOC_UART_PERIPH_SIGNAL_RX), false);

  gpio_set_level(pps, 0);
  gpio_set_direction(pps, GPIO_MODE_INPUT_OUTPUT);
}

void GNSSSim::dump_config() {
  ESP_LOGCONFIG(TAG,
                "GNSS simulator (TEST ONLY):\n"
                "  PPS out: GPIO%d\n"
                "  TX: GPIO%d  RX: GPIO%d  (UART%d, %" PRIu32 " baud)\n"
                "  Period offset: %d ppm",
                this->pps_pin_, this->tx_pin_, this->rx_pin_, this->uart_num_, this->baud_, this->ppm_);
}

void GNSSSim::start_() {
  // Label pulses from the system clock once, then free-run like a receiver's own timebase. Following
  // the system clock instead would turn every SNTP/HA time adjustment into a PPS phase step.
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  if (tv.tv_sec < MIN_VALID_TIME_S)
    return;
  this->first_epoch_s_ = tv.tv_sec + 2;

  esp_timer_create_args_t args = {};
  args.callback = &GNSSSim::pulse_cb;
  args.arg = this;
  args.dispatch_method = ESP_TIMER_TASK;
  args.name = "gnss_sim_pps";
  if (esp_timer_create(&args, &this->timer_) != ESP_OK) {
    this->mark_failed();
    return;
  }
  // First pulse on the next-but-one second boundary of the system clock. Every later deadline is the
  // previous one plus the period, so timer latency never accumulates.
  gettimeofday(&tv, nullptr);
  int64_t to_first = (this->first_epoch_s_ - tv.tv_sec) * 1000000LL - tv.tv_usec;
  if (to_first < 1000)
    to_first = 1000;
  this->deadline_us_ = esp_timer_get_time() + to_first;
  esp_timer_start_once(this->timer_, to_first);
  this->started_ = true;
  ESP_LOGI(TAG, "First pulse in %lld ms, labelled unix %lld; period %d us", static_cast<long long>(to_first / 1000),
           static_cast<long long>(this->first_epoch_s_), 1000000 + this->ppm_);
}

// esp_timer task: raise the pin as close to the deadline as a task-level callback allows
void GNSSSim::pulse_cb(void *arg) {
  auto *self = static_cast<GNSSSim *>(arg);
  if (self->pps_enabled_)
    gpio_set_level(static_cast<gpio_num_t>(self->pps_pin_), 1);
  self->pulse_us_ = esp_timer_get_time();
  self->pulse_count_++;
  self->deadline_us_ += 1000000 + self->ppm_ + self->phase_step_us_.exchange(0);
  int64_t wait = self->deadline_us_ - esp_timer_get_time();
  esp_timer_start_once(self->timer_, wait > 1000 ? wait : 1000);
}

void GNSSSim::loop() {
  if (this->is_failed())
    return;
  if (!this->started_) {
    this->start_();
    return;
  }
  int64_t now = esp_timer_get_time();
  auto pps = static_cast<gpio_num_t>(this->pps_pin_);

  uint32_t count = this->pulse_count_;
  if (count != this->handled_pulse_) {
    this->handled_pulse_ = count;
    int64_t pulse_us = this->pulse_us_;
    this->pin_high_ = true;
    this->epoch_pending_ = true;
    this->epoch_due_us_ = pulse_us + this->nmea_delay_us_;
    if (this->glitch_requested_) {
      this->glitch_requested_ = false;
      this->glitch_due_us_ = pulse_us + 50000;
    }
  }

  if (this->pin_high_ && now - this->pulse_us_ >= this->pulse_width_us_) {
    gpio_set_level(pps, 0);
    this->pin_high_ = false;
  }
  if (this->glitch_due_us_ != 0 && now >= this->glitch_due_us_) {
    gpio_set_level(pps, 1);
    this->glitch_due_us_ = 0;
    this->glitch_end_us_ = now + 1000;
    ESP_LOGW(TAG, "Injected a stray PPS edge");
  }
  if (this->glitch_end_us_ != 0 && now >= this->glitch_end_us_) {
    gpio_set_level(pps, 0);
    this->glitch_end_us_ = 0;
  }

  if (this->epoch_pending_ && now >= this->epoch_due_us_) {
    this->epoch_pending_ = false;
    if (this->nmea_enabled_)
      this->send_epoch_(this->first_epoch_s_ + this->handled_pulse_ - 1 + this->epoch_offset_s_);
  }

  this->read_commands_();
}

void GNSSSim::send_nmea_(const std::string &body) {
  uint8_t checksum = 0;
  for (char c : body)
    checksum ^= static_cast<uint8_t>(c);
  char tail[8];
  snprintf(tail, sizeof(tail), "*%02X\r\n", checksum);
  std::string line = "$" + body + tail;
  uart_write_bytes(static_cast<uart_port_t>(this->uart_num_), line.data(), line.size());
}

// The default sentence set of a u-blox 6/7, in its order, so the UART carries a realistic load
void GNSSSim::send_epoch_(int64_t utc_s) {
  time_t t = static_cast<time_t>(utc_s - (this->week_rollover_ ? GPS_WEEK_ROLLOVER_S : 0));
  struct tm tm;
  gmtime_r(&t, &tm);
  char hms[16], buf[128];
  snprintf(hms, sizeof(hms), "%02d%02d%02d.00", tm.tm_hour, tm.tm_min, tm.tm_sec);
  char status = this->fix_ ? 'A' : 'V';

  snprintf(buf, sizeof(buf), "GPRMC,%s,%c,4118.8000,N,08143.4000,W,0.012,,%02d%02d%02d,,,%c", hms, status, tm.tm_mday,
           tm.tm_mon + 1, tm.tm_year % 100, this->fix_ ? 'A' : 'N');
  this->send_nmea_(buf);
  this->send_nmea_("GPVTG,,T,,M,0.012,N,0.022,K,A");
  snprintf(buf, sizeof(buf), "GPGGA,%s,4118.8000,N,08143.4000,W,%d,%02d,0.92,265.1,M,-33.9,M,,", hms, this->fix_ ? 1 : 0,
           this->fix_ ? this->satellites_ : 0);
  this->send_nmea_(buf);
  this->send_nmea_("GPGSA,A,3,02,05,12,13,15,18,20,25,29,,,,1.63,0.92,1.35");
  this->send_nmea_("GPGSV,3,1,11,02,23,297,28,05,65,265,35,12,28,207,30,13,44,128,33");
  this->send_nmea_("GPGSV,3,2,11,15,71,064,38,18,32,312,27,20,18,156,22,25,12,246,24");
  this->send_nmea_("GPGSV,3,3,11,29,40,070,34,46,28,230,,48,31,226,");
  snprintf(buf, sizeof(buf), "GPGLL,4118.8000,N,08143.4000,W,%s,%c,%c", hms, status, this->fix_ ? 'A' : 'N');
  this->send_nmea_(buf);
}

void GNSSSim::send_ubx_(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len) {
  uint8_t frame[8 + 32];
  if (len > 32)
    return;
  frame[0] = 0xB5;
  frame[1] = 0x62;
  frame[2] = cls;
  frame[3] = id;
  frame[4] = len & 0xFF;
  frame[5] = len >> 8;
  if (len > 0)
    memcpy(frame + 6, payload, len);
  uint8_t a = 0, b = 0;
  for (int i = 2; i < 6 + len; i++) {
    a += frame[i];
    b += a;
  }
  frame[6 + len] = a;
  frame[7 + len] = b;
  uart_write_bytes(static_cast<uart_port_t>(this->uart_num_), frame, 8 + len);
}

void GNSSSim::read_commands_() {
  auto port = static_cast<uart_port_t>(this->uart_num_);
  uint8_t byte;
  while (uart_read_bytes(port, &byte, 1, 0) == 1) {
    // Hunt for B5 62, then collect class, id, length, payload and checksum
    if (this->rx_len_ == 0 && byte != 0xB5)
      continue;
    if (this->rx_len_ == 1 && byte != 0x62) {
      this->rx_len_ = byte == 0xB5 ? 1 : 0;
      continue;
    }
    this->rx_[this->rx_len_++] = byte;
    if (this->rx_len_ < 6)
      continue;
    uint16_t len = this->rx_[4] | (this->rx_[5] << 8);
    if (len > sizeof(this->rx_) - 8) {
      this->rx_len_ = 0;
      continue;
    }
    if (this->rx_len_ < 8 + len)
      continue;
    uint8_t a = 0, b = 0;
    for (int i = 2; i < 6 + len; i++) {
      a += this->rx_[i];
      b += a;
    }
    if (a == this->rx_[6 + len] && b == this->rx_[7 + len])
      this->handle_ubx_(this->rx_[2], this->rx_[3], this->rx_ + 6, len);
    this->rx_len_ = 0;
  }
}

void GNSSSim::handle_ubx_(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len) {
  if (!this->answer_ubx_)
    return;
  auto port = static_cast<uart_port_t>(this->uart_num_);
  uint8_t ack[2] = {cls, id};

  if (cls == 0x01 && id == 0x21 && len == 0) {  // poll NAV-TIMEUTC
    uint8_t p[20] = {0};
    p[19] = this->utc_valid_ ? 0x07 : 0x03;  // validTOW | validWKN | validUTC
    this->send_ubx_(0x01, 0x21, p, sizeof(p));
  } else if (cls == 0x06 && id == 0x00 && len == 20) {  // CFG-PRT: acknowledge at the old baud, then switch
    uint32_t baud = payload[8] | (payload[9] << 8) | (payload[10] << 16) | (static_cast<uint32_t>(payload[11]) << 24);
    this->send_ubx_(0x05, 0x01, ack, sizeof(ack));
    uart_wait_tx_done(port, pdMS_TO_TICKS(200));
    uart_set_baudrate(port, baud);
    this->baud_ = baud;
    ESP_LOGI(TAG, "CFG-PRT: now at %" PRIu32 " baud", baud);
  } else if (cls == 0x06 && id == 0x09) {  // CFG-CFG save
    this->saves_++;
    this->send_ubx_(0x05, 0x01, ack, sizeof(ack));
    ESP_LOGI(TAG, "CFG-CFG: configuration saved (%" PRIu32 " time%s since boot)", this->saves_,
             this->saves_ == 1 ? "" : "s");
  }
}

}  // namespace esphome::gnss_sim
