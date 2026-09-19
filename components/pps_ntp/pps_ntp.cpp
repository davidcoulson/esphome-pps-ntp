#include "pps_ntp.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

#include <esp_attr.h>
#include <esp_timer.h>
#include <lwip/sockets.h>
#include <unistd.h>

#include "esphome/components/network/util.h"
#include "esphome/core/log.h"

namespace esphome::pps_ntp {

static const char *const TAG = "pps_ntp";

static constexpr int MIN_PULSES_FOR_SYNC = 4;
static constexpr double MAX_RESIDUAL_US = 1000.0;     // a pulse further than this from the model is an outlier
static constexpr double MAX_RATE_ERROR_PPM = 500.0;   // reject fits that imply a broken crystal
static constexpr int64_t MAX_PULSE_GAP_S = 600;       // longer gaps are relabelled from NMEA
static constexpr int64_t FIX_STALE_US = 3000000;      // pulses only count while the receiver reports a fix
static constexpr int64_t RMC_MAX_DELAY_US = 950000;   // RMC must follow its pulse within this window
static constexpr uint32_t UBX_POLL_INTERVAL_MS = 10000;
static constexpr uint32_t UBX_ABSENT_TIMEOUT_MS = 60000;
static constexpr uint32_t BAUD_VERIFY_MS = 4000;
static constexpr uint32_t BAUD_RETRY_MS = 5000;
static constexpr uint8_t BAUD_MAX_ATTEMPTS = 3;
static constexpr uint64_t NTP_UNIX_OFFSET = 2208988800ULL;  // seconds from 1900 to 1970
static constexpr double DISPERSION_BASE_US = 20.0;
static constexpr double HOLDOVER_DRIFT_PPM = 5.0;

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

static int64_t utc_us_at(const ClockModel &model, int64_t local_us) {
  double elapsed = static_cast<double>(local_us - model.anchor_local_us) * (1e6 / model.local_us_per_s);
  return model.anchor_utc_s * 1000000LL + llround(elapsed);
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
  this->pps_pin_->setup();
  this->pps_pin_->attach_interrupt(PPSNTPServer::pps_isr, this, gpio::INTERRUPT_RISING_EDGE);

  this->original_baud_ = this->parent_->get_baud_rate();
  if (this->gnss_baud_rate_ != 0 && this->gnss_baud_rate_ != this->original_baud_)
    this->start_baud_switch_();
}

void PPSNTPServer::loop() {
  // The socket API needs lwIP's tcpip thread, which the network component only brings up after
  // our setup(); calling socket() earlier asserts on an uninitialised lwIP mutex
  if (this->task_ == nullptr && network::is_connected()) {
    if (xTaskCreate(PPSNTPServer::ntp_task, "pps_ntp", 4096, this, 10, &this->task_) != pdPASS) {
      ESP_LOGE(TAG, "Could not start NTP server task");
      this->mark_failed();
      return;
    }
    ESP_LOGI(TAG, "Network up; serving NTP on UDP port %u", this->port_);
  }

  uint8_t byte;
  while (this->available() > 0 && this->read_byte(&byte))
    this->feed_byte_(byte);

  // Take the latest pulse from the ISR; re-read if one landed while copying the 64-bit timestamp
  uint32_t count = this->isr_pulse_count_;
  if (count != this->seen_pulse_count_) {
    int64_t pulse_us = this->isr_pulse_us_;
    while (count != this->isr_pulse_count_) {
      count = this->isr_pulse_count_;
      pulse_us = this->isr_pulse_us_;
    }
    this->seen_pulse_count_ = count;
    this->handle_pulse_(pulse_us);
  }

  this->service_baud_switch_();

  uint32_t now_ms = millis();
  if (now_ms - this->last_ubx_poll_ms_ >= UBX_POLL_INTERVAL_MS) {
    this->last_ubx_poll_ms_ = now_ms;
    this->send_ubx_(0x01, 0x21, nullptr, 0);  // poll UBX-NAV-TIMEUTC for the validUTC flag
  }
  if (!this->ubx_seen_ && !this->utc_trusted_ && now_ms - this->boot_ms_ >= UBX_ABSENT_TIMEOUT_MS) {
    ESP_LOGW(TAG, "Receiver does not answer UBX polls; trusting NMEA UTC without leap-second confirmation");
    this->utc_trusted_ = true;
    this->publish_model_();
  }

  bool synced = this->is_synced_(this->model_, esp_timer_get_time());
  if (synced != this->last_synced_) {
    this->last_synced_ = synced;
    ESP_LOGI(TAG, "%s", synced ? "Synchronised to GNSS PPS; serving stratum 1" : "Lost synchronisation; serving stratum 16");
    if (this->synced_binary_sensor_ != nullptr)
      this->synced_binary_sensor_->publish_state(synced);
  }
}

void PPSNTPServer::update() {
  ClockModel model = this->get_model_();
  if (this->satellites_sensor_ != nullptr && this->satellites_ >= 0)
    this->satellites_sensor_->publish_state(this->satellites_);
  if (this->frequency_offset_sensor_ != nullptr && model.valid)
    this->frequency_offset_sensor_->publish_state(model.local_us_per_s - 1e6);
  if (this->pps_jitter_sensor_ != nullptr && model.valid)
    this->pps_jitter_sensor_->publish_state(std::sqrt(this->jitter_sq_us_));
  if (this->requests_sensor_ != nullptr)
    this->requests_sensor_->publish_state(this->requests_.load());
  if (this->synced_binary_sensor_ != nullptr)
    this->synced_binary_sensor_->publish_state(this->last_synced_);
}

void PPSNTPServer::dump_config() {
  ESP_LOGCONFIG(TAG, "PPS NTP Server:");
  LOG_PIN("  PPS Pin: ", this->pps_pin_);
  ESP_LOGCONFIG(TAG,
                "  Port: %u\n"
                "  Holdover: %us",
                this->port_, static_cast<unsigned>(this->holdover_us_ / 1000000));
  if (this->gnss_baud_rate_ != 0) {
    ESP_LOGCONFIG(TAG, "  GNSS baud rate: %u (from %u)", static_cast<unsigned>(this->gnss_baud_rate_),
                  static_cast<unsigned>(this->original_baud_));
  }
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Satellites", this->satellites_sensor_);
  LOG_SENSOR("  ", "Frequency Offset", this->frequency_offset_sensor_);
  LOG_SENSOR("  ", "PPS Jitter", this->pps_jitter_sensor_);
  LOG_SENSOR("  ", "Requests", this->requests_sensor_);
  LOG_BINARY_SENSOR("  ", "Synced", this->synced_binary_sensor_);
}

// ---------------------------------------------------------------------------
// Clock discipline

void PPSNTPServer::handle_pulse_(int64_t local_us) {
  this->last_pulse_us_ = local_us;
  this->last_pulse_labelled_ = false;

  // Without a fix the receiver's pulse is free-running; hold over on the local crystal instead
  bool fix_recent = this->last_rmc_valid_us_ != 0 && local_us - this->last_rmc_valid_us_ < FIX_STALE_US;
  if (this->hist_count_ == 0 || !fix_recent)
    return;  // the next valid RMC labels this pulse

  // Label by counting whole seconds since the last accepted pulse
  double rate = this->model_.valid ? this->model_.local_us_per_s : 1e6;
  int64_t elapsed = local_us - this->last_accepted_local_us_;
  int64_t seconds = llround(elapsed / rate);
  if (seconds < 1)
    return;  // spurious edge
  if (seconds > MAX_PULSE_GAP_S) {
    this->reset_discipline_("PPS gap too long");
    return;
  }
  double error = static_cast<double>(elapsed) - seconds * rate;
  double tolerance = 300.0 + 100e-6 * static_cast<double>(elapsed);
  if (std::fabs(error) > tolerance) {
    ESP_LOGV(TAG, "Ignoring PPS edge %.0f µs off the second", error);
    return;
  }

  this->last_pulse_labelled_ = true;
  this->accept_pulse_(local_us, this->last_accepted_utc_s_ + seconds);
}

void PPSNTPServer::accept_pulse_(int64_t local_us, int64_t utc_s) {
  if (this->hist_count_ > 0 && utc_s <= this->last_accepted_utc_s_)
    this->reset_discipline_("GNSS time went backwards");

  if (this->model_.valid) {
    double predicted = this->model_.anchor_local_us + (utc_s - this->model_.anchor_utc_s) * this->model_.local_us_per_s;
    double residual = static_cast<double>(local_us) - predicted;
    if (std::fabs(residual) > MAX_RESIDUAL_US) {
      if (++this->outliers_ < 3) {
        ESP_LOGW(TAG, "PPS pulse %.0f µs from prediction; ignoring", residual);
        return;
      }
      this->reset_discipline_("PPS phase stepped");
    } else {
      this->outliers_ = 0;
      this->jitter_sq_us_ = this->hist_count_ < 8 ? residual * residual
                                                  : 0.95 * this->jitter_sq_us_ + 0.05 * residual * residual;
    }
  }

  this->hist_local_[this->hist_head_] = local_us;
  this->hist_utc_[this->hist_head_] = utc_s;
  this->hist_head_ = (this->hist_head_ + 1) % HISTORY_SIZE;
  if (this->hist_count_ < HISTORY_SIZE)
    this->hist_count_++;
  this->last_accepted_local_us_ = local_us;
  this->last_accepted_utc_s_ = utc_s;

  // Least-squares fit of local time against UTC seconds, relative to the newest pulse
  double rate = 1e6;
  double fitted_offset = 0;
  if (this->hist_count_ >= 2) {
    double sum_x = 0, sum_y = 0;
    for (int i = 0; i < this->hist_count_; i++) {
      sum_x += static_cast<double>(this->hist_utc_[i] - utc_s);
      sum_y += static_cast<double>(this->hist_local_[i] - local_us);
    }
    double mean_x = sum_x / this->hist_count_;
    double mean_y = sum_y / this->hist_count_;
    double sxx = 0, sxy = 0;
    for (int i = 0; i < this->hist_count_; i++) {
      double dx = static_cast<double>(this->hist_utc_[i] - utc_s) - mean_x;
      double dy = static_cast<double>(this->hist_local_[i] - local_us) - mean_y;
      sxx += dx * dx;
      sxy += dx * dy;
    }
    rate = sxy / sxx;
    fitted_offset = mean_y - rate * mean_x;
    if (std::fabs(rate - 1e6) > MAX_RATE_ERROR_PPM) {
      this->reset_discipline_("implausible crystal rate");
      return;
    }
  }

  ClockModel next;
  next.valid = this->hist_count_ >= MIN_PULSES_FOR_SYNC;
  next.anchor_local_us = local_us + llround(fitted_offset);
  next.anchor_utc_s = utc_s;
  next.local_us_per_s = rate;
  next.last_pulse_local_us = local_us;
  next.utc_trusted = this->utc_trusted_;
  portENTER_CRITICAL(&this->lock_);
  this->model_ = next;
  portEXIT_CRITICAL(&this->lock_);
}

void PPSNTPServer::reset_discipline_(const char *reason) {
  ESP_LOGW(TAG, "Resetting clock discipline: %s", reason);
  this->hist_count_ = 0;
  this->hist_head_ = 0;
  this->outliers_ = 0;
  this->label_mismatches_ = 0;
  this->jitter_sq_us_ = 0;
  portENTER_CRITICAL(&this->lock_);
  this->model_.valid = false;
  portEXIT_CRITICAL(&this->lock_);
}

void PPSNTPServer::publish_model_() {
  portENTER_CRITICAL(&this->lock_);
  this->model_.utc_trusted = this->utc_trusted_;
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
        for (uint16_t i = 0; i < this->ubx_len_ - 2; i++) {
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
  if (star == nullptr || strlen(star) < 3)
    return;
  uint8_t checksum = 0;
  for (char *p = line + 1; p < star; p++)
    checksum ^= static_cast<uint8_t>(*p);
  if (checksum != static_cast<uint8_t>(strtoul(star + 1, nullptr, 16)))
    return;
  *star = '\0';
  this->nmea_ok_since_switch_ = true;

  char *fields[20];
  int count = 0;
  char *p = line;
  fields[count++] = p;
  while ((p = strchr(p, ',')) != nullptr && count < 20) {
    *p++ = '\0';
    fields[count++] = p;
  }

  // Address is $ttSSS; the talker (GP, GN, GL...) is ignored
  const char *type = fields[0] + 3;
  if (strcmp(type, "RMC") == 0) {
    this->handle_rmc_(fields, count);
  } else if (strcmp(type, "GGA") == 0 && count > 7 && fields[7][0] != '\0') {
    this->satellites_ = atoi(fields[7]);
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

  int64_t utc_s = days_from_civil(2000 + two_digits(date + 4), two_digits(date + 2), two_digits(date)) * 86400 +
                  two_digits(time) * 3600 + two_digits(time + 2) * 60 + two_digits(time + 4);
  this->last_rmc_valid_us_ = now_us;

  // A u-blox receiver sends the RMC for an epoch shortly after that epoch's pulse
  int64_t age = now_us - this->last_pulse_us_;
  if (this->last_pulse_us_ == 0 || age <= 0 || age > RMC_MAX_DELAY_US)
    return;

  if (this->last_pulse_labelled_) {
    if (this->last_accepted_utc_s_ == utc_s) {
      this->label_mismatches_ = 0;
      return;
    }
    if (++this->label_mismatches_ < 3)
      return;
    this->reset_discipline_("NMEA time disagrees with PPS count");
  }

  this->last_pulse_labelled_ = true;
  this->accept_pulse_(this->last_pulse_us_, utc_s);
}

void PPSNTPServer::handle_ubx_(uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t len) {
  this->ubx_seen_ = true;
  if (msg_class != 0x01 || msg_id != 0x21 || len < 20)
    return;
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
// NTP server task: timestamps as close to the socket as ESPHome allows

void PPSNTPServer::ntp_task(void *arg) { static_cast<PPSNTPServer *>(arg)->ntp_loop_(); }

void PPSNTPServer::ntp_loop_() {
  for (;;) {
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    struct sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(this->port_);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(sock, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      close(sock);
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    for (;;) {
      uint8_t request[68];
      struct sockaddr_in source {};
      socklen_t source_len = sizeof(source);
      int received = recvfrom(sock, request, sizeof(request), 0, reinterpret_cast<struct sockaddr *>(&source),
                              &source_len);
      int64_t rx_local = esp_timer_get_time();
      if (received < 0)
        break;
      if (received < 48)
        continue;
      uint8_t version = (request[0] >> 3) & 0x07;
      uint8_t mode = request[0] & 0x07;
      if (mode != 3 || version < 1 || version > 4)
        continue;

      ClockModel model = this->get_model_();
      if (!model.valid)
        continue;  // never synchronised: stay silent rather than hand out a bogus time
      bool synced = this->is_synced_(model, rx_local);
      double age_s = static_cast<double>(rx_local - model.last_pulse_local_us) / 1e6;
      double dispersion_us = DISPERSION_BASE_US + HOLDOVER_DRIFT_PPM * age_s;

      uint8_t reply[48] = {0};
      reply[0] = ((synced ? 0 : 3) << 6) | (version << 3) | 4;  // LI, VN, mode = server
      reply[1] = synced ? 1 : 16;
      reply[2] = request[2];                       // poll
      reply[3] = static_cast<uint8_t>(-20);       // precision ~1 µs
      put_be32(reply + 8, static_cast<uint32_t>(std::min(dispersion_us * 65536.0 / 1e6, 4294967295.0)));
      memcpy(reply + 12, "GPS", 4);
      put_ntp_timestamp(reply + 16, model.anchor_utc_s * 1000000LL);
      memcpy(reply + 24, request + 40, 8);  // originate = client's transmit
      put_ntp_timestamp(reply + 32, utc_us_at(model, rx_local));
      put_ntp_timestamp(reply + 40, utc_us_at(model, esp_timer_get_time()));
      sendto(sock, reply, sizeof(reply), 0, reinterpret_cast<struct sockaddr *>(&source), source_len);
      this->requests_++;
    }
    close(sock);
  }
}

}  // namespace esphome::pps_ntp
