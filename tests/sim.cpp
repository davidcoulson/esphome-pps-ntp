// Host-side simulation of the pulse/NMEA/UBX state machine. Not a substitute for hardware:
// it checks the labelling and discipline logic against scripted receiver behaviour.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <random>
#include <string>

#include "../components/pps_ntp/pps_ntp.h"

int64_t g_now_us = 0;
bool g_log = false;
static bool g_counter_underflow = false;  // set if any scenario accepted more pulses than it saw

using namespace esphome;
using namespace esphome::pps_ntp;

static const int64_t T0 = 1789819200;  // 2026-09-19 12:00:00 UTC
static const double PPM = 23.0;        // crystal runs fast
static const int64_t L0 = 1200000;     // local time of the first pulse (a powered receiver is already talking at boot)

struct Sim : PPSNTPServer {
  friend int main(int, char **);
  uart::UARTComponent uart;
  InternalGPIOPin pin;
  sensor::Sensor sats;
  sensor::Sensor cno, strong, hdop, rejected, nmea_err, pulse_age;
  sensor::Sensor latitude, longitude, altitude;
  bool tracking = true;  // false: satellites in view but none tracked (e.g. antenna unplugged)
  std::mt19937 rng{42};

  // Receiver behaviour, scriptable per test
  uint32_t module_baud = 9600;
  bool module_accepts_cfg_prt = true;
  bool fix = true;
  std::function<bool(int64_t)> no_fix_at = [](int64_t) { return false; };  // epochs whose RMC says V
  bool pps_on = true;
  bool nmea_on = true;          // sentences keep coming without PPS (a receiver with no fix still talks)
  bool capture = false;         // MCPWM-like capture: sub-µs edge times handed straight to the discipline
  double capture_frac_us = 0.375;
  bool answers_ubx = true;      // answers UBX at all
  bool answers_timeutc = true;  // answers NAV-TIMEUTC specifically (some receivers know only some messages)
  bool utc_valid = true;
  int64_t date_offset_s = 0;   // e.g. -1024 weeks
  int64_t t0 = T0;             // UTC of pulse k = 0
  int64_t leap_k = -1;         // the pulse that marks 23:59:60 (an inserted leap second); -1 = none
  bool answers_timels = false; // u-blox 8+: announces the leap in advance
  int li_seen[4] = {0, 0, 0, 0};
  int64_t phase_offset_us = 0;  // PPS phase step
  double jitter_us = 3.0;
  std::function<bool(int64_t)> stalled = [](int64_t) { return false; };
  std::function<bool(int64_t)> drop_epoch = [](int64_t) { return false; };  // sentences lost, e.g. RX overflow
  std::function<int64_t(int64_t)> glitch_after_pulse_us = [](int64_t) { return 0; };  // 0 = none
  std::function<int64_t(int64_t)> pulse_err_us = [](int64_t) { return 0; };  // receiver error on pulse k (not truth)

  // Results
  double worst_served_error_us = 0;
  int64_t first_synced_s = -1;
  int served = 0, silent = 0, unsynced = 0;
  bool saw_relocking = false;  // the holdover relock path (a keep_time reset) ran at some point
  size_t tx_seen = 0;

  Sim() {
    this->parent_ = &uart;
    this->set_pps_pin(&pin);
    this->set_satellites_sensor(&sats);
    this->set_signal_strength_sensor(&cno);
    this->set_strong_satellites_sensor(&strong);
    this->set_hdop_sensor(&hdop);
    this->set_rejected_pulses_sensor(&rejected);
    this->set_nmea_errors_sensor(&nmea_err);
    this->set_pulse_age_sensor(&pulse_age);
    this->set_latitude_sensor(&latitude);
    this->set_longitude_sensor(&longitude);
    this->set_altitude_sensor(&altitude);
  }

  static std::string nmea(const std::string &body) {
    uint8_t ck = 0;
    for (char c : body) ck ^= static_cast<uint8_t>(c);
    char tail[8];
    snprintf(tail, sizeof(tail), "*%02X\r\n", ck);
    return "$" + body + tail;
  }
  void rx(const std::string &s) {
    if (uart.baud != module_baud) return;  // wrong baud: nothing intelligible arrives
    for (char c : s) uart.rx.push_back(static_cast<uint8_t>(c));
  }
  void rx_raw(const std::vector<uint8_t> &v) { for (auto b : v) uart.rx.push_back(b); }

  static std::vector<uint8_t> ubx(uint8_t cls, uint8_t id, const std::vector<uint8_t> &payload) {
    std::vector<uint8_t> f = {0xB5, 0x62, cls, id, static_cast<uint8_t>(payload.size() & 0xFF), static_cast<uint8_t>(payload.size() >> 8)};
    f.insert(f.end(), payload.begin(), payload.end());
    uint8_t a = 0, b = 0;
    for (size_t i = 2; i < f.size(); i++) { a += f[i]; b += a; }
    f.push_back(a); f.push_back(b);
    return f;
  }

  void send_epoch(int64_t k) {
    // After an inserted second the receiver's UTC is one behind the pulse count
    bool in_leap = leap_k >= 0 && k == leap_k;
    int64_t utc = t0 + k - ((leap_k >= 0 && k >= leap_k) ? 1 : 0);
    time_t t = static_cast<time_t>(utc + date_offset_s);
    struct tm tm; gmtime_r(&t, &tm);
    if (in_leap) tm.tm_sec = 60;  // 23:59:59 + 1
    char body[128];
    snprintf(body, sizeof(body), "GPRMC,%02d%02d%02d.00,%c,4124.8963,N,08151.6838,W,0.0,0.0,%02d%02d%02d,,,A",
             tm.tm_hour, tm.tm_min, tm.tm_sec, fix && !no_fix_at(k) ? 'A' : 'V', tm.tm_mday, tm.tm_mon + 1, tm.tm_year % 100);
    rx(nmea(body));
    // Quality 0 without a fix; the receiver keeps repeating its last position, which must not count.
    // The altitude alternates a metre either side of 250 so the interval mean is exercised.
    snprintf(body, sizeof(body), "GPGGA,%02d%02d%02d.00,4124.8963,N,08151.6838,W,%d,09,0.9,%.1f,M,-33.0,M,,",
             tm.tm_hour, tm.tm_min, tm.tm_sec, fix ? 1 : 0, k % 2 ? 251.0 : 249.0);
    rx(nmea(body));
    // Two GSV messages: 6 tracked satellites (45+38+41+33+29+22 = 208, mean 34.667) and one in view
    // but untracked, whose empty C/N0 field must not count
    if (tracking) {
      rx(nmea("GPGSV,2,1,06,02,23,297,45,05,65,265,38,12,28,207,41,13,44,128,33"));
      rx(nmea("GPGSV,2,2,06,15,71,064,29,18,32,312,22,20,18,156,"));
    } else {
      rx(nmea("GPGSV,2,1,06,02,23,297,,05,65,265,,12,28,207,,13,44,128,"));
      rx(nmea("GPGSV,2,2,06,15,71,064,,18,32,312,,20,18,156,"));
    }
  }

  // Watch what the component transmits: answer polls, obey CFG-PRT
  void service_tx() {
    auto &tx = uart.tx;
    while (tx_seen + 8 <= tx.size()) {
      if (tx[tx_seen] != 0xB5) { tx_seen++; continue; }
      uint16_t len = tx[tx_seen + 4] | (tx[tx_seen + 5] << 8);
      if (tx_seen + 8 + len > tx.size()) break;
      uint8_t cls = tx[tx_seen + 2], id = tx[tx_seen + 3];
      bool heard = uart.tx_baud[tx_seen] == module_baud;  // the baud at the moment it was sent
      if (heard && cls == 0x01 && id == 0x21 && answers_ubx && answers_timeutc) {
        std::vector<uint8_t> p(20, 0);
        p[19] = utc_valid ? 0x07 : 0x03;
        if (uart.baud == module_baud) rx_raw(ubx(0x01, 0x21, p));
      } else if (heard && cls == 0x01 && id == 0x26 && answers_timels) {
        std::vector<uint8_t> p(24, 0);
        int64_t now_k = static_cast<int64_t>((g_now_us - L0) / 1000000);
        int32_t to_event = leap_k >= 0 ? static_cast<int32_t>(leap_k + 1 - now_k) : 0;  // to the midnight after it
        p[9] = 18;
        p[11] = (leap_k >= 0 && to_event > 0) ? 1 : 0;
        p[12] = to_event & 0xFF; p[13] = (to_event >> 8) & 0xFF; p[14] = (to_event >> 16) & 0xFF; p[15] = (to_event >> 24) & 0xFF;
        p[23] = 0x03;
        rx_raw(ubx(0x01, 0x26, p));
      } else if (heard && cls == 0x06 && id == 0x3E && len == 0 && answers_ubx) {
        // CFG-GNSS poll: a NEO-7M-like answer, GLONASS present but switched off
        std::vector<uint8_t> p = {0x00, 22, 22, 4};
        auto block = [&](uint8_t gnss_id, uint8_t max_ch, bool on) {
          p.insert(p.end(), {gnss_id, 8, max_ch, 0, static_cast<uint8_t>(on ? 1 : 0), 0, 0, 0});
        };
        block(0, 16, true); block(1, 3, true); block(5, 3, true); block(6, 14, false);
        rx_raw(ubx(0x06, 0x3E, p));
      } else if (heard && answers_ubx && (cls == 0x06 && (id == 0x24 || id == 0x01))) {
        std::vector<uint8_t> ack = {cls, id};
        rx_raw(ubx(0x05, 0x01, ack));
      } else if (heard && cls == 0x06 && id == 0x00 && module_accepts_cfg_prt) {
        module_baud = tx[tx_seen + 6 + 8] | (tx[tx_seen + 6 + 9] << 8) | (tx[tx_seen + 6 + 10] << 16) | (tx[tx_seen + 6 + 11] << 24);
      }
      tx_seen += 8 + len;
    }
  }
  int count_tx(uint8_t cls, uint8_t id) {
    int n = 0;
    for (size_t i = 0; i + 8 <= uart.tx.size(); i++)
      if (uart.tx[i] == 0xB5 && uart.tx[i + 1] == 0x62 && uart.tx[i + 2] == cls && uart.tx[i + 3] == id) n++;
    return n;
  }

  // UTC as an NTP server can express it: an inserted second repeats 23:59:59
  long double truth_us(int64_t local_us, int64_t phase) {
    long double since = (static_cast<long double>(local_us - L0 - phase)) / (1.0L + PPM * 1e-6L);
    long double utc = static_cast<long double>(t0) * 1e6L + since;
    if (leap_k >= 0 && since >= leap_k * 1e6L) utc -= 1e6L;
    return utc;
  }

  void query(bool check) {
    uint8_t req[48] = {0x23}, rep[48];
    if (!this->build_reply_(req, g_now_us, CLIENT, 0, rep)) { silent++; return; }
    if (rep[1] != 1) { unsynced++; return; }
    li_seen[rep[0] >> 6]++;
    served++;
    uint32_t sec = (rep[40] << 24) | (rep[41] << 16) | (rep[42] << 8) | rep[43];
    uint32_t frac = (rep[44] << 24) | (rep[45] << 16) | (rep[46] << 8) | rep[47];
    long double got = (static_cast<long double>(sec) - 2208988800.0L) * 1e6L + frac * 1e6L / 4294967296.0L;
    double err = static_cast<double>(got - truth_us(g_now_us, phase_offset_us));
    if (check && std::fabs(err) > std::fabs(worst_served_error_us)) worst_served_error_us = err;
  }

  void run(double seconds, bool check = true) {
    std::normal_distribution<double> noise(0.0, jitter_us);
    int64_t end = g_now_us + std::llround(seconds * 1e6);
    while (g_now_us < end) {
      int64_t tick_end = g_now_us + 1000;
      // Epoch (pulse, optional glitch, sentences) due in this millisecond?
      long double per = 1e6L * (1.0L + PPM * 1e-6L);
      int64_t k = static_cast<int64_t>(std::ceil((g_now_us - L0 - phase_offset_us) / per));  // next epoch at or after now
      int64_t pulse_at = L0 + phase_offset_us + static_cast<int64_t>(std::llround(k * per));
      if (k >= 0 && pulse_at < tick_end && pulse_at >= g_now_us && k != last_k) {
        last_k = k;
        if (pps_on) {
          g_now_us = pulse_at + static_cast<int64_t>(std::llround(noise(rng))) + pulse_err_us(k);
          if (capture) {
            this->handle_pulse_(static_cast<double>(g_now_us) + capture_frac_us);
          } else {
            PPSNTPServer::pps_isr(this);
          }
          int64_t g = glitch_after_pulse_us(k);
          glitch_due_us = g ? pulse_at + g : 0;
        }
        if (nmea_on) {
          epoch_due_us = pulse_at + 100000;
          epoch_k = k;
        }
      }
      if (glitch_due_us && glitch_due_us < tick_end) { g_now_us = glitch_due_us; PPSNTPServer::pps_isr(this); glitch_due_us = 0; }
      if (epoch_due_us && epoch_due_us < tick_end) { if (!drop_epoch(epoch_k)) send_epoch(epoch_k); epoch_due_us = 0; }
      g_now_us = tick_end;
      if ((g_now_us / 1000) % 10 == 0 && !stalled(g_now_us)) {
        this->loop();
        service_tx();
      }
      saw_relocking |= this->relocking_;
      if (this->pulses_accepted_ > this->edges_seen_ && !g_counter_underflow) {
        g_counter_underflow = true;
        printf("    !! pulses_accepted (%u) > edges_seen (%u) at t=%.3f s\n", this->pulses_accepted_, this->edges_seen_,
               g_now_us / 1e6);
      }
      if ((g_now_us / 1000) % 250 == 0) {
        query(check);
        if (first_synced_s < 0 && served > 0) first_synced_s = g_now_us / 1000000;
      }
    }
  }
  int64_t last_k = -1, epoch_due_us = 0, epoch_k = 0, glitch_due_us = 0;

  // The client address every scripted query comes from (an IPv4-mapped 192.0.2.1)
  static constexpr uint8_t CLIENT[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xFF, 0xFF, 192, 0, 2, 1};
};

static int failures = 0;
static void check(bool ok, const char *what) {
  printf("    %s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) failures++;
}
static void begin(const char *name) { printf("\n%s\n", name); g_now_us = 1000000; }

int main(int argc, char **argv) {
  g_log = argc > 1;

  begin("A. Normal start: module at 9600, switch to 115200, UTC valid");
  {
    Sim s; s.set_gnss_baud_rate(115200); s.setup(); s.run(120);
    printf("    first stratum-1 reply at t=%llds, worst served error %.1f us, served=%d\n", (long long) s.first_synced_s, s.worst_served_error_us, s.served);
    check(s.module_baud == 115200 && s.uart.baud == 115200, "receiver and UART end at 115200");
    check(s.count_tx(0x06, 0x09) == 1, "config saved exactly once");
    check(s.first_synced_s > 0 && s.first_synced_s < 25, "synced within 25 s");
    check(std::fabs(s.worst_served_error_us) < 50, "served time within 50 us of truth");
    check(s.count_tx(0x06, 0x00) == 1, "CFG-PRT sent once");
  }

  begin("B. Module already at target baud (#7): probe succeeds, nothing sent or saved");
  {
    Sim s; s.module_baud = 115200; s.set_gnss_baud_rate(115200); s.setup(); s.run(30);
    check(s.count_tx(0x06, 0x00) == 0, "no CFG-PRT sent");
    check(s.count_tx(0x06, 0x09) == 0, "no CFG-CFG save");
    check(s.served > 0, "synced");
  }

  begin("C. Glitch edges (#1): 50 ms after every 7th pulse, 600 ms after every 11th");
  {
    Sim s; s.setup();
    s.glitch_after_pulse_us = [](int64_t k) -> int64_t { return k % 7 == 0 ? 50000 : (k % 11 == 0 ? 600000 : 0); };
    s.run(30); int silent_before = s.silent;
    s.run(200);
    check(s.silent == silent_before, "never goes silent once synced (no discipline reset)");
    check(std::fabs(s.worst_served_error_us) < 50, "served time within 50 us of truth");
    s.update();
    check(s.rejected.state > 20, "rejected-pulse counter recorded the glitches");
    check(s.nmea_err.state == 0, "no NMEA checksum errors");
  }

  begin("C2. GPIO interrupt: three edges in a row 700 us late, inside the 1000 us max_residual");
  {
    Sim s; s.setup(); s.run(60);
    int64_t kf = s.last_k + 2;
    s.pulse_err_us = [kf](int64_t k) -> int64_t { return k >= kf && k < kf + 3 ? 700 : 0; };
    int unsynced_before = s.unsynced, silent_before = s.silent;
    uint32_t a0 = s.pulses_accepted_, e0 = s.edges_seen_;
    s.run(10);
    check(s.unsynced == unsynced_before && s.silent == silent_before, "late edges inside max_residual don't reset the discipline");
    check(s.pulses_accepted_ - a0 == s.edges_seen_ - e0, "and they are used, not dropped as off-second edges");
  }

  begin("D. Loop stalls 1.2 s right at the first label and loses a sentence (#2)");
  {
    Sim s; s.setup();
    // Stall so RMC(k=0) is parsed only after pulse k=1 has been seen
    s.stalled = [](int64_t now) { return now > L0 - 50000 && now < L0 + 1150000; };
    // ...and the next sentence is lost in the same stall (RX overflow), so the cross-check starts a second late
    s.drop_epoch = [](int64_t k) { return k == 1; };
    s.run(60);
    printf("    worst served error %.1f us, first sync t=%llds\n", s.worst_served_error_us, (long long) s.first_synced_s);
    check(s.served > 0, "eventually synced");
    check(std::fabs(s.worst_served_error_us) < 1000, "never served a time off by a second");
  }

  begin("E. GPS week rollover (#3): receiver reports a date 1024 weeks in the past");
  {
    Sim s; s.date_offset_s = -619315200; s.setup(); s.run(40);
    check(s.served > 0, "synced");
    check(std::fabs(s.worst_served_error_us) < 50, "served the corrected date, within 50 us");
  }

  begin("F. UBX frame with length 0xFFFF (#5), then garbage");
  {
    Sim s; s.setup(); s.run(20);
    s.rx_raw({0xB5, 0x62, 0x01, 0x21, 0xFF, 0xFF, 0x00, 0x00, 0x11, 0x22, 0x33, 0x44});
    for (int i = 0; i < 600; i++) s.rx_raw({static_cast<uint8_t>(i * 37)});
    int silent_before = s.silent; s.run(30);
    check(s.silent == silent_before && s.served > 0, "parser recovers; still serving");
  }

  begin("G. Receiver never answers UBX");
  {
    Sim a; a.answers_ubx = false; a.setup(); a.run(90);
    check(a.served > 0 && a.first_synced_s >= 60, "default: trusted only after the 60 s fallback");
    Sim b; b.answers_ubx = false; b.set_require_utc_valid(true); b.setup(); b.run(300);
    check(b.served == 0 && b.unsynced > 0, "require_utc_valid: never claims stratum 1");
    // Answers CFG-GNSS but not NAV-TIMEUTC: the fallback must still fire, or it would never serve
    Sim c; c.answers_timeutc = false; c.setup(); c.run(90);
    check(c.served > 0 && c.first_synced_s >= 60, "answers some UBX but not NAV-TIMEUTC: still falls back");
  }

  begin("H. UTC not yet valid for 200 s (cold start), then valid");
  {
    Sim s; s.utc_valid = false; s.setup(); s.run(200);
    check(s.served == 0, "no stratum-1 replies while the receiver says UTC is not valid");
    s.utc_valid = true; s.run(30);
    check(s.served > 0, "serves once UTC is confirmed");
  }

  begin("H2. Cold start: NMEA two seconds off while validUTC is false, then both corrected together");
  {
    Sim s; s.utc_valid = false; s.date_offset_s = -2; s.set_require_utc_valid(true); s.setup();
    s.run(60, false);
    check(s.served == 0, "nothing served at stratum 1 while UTC is not valid");
    s.date_offset_s = 0; s.utc_valid = true; s.run(30);
    printf("    worst served error %.1f us, served=%d\n", s.worst_served_error_us, s.served);
    check(s.served > 0, "serves once the fit is relabelled");
    check(std::fabs(s.worst_served_error_us) < 50, "never serves the fit labelled before UTC was valid");
  }

  begin("I. Fix lost for 120 s (pulses free-run, drifting to +2 ms), then regained");
  {
    Sim s; s.setup(); s.run(60);
    int unsynced_before = s.unsynced;
    int64_t kf = s.last_k + 1, kr = kf + 120;
    s.no_fix_at = [kf, kr](int64_t k) { return k >= kf && k < kr; };
    // Free-running from the first epoch without a fix; back on the second once the fix returns
    s.pulse_err_us = [kf, kr](int64_t k) -> int64_t { return k > kf && k < kr ? (k - kf) * 2000 / 120 : 0; };
    s.run(120);
    check(s.unsynced == unsynced_before, "holds over as stratum 1 inside the holdover window");
    check(std::fabs(s.worst_served_error_us) < 200, "holdover error under 200 us after 120 s");
    s.run(60);
    check(std::fabs(s.worst_served_error_us) < 200, "re-locks without serving a bad time");
    check(s.unsynced == unsynced_before, "and without dropping to stratum 16");
  }

  begin("I2. Receiver reports V: the very next pulse is no longer used (fit_window 8, pulses free-run 100 us/s)");
  {
    Sim s; s.set_fit_window(8); s.setup(); s.run(60);
    int64_t k0 = s.last_k, kf = k0 + 2;  // the receiver loses its fix two epochs from now
    s.no_fix_at = [kf](int64_t k) { return k >= kf; };
    s.pulse_err_us = [kf](int64_t k) -> int64_t { return k > kf ? (k - kf) * 100 : 0; };
    uint32_t a0 = s.pulses_accepted_;
    s.worst_served_error_us = 0; s.run(120);
    printf("    pulses used after the last good one: %u (expected %lld), worst served error %.1f us\n",
           s.pulses_accepted_ - a0, (long long) (kf - k0), s.worst_served_error_us);
    check(s.pulses_accepted_ - a0 == kf - k0, "only pulses up to the first V epoch enter the fit");
    check(std::fabs(s.worst_served_error_us) < 200, "holdover unaffected by the free-running pulses");
  }

  begin("J. Receiver unplugged for 20 minutes (holdover is 15); sentences return just before PPS");
  {
    Sim s; s.setup(); s.run(60);
    s.pps_on = false; s.nmea_on = false; s.run(20 * 60, false);
    check(s.unsynced > 0, "drops to stratum 16 after the holdover expires");
    int before = s.served, silent_before = s.silent;
    s.nmea_on = true; s.run(1.5, false);  // RMCs with a fix first, as after a receiver restart
    s.pps_on = true; s.run(30);
    check(s.saw_relocking, "the first pulse back took the long-gap path (relabel from NMEA)");
    check(s.served > before, "recovers when the receiver comes back");
    check(s.silent == silent_before, "answers (stratum 16) rather than going silent while re-locking");
  }

  begin("J2. PPS lost for 11 minutes (past the 600 s relabel limit, inside the 15 min holdover), crystal 150 us off");
  {
    Sim s; s.set_max_residual_us(100); s.setup(); s.run(60);
    int unsynced_before = s.unsynced, silent_before = s.silent;
    s.pps_on = false; s.fix = false; s.run(11 * 60, false);  // the receiver keeps talking, reporting V
    s.phase_offset_us = 150;                                  // the held model is now 150 us off the second
    s.fix = true; s.run(1.5, false);                          // 'A' again just before the pulses return
    s.pps_on = true; s.worst_served_error_us = 0; s.run(30);
    check(s.saw_relocking, "the gap took the relabel-and-relock path");
    check(s.unsynced == unsynced_before && s.silent == silent_before,
          "kept serving stratum 1 on holdover through the re-lock (no outlier test against the held model)");
    check(std::fabs(s.worst_served_error_us) < 200, "held-over time stayed within 200 us");
    s.worst_served_error_us = 0; s.run(30);
    check(std::fabs(s.worst_served_error_us) < 50, "locked again on the new pulses");
  }

  begin("K. PPS phase steps by 5 ms (receiver re-acquisition)");
  {
    Sim s; s.setup(); s.run(60);
    int silent_before = s.silent, unsynced_before = s.unsynced;
    s.phase_offset_us = 5000; s.run(10, false);
    check(s.silent == silent_before && s.unsynced > unsynced_before, "a phase step drops to stratum 16, not silence");
    s.worst_served_error_us = 0; s.run(60);
    check(std::fabs(s.worst_served_error_us) < 50, "re-disciplines to the new phase within 10 s");
  }

  begin("K2. Hardware capture (sub-us edges): NMEA jumps a second, the fit relabels; each edge counted once");
  {
    Sim s; s.capture = true; s.set_max_residual_us(200); s.setup(); s.hw_capture_ = true; s.run(30);
    check(s.served > 0 && std::fabs(s.worst_served_error_us) < 50, "synced on sub-microsecond edges");
    uint32_t a0 = s.pulses_accepted_, e0 = s.edges_seen_;
    s.date_offset_s = 1; s.run(15, false);
    printf("    edges %u, counted as accepted %u\n", s.edges_seen_ - e0, s.pulses_accepted_ - a0);
    check(s.pulses_accepted_ - a0 <= s.edges_seen_ - e0, "the relabelled pulse is not counted a second time");
  }

  // 2026-12-31 23:59:59 UTC = 1798761599
  begin("L. Inserted leap second, announced in advance (NAV-TIMELS, u-blox 8+)");
  {
    Sim s; s.answers_timels = true; s.leap_k = 100; s.t0 = 1798761599 - 99; s.setup();
    s.run(30); int unsynced_before = s.unsynced, silent_before = s.silent;
    s.run(60); int li1_before = s.li_seen[1];
    s.run(15); int li1_after_leap = s.li_seen[1];  // the leap is pulse 100, ~101 s in
    s.run(75);
    printf("    worst served error %.1f us; LI=1 replies before the leap: %d, after it: %d\n", s.worst_served_error_us,
           li1_before, s.li_seen[1] - li1_after_leap);
    check(li1_before > 0, "announces the leap (LI=1) beforehand");
    check(std::fabs(s.worst_served_error_us) < 50, "served time right through 23:59:60 and after");
    check(s.li_seen[1] == li1_after_leap, "stops announcing once the leap has happened");
    check(s.unsynced == unsynced_before && s.silent == silent_before, "no reset: stratum 1 throughout");
  }

  begin("L2. Leap second, then an 11-minute PPS gap: the held model keeps its time scale while relocking");
  {
    Sim s; s.answers_timels = true; s.leap_k = 100; s.t0 = 1798761599 - 99; s.setup();
    s.run(150);
    int unsynced_before = s.unsynced, silent_before = s.silent;
    s.pps_on = false; s.fix = false; s.run(11 * 60, false);
    s.fix = true; s.run(1.5, false);
    s.pps_on = true;
    for (int i = 0; i < 300 && !s.relocking_; i++) s.run(0.01);
    check(s.relocking_, "relocking on the held model");
    // Anything that republishes the model while it relocks (a NAV-TIMEUTC or NAV-TIMELS answer, a leap
    // being folded in) must not reinterpret the held model's anchor
    s.publish_model_();
    s.worst_served_error_us = 0; s.run(10);
    printf("    worst served error %.1f us\n", s.worst_served_error_us);
    check(std::fabs(s.worst_served_error_us) < 200, "served time keeps its scale through the relock");
    check(s.unsynced == unsynced_before && s.silent == silent_before, "stratum 1 throughout");
  }

  begin("L3. NAV-TIMELS is polled as soon as UTC is trusted, not a minute after boot");
  {
    Sim s; s.answers_timels = true; s.setup(); s.run(15);
    check(s.count_tx(0x01, 0x26) >= 1, "leap-second schedule requested within 15 s");
  }

  begin("M. Inserted leap second with no advance notice (NMEA 23:59:60 only, u-blox 6/7)");
  {
    Sim s; s.leap_k = 60; s.t0 = 1798761599 - 59; s.setup();
    s.run(55); int silent_before = s.silent, unsynced_before = s.unsynced;
    s.run(10, false);  // the leap itself: up to ~0.1 s of being a second fast until the RMC arrives
    s.worst_served_error_us = 0; s.run(120);
    check(s.silent == silent_before && s.unsynced == unsynced_before, "no discipline reset: stratum 1 throughout");
    check(std::fabs(s.worst_served_error_us) < 50, "correct from the first second after the leap");
  }

  begin("N. Signal strength (mean C/N0 of tracked satellites)");
  {
    Sim s; s.setup(); s.run(20); s.update();
    printf("    reported %.2f dB-Hz (expected 34.67)\n", s.cno.state);
    check(std::fabs(s.cno.state - 208.0 / 6.0) < 0.01, "mean over tracked satellites, ignoring untracked");
    check(s.strong.state == 3, "3 satellites at or above the 35 dB-Hz threshold (45, 38, 41)");
    check(std::fabs(s.hdop.state - 0.9) < 0.001, "HDOP from GGA");
    check(s.pulse_age.state < 2, "pulse age is seconds, not minutes");
    s.tracking = false; s.run(5); s.update();
    check(s.cno.state == 0.0f, "reads 0 when nothing is tracked");
    check(s.strong.state == 0, "no strong satellites either");
    s.tracking = true; s.run(5); s.update();
    check(std::fabs(s.cno.state - 208.0 / 6.0) < 0.01, "recovers");
  }

  begin("O. Receiver configuration (stationary model, NMEA trim, constellation report)");
  {
    Sim a; a.setup(); a.run(20);
    check(a.count_tx(0x06, 0x24) == 1, "CFG-NAV5 stationary sent exactly once");
    check(a.count_tx(0x06, 0x3E) == 1, "CFG-GNSS polled once");
    check(a.count_tx(0x06, 0x01) == 0, "no NMEA trim by default");
    a.run(120);
    check(a.count_tx(0x06, 0x24) == 1, "still only once after two minutes");
    check(a.served > 0, "still serving");

    Sim b; b.set_trim_nmea(true); b.setup(); b.run(20);
    check(b.count_tx(0x06, 0x01) == 11, "trim_nmea sent 11 CFG-MSG frames (3 kept, 8 dropped)");

    Sim c; c.set_stationary(false); c.setup(); c.run(20);
    check(c.count_tx(0x06, 0x24) == 0, "stationary: false sends no CFG-NAV5");
  }

  begin("P. Driver-level receive stamps (frame parsing, lookup, fallback)");
  {
    // Ethernet + IPv4 + UDP + NTP client request from 198.51.100.7 port 50123 to 123, with real lengths
    auto put16 = [](uint8_t *p, uint32_t v) { p[0] = v >> 8; p[1] = v & 0xFF; };
    uint8_t f[14 + 20 + 8 + 48] = {};
    f[12] = 0x08; f[13] = 0x00;
    uint8_t *ip = f + 14; ip[0] = 0x45; ip[9] = 17; put16(ip + 2, 20 + 8 + 48);
    ip[12] = 198; ip[13] = 51; ip[14] = 100; ip[15] = 7;
    uint8_t *udp = ip + 20; put16(udp, 50123); put16(udp + 2, 123); put16(udp + 4, 8 + 48);
    uint8_t *ntp = udp + 8; ntp[0] = 0x23;
    for (int i = 0; i < 8; i++) ntp[40 + i] = 0xA0 + i;
    uint8_t key[RX_KEY_LEN];
    static const uint8_t MAPPED[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xFF, 0xFF, 198, 51, 100, 7};
    check(parse_ntp_request_frame(f, sizeof(f), 123, key) && key[0] == 0xA0 && key[7] == 0xA7 &&
              key[8] == (50123 >> 8) && key[9] == (50123 & 0xFF) && memcmp(key + 10, MAPPED, 16) == 0,
          "IPv4 request parsed, key = transmit ts + src port + src address (v4-mapped)");
    check(!parse_ntp_request_frame(f, sizeof(f), 124, key), "other destination port ignored");
    check(!parse_ntp_request_frame(f, sizeof(f) - 1, 123, key), "truncated frame ignored");
    uint8_t padded[sizeof(f) + 10] = {};
    memcpy(padded, f, sizeof(f));
    check(parse_ntp_request_frame(padded, sizeof(padded), 123, key), "Ethernet padding past the IP length ignored");
    put16(ip + 2, 20 + 8 + 40);
    check(!parse_ntp_request_frame(f, sizeof(f), 123, key), "IPv4 total length too short for a request ignored");
    put16(ip + 2, 20 + 8 + 48 + 1);
    check(!parse_ntp_request_frame(f, sizeof(f), 123, key), "IPv4 total length past the frame ignored");
    put16(ip + 2, 20 + 8 + 48); put16(udp + 4, 8);
    check(!parse_ntp_request_frame(f, sizeof(f), 123, key), "UDP length too short for a request ignored");
    put16(udp + 4, 8 + 48 + 1);
    check(!parse_ntp_request_frame(f, sizeof(f), 123, key), "UDP length past the IP payload ignored");
    put16(udp + 4, 8 + 48);
    ntp[0] = 0x24;
    check(!parse_ntp_request_frame(f, sizeof(f), 123, key), "server-mode packet ignored");
    ntp[0] = 0x23; ip[6] = 0x20;  // MF
    check(!parse_ntp_request_frame(f, sizeof(f), 123, key), "IPv4 fragment ignored");
    ip[6] = 0x40;  // DF is fine
    check(parse_ntp_request_frame(f, sizeof(f), 123, key), "DF set still parsed");

    uint8_t v[14 + 40 + 8 + 48] = {};
    v[12] = 0x86; v[13] = 0xDD; v[14] = 0x60; put16(v + 14 + 4, 8 + 48); v[14 + 6] = 17;
    v[14 + 8] = 0x20; v[14 + 9] = 0x01; v[14 + 23] = 0x42;  // source 2001::42
    uint8_t *vu = v + 54; put16(vu, 77); put16(vu + 2, 123); put16(vu + 4, 8 + 48); vu[8] = 0x23; vu[8 + 40] = 0x55;
    check(parse_ntp_request_frame(v, sizeof(v), 123, key) && key[0] == 0x55 && key[9] == 77 && key[10] == 0x20 &&
              key[11] == 0x01 && key[25] == 0x42,
          "IPv6 request parsed, with its source address");
    put16(v + 14 + 4, 8 + 40);
    check(!parse_ntp_request_frame(v, sizeof(v), 123, key), "IPv6 payload length too short for a request ignored");
    put16(v + 14 + 4, 8 + 48);
    v[14 + 6] = 0;  // hop-by-hop extension header
    check(!parse_ntp_request_frame(v, sizeof(v), 123, key), "IPv6 with extension headers ignored");

    uint8_t t[4 + sizeof(f)] = {};
    memcpy(t, f, 12); t[12] = 0x81; t[13] = 0x00; memcpy(t + 16, f + 12, sizeof(f) - 12);
    check(parse_ntp_request_frame(t, sizeof(t), 123, key), "802.1Q-tagged frame parsed");

    Sim s; s.setup(); s.run(30);
    check(s.served > 0, "synced");
    uint8_t req[48] = {0x23}, rep[48];
    for (int i = 0; i < 8; i++) req[40 + i] = 0x10 + i;
    uint8_t k[RX_KEY_LEN]; make_rx_key(req + 40, Sim::CLIENT, 0x1234, k);
    uint8_t other_client[16]; memcpy(other_client, Sim::CLIENT, 16); other_client[15] = 2;
    auto t2_us = [&](const uint8_t *r) {
      uint32_t sec = (r[32] << 24) | (r[33] << 16) | (r[34] << 8) | r[35];
      uint32_t frac = (r[36] << 24) | (r[37] << 16) | (r[38] << 8) | r[39];
      return (static_cast<long double>(sec) - 2208988800.0L) * 1e6L + frac * 1e6L / 4294967296.0L;
    };
    auto off_stack = [&](long double stack) { return static_cast<double>(stack - t2_us(rep)); };
    s.build_reply_(req, g_now_us, Sim::CLIENT, 0x1234, rep);
    long double stack_t2 = t2_us(rep);
    s.rx_hook_active_ = true;
    s.rx_record_(k, g_now_us - 700);
    s.build_reply_(req, g_now_us, Sim::CLIENT, 0x1234, rep);
    printf("    stack T2 - hook T2 = %.1f us\n", off_stack(stack_t2));
    check(std::fabs(off_stack(stack_t2) - 700) < 2, "T2 taken from the driver stamp");
    check(s.rx_hook_hits_ == 1 && s.rx_gain_count_ == 1 && s.rx_gain_sum_us_ == 700, "gain recorded");
    s.build_reply_(req, g_now_us, Sim::CLIENT, 0x1234, rep);
    check(std::fabs(off_stack(stack_t2)) < 2 && s.rx_hook_misses_ == 1, "a stamp serves one request: a repeat falls back");
    s.rx_record_(k, g_now_us - 700);
    s.set_driver_rx_timestamp(false);
    s.build_reply_(req, g_now_us, Sim::CLIENT, 0x1234, rep);
    check(std::fabs(off_stack(stack_t2)) < 2, "switch off: stack stamp used, gain still measured");
    check(s.rx_gain_count_ == 2, "second gain sample");
    s.set_driver_rx_timestamp(true);
    s.rx_record_(k, g_now_us - 700);
    s.build_reply_(req, g_now_us, Sim::CLIENT, 0x1235, rep);
    check(std::fabs(off_stack(stack_t2)) < 2 && s.rx_hook_misses_ == 2,
          "different source port: no match, falls back to the stack stamp");
    s.build_reply_(req, g_now_us, other_client, 0x1234, rep);
    check(std::fabs(off_stack(stack_t2)) < 2 && s.rx_hook_misses_ == 3,
          "another client with the same transmit timestamp and port: no match");
    s.build_reply_(req, g_now_us, Sim::CLIENT, 0x1234, rep);
    check(std::fabs(off_stack(stack_t2) - 700) < 2, "the right client still finds its stamp");
    s.rx_record_(k, g_now_us - 900); s.rx_record_(k, g_now_us - 500);  // a retransmitted request
    s.build_reply_(req, g_now_us, Sim::CLIENT, 0x1234, rep);
    double first = off_stack(stack_t2);
    s.build_reply_(req, g_now_us, Sim::CLIENT, 0x1234, rep);
    check(std::fabs(first - 900) < 2 && std::fabs(off_stack(stack_t2) - 500) < 2,
          "two copies of one request take one stamp each, oldest first");
    s.rx_record_(k, g_now_us);
    s.run(1);  // the stamp is now a second old
    uint32_t hits = s.rx_hook_hits_;
    s.build_reply_(req, g_now_us, Sim::CLIENT, 0x1234, rep);
    check(s.rx_hook_hits_ == hits, "stale stamp not used");
    for (int i = 0; i < 20; i++) { uint8_t other[RX_KEY_LEN] = {static_cast<uint8_t>(i)}; s.rx_record_(other, g_now_us); }
    check(s.rx_ring_[0].seq.load() % 2 == 0, "ring slots settle on even sequence numbers");

    check(static_cast<int8_t>(rep[3]) == -20, "precision measured from the clock (1 us step = 2^-20 s)");
    uint32_t disp = (rep[8] << 24) | (rep[9] << 16) | (rep[10] << 8) | rep[11];
    check(disp >= 16 && disp <= 17, "root dispersion defaults to 250 us (+ drift since the last pulse)");
    Sim d; d.set_root_dispersion_us(1000); d.setup(); d.run(30);
    d.build_reply_(req, g_now_us, Sim::CLIENT, 0, rep);
    disp = (rep[8] << 24) | (rep[9] << 16) | (rep[10] << 8) | rep[11];
    check(disp >= 65 && disp <= 66, "root dispersion configurable");
  }

  begin("Q. ARP priming client table");
  {
    Sim s; s.setup();
    for (uint32_t i = 1; i <= 8; i++) { s.note_client_ipv4_(i); g_now_us += 1000000; s.run(0); }
    s.note_client_ipv4_(3);  // refresh an existing client
    int n = 0; for (auto &c : s.arp_clients_) n += c.load() != 0;
    check(n == 8, "eight clients held");
    s.run(1);
    s.note_client_ipv4_(99);
    bool has99 = false, has1 = false, has3 = false;
    for (auto &c : s.arp_clients_) { has99 |= c == 99; has1 |= c == 1; has3 |= c == 3; }
    check(has99 && !has1 && has3, "a ninth client replaces the least recently heard, not a recent one");
    s.note_client_ipv4_(0);
    n = 0; for (auto &c : s.arp_clients_) n += c.load() != 0;
    check(n == 8, "address 0 ignored");
  }

  begin("R. Position from GGA, averaged over the update interval");
  {
    check(std::fabs(nmea_coordinate("4124.8963", "N") - 41.414938) < 1e-6, "ddmm.mmmm latitude");
    check(std::fabs(nmea_coordinate("08151.6838", "W") + 81.861397) < 1e-6, "dddmm.mmmm longitude, west negative");
    check(std::fabs(nmea_coordinate("3352.1000", "S") + 33.868333) < 1e-6, "south negative");
    check(nmea_coordinate("00000.0000", "E") == 0.0, "zero is a real coordinate");
    check(std::isnan(nmea_coordinate("", "N")), "empty field (no fix) is not a coordinate");
    check(std::isnan(nmea_coordinate("4160.0000", "N")), "60 minutes is malformed");
    check(std::isnan(nmea_coordinate("9100.0000", "N")), "latitude beyond 90 degrees rejected");
    check(std::isnan(nmea_coordinate("4124.8963", "X")), "unknown hemisphere rejected");
    check(std::isnan(nmea_coordinate("41x4.8963", "N")), "garbage rejected");

    Sim s; s.setup(); s.run(20); s.update();
    printf("    %.6f, %.6f, %.1f m\n", s.latitude.state, s.longitude.state, s.altitude.state);
    check(std::fabs(s.latitude.state - 41.414938) < 1e-5, "latitude published");
    check(std::fabs(s.longitude.state + 81.861397) < 1e-5, "longitude published");
    check(std::fabs(s.altitude.state - 250.0) < 0.06, "altitude is the interval mean (249/251 alternating)");

    int before = s.latitude.publishes;
    s.fix = false; s.run(10); s.update();
    check(s.latitude.publishes == before, "an interval without a fix publishes no position");
    s.fix = true; s.run(10); s.update();
    check(s.latitude.publishes == before + 1, "published again once the fix returns");
  }

  begin("S. rx/tx delay compensation, max_residual default, clock scale");
  {
    auto ts_us = [](const uint8_t *r) {
      uint32_t sec = (r[0] << 24) | (r[1] << 16) | (r[2] << 8) | r[3];
      uint32_t frac = (r[4] << 24) | (r[5] << 16) | (r[6] << 8) | r[7];
      return (static_cast<long double>(sec) - 2208988800.0L) * 1e6L + frac * 1e6L / 4294967296.0L;
    };
    Sim s; s.setup(); s.run(30);
    check(s.served > 0, "synced");
    uint8_t req[48] = {0x23}, rep[48];
    s.build_reply_(req, g_now_us, Sim::CLIENT, 0, rep);
    long double t2 = ts_us(rep + 32), t3 = ts_us(rep + 40);
    s.set_rx_delay_us(30); s.set_tx_delay_us(20);
    s.build_reply_(req, g_now_us, Sim::CLIENT, 0, rep);
    printf("    T2 moved %.2f us, T3 moved %.2f us\n", static_cast<double>(ts_us(rep + 32) - t2),
           static_cast<double>(ts_us(rep + 40) - t3));
    check(std::fabs(static_cast<double>(ts_us(rep + 32) - t2) + 30) < 0.01, "rx_delay taken off T2");
    check(std::fabs(static_cast<double>(ts_us(rep + 40) - t3) - 20) < 0.01, "tx_delay added to T3");
    s.rx_hook_active_ = true;
    uint8_t k[RX_KEY_LEN]; make_rx_key(req + 40, Sim::CLIENT, 0x1234, k);
    s.rx_record_(k, g_now_us - 700);
    s.build_reply_(req, g_now_us, Sim::CLIENT, 0x1234, rep);
    check(std::fabs(static_cast<double>(ts_us(rep + 32) - t2) + 730) < 0.01, "rx_delay also taken off a driver stamp");

    check(s.max_residual_us_ == 1000.0, "GPIO interrupt capture: max_residual defaults to 1000 us");
    Sim e; e.set_max_residual_us(50); e.setup();
    check(e.max_residual_us_ == 50.0, "an explicit max_residual is kept");

    // esp_timer_get_time() truncates the counter: over every sub-microsecond phase it averages the
    // counter's microseconds less 15/32 (at 16 ticks/µs). Edges placed by counter_to_stamp_us must agree.
    double floor_sum = 0, placed_sum = 0;
    for (uint64_t c = 16 * 1000003; c < 16 * 1000003 + 16; c++) {
      floor_sum += static_cast<double>(c / 16);
      placed_sum += counter_to_stamp_us(static_cast<double>(c), 16.0);
    }
    check(std::fabs(placed_sum - floor_sum) < 1e-6, "edges sit on esp_timer_get_time()'s truncated scale");
    check(counter_to_stamp_us(1234567.0, 1.0) == 1234567.0, "a 1 tick/us counter needs no correction");
  }

  begin("T. Strict NMEA parsing: checksum and RMC time/date fields");
  {
    Sim s;
    auto line = [](const std::string &body) { std::string l = Sim::nmea(body); return l.substr(0, l.size() - 2); };
    auto feed = [&](const std::string &l) { char buf[96]; snprintf(buf, sizeof(buf), "%s", l.c_str()); s.handle_nmea_(buf); };
    std::string good = line("GPGSV,1,1,01,02,23,297,45");
    uint32_t ok0 = s.nmea_ok_, bad0 = s.nmea_bad_;
    feed(good);
    check(s.nmea_ok_ == ok0 + 1, "well-formed sentence accepted");
    feed(good + "junk");
    check(s.nmea_bad_ == bad0 + 1, "text after the checksum rejected");
    std::string three = good; three.insert(three.size() - 2, "0");
    feed(three);
    check(s.nmea_bad_ == bad0 + 2, "three-digit checksum rejected");
    std::string one = good.substr(0, good.size() - 1);
    feed(one);
    check(s.nmea_bad_ == bad0 + 3 && s.nmea_ok_ == ok0 + 1, "one-digit checksum rejected");

    auto rmc = [&](const char *time, const char *date) {
      char body[128];
      snprintf(body, sizeof(body), "GPRMC,%s,A,4124.8963,N,08151.6838,W,0.0,0.0,%s,,,A", time, date);
      s.last_rmc_valid_us_ = 0;
      feed(line(body));
      return s.last_rmc_valid_us_ != 0;
    };
    check(rmc("120000.00", "190926"), "well-formed RMC accepted");
    check(rmc("120000", "190926"), "time without a fraction accepted");
    check(!rmc("120000junk", "190926"), "junk after the time rejected");
    check(!rmc("120000.0x", "190926"), "junk in the fraction rejected");
    check(!rmc("120000.", "190926"), "empty fraction rejected");
    check(!rmc("120000.50", "190926"), "a fractional epoch is not a pulse epoch");
    check(!rmc("120000.00", "290226"), "29 February 2026 rejected, not normalised to 1 March");
    check(rmc("120000.00", "290228"), "29 February 2028 accepted");
    check(!rmc("120000.00", "310426"), "31 April rejected");
    check(!rmc("120000.00", "19092"), "short date rejected");
  }

  begin("U. A model that becomes trusted outside a pulse (publish_model_) marks the server as synchronised");
  {
    Sim s; s.utc_valid = false; s.setup(); s.run(30, false);
    check(s.model_.valid && !s.model_.utc_trusted && !s.ever_synced_, "fit ready, receiver UTC not yet confirmed");
    // The receiver confirms UTC between pulses, with the labels already confirmed under trusted UTC (for
    // example it went invalid and valid again): the republish is what makes the model trusted...
    s.utc_trusted_ = true; s.trusted_confirmations_ = 3;
    s.publish_model_();
    check(s.model_.utc_trusted, "published as trusted");
    // ...and a lost lock before the next pulse must answer stratum 16, not fall silent
    s.reset_discipline_("test", false);
    int silent_before = s.silent; s.query(false);
    check(s.silent == silent_before && s.unsynced > 0, "a reset right after answers stratum 16, not silence");
  }

  begin("Z. Counters across every scenario above");
  check(!g_counter_underflow, "never accepted more pulses than edges seen (rejected_pulses can't underflow)");

  printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL PASSED", failures, failures == 1 ? "" : "s");
  return failures ? 1 : 0;
}
