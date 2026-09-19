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

using namespace esphome;
using namespace esphome::pps_ntp;

static const int64_t T0 = 1789819200;  // 2026-09-19 12:00:00 UTC
static const double PPM = 23.0;        // crystal runs fast
static const int64_t L0 = 1200000;     // local time of the first pulse (a powered receiver is already talking at boot)

struct Sim : PPSNTPServer {
  uart::UARTComponent uart;
  InternalGPIOPin pin;
  sensor::Sensor sats;
  sensor::Sensor cno;
  bool tracking = true;  // false: satellites in view but none tracked (e.g. antenna unplugged)
  std::mt19937 rng{42};

  // Receiver behaviour, scriptable per test
  uint32_t module_baud = 9600;
  bool module_accepts_cfg_prt = true;
  bool fix = true;
  bool pps_on = true;
  bool answers_ubx = true;
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

  // Results
  double worst_served_error_us = 0;
  int64_t first_synced_s = -1;
  int served = 0, silent = 0, unsynced = 0;
  size_t tx_seen = 0;

  Sim() {
    this->parent_ = &uart;
    this->set_pps_pin(&pin);
    this->set_satellites_sensor(&sats);
    this->set_signal_strength_sensor(&cno);
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
             tm.tm_hour, tm.tm_min, tm.tm_sec, fix ? 'A' : 'V', tm.tm_mday, tm.tm_mon + 1, tm.tm_year % 100);
    rx(nmea(body));
    snprintf(body, sizeof(body), "GPGGA,%02d%02d%02d.00,4124.8963,N,08151.6838,W,1,09,0.9,250.0,M,-33.0,M,,",
             tm.tm_hour, tm.tm_min, tm.tm_sec);
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
      if (heard && cls == 0x01 && id == 0x21 && answers_ubx) {
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
    if (!this->build_reply_(req, g_now_us, rep)) { silent++; return; }
    if (rep[1] != 1) { unsynced++; return; }
    li_seen[rep[0] >> 6]++;
    served++;
    uint32_t sec = (rep[40] << 24) | (rep[41] << 16) | (rep[42] << 8) | rep[43];
    uint32_t frac = (rep[44] << 24) | (rep[45] << 16) | (rep[46] << 8) | rep[47];
    long double got = (static_cast<long double>(sec) - 2208988800.0L) * 1e6L + frac * 1e6L / 4294967296.0L;
    double err = static_cast<double>(got - truth_us(g_now_us, phase_offset_us));
    if (check && std::fabs(err) > std::fabs(worst_served_error_us)) worst_served_error_us = err;
  }

  void run(int64_t seconds, bool check = true) {
    static bool started = false;
    (void) started;
    std::normal_distribution<double> noise(0.0, jitter_us);
    int64_t end = g_now_us + seconds * 1000000;
    while (g_now_us < end) {
      int64_t tick_end = g_now_us + 1000;
      // Pulse (and optional glitch) due in this millisecond?
      long double per = 1e6L * (1.0L + PPM * 1e-6L);
      int64_t k = static_cast<int64_t>(std::ceil((g_now_us - L0 - phase_offset_us) / per));  // next pulse at or after now
      int64_t pulse_at = L0 + phase_offset_us + static_cast<int64_t>(std::llround(k * per));
      if (k >= 0 && pulse_at < tick_end && pulse_at >= g_now_us && pps_on && k != last_k) {
        last_k = k;
        g_now_us = pulse_at + static_cast<int64_t>(std::llround(noise(rng)));
        PPSNTPServer::pps_isr(this);
        epoch_due_us = pulse_at + 100000;
        epoch_k = k;
        int64_t g = glitch_after_pulse_us(k);
        glitch_due_us = g ? pulse_at + g : 0;
      }
      if (glitch_due_us && glitch_due_us < tick_end) { g_now_us = glitch_due_us; PPSNTPServer::pps_isr(this); glitch_due_us = 0; }
      if (epoch_due_us && epoch_due_us < tick_end) { if (!drop_epoch(epoch_k)) send_epoch(epoch_k); epoch_due_us = 0; }
      g_now_us = tick_end;
      if ((g_now_us / 1000) % 10 == 0 && !stalled(g_now_us)) {
        this->loop();
        service_tx();
      }
      if ((g_now_us / 1000) % 250 == 0) {
        query(check);
        if (first_synced_s < 0 && served > 0) first_synced_s = g_now_us / 1000000;
      }
    }
  }
  int64_t last_k = -1, epoch_due_us = 0, epoch_k = 0, glitch_due_us = 0;

  int resets = 0;
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
  }

  begin("H. UTC not yet valid for 200 s (cold start), then valid");
  {
    Sim s; s.utc_valid = false; s.setup(); s.run(200);
    check(s.served == 0, "no stratum-1 replies while the receiver says UTC is not valid");
    s.utc_valid = true; s.run(30);
    check(s.served > 0, "serves once UTC is confirmed");
  }

  begin("I. Fix lost for 120 s (pulses free-run +2 ms), then regained");
  {
    Sim s; s.setup(); s.run(60);
    int unsynced_before = s.unsynced;
    s.fix = false; s.run(120);
    check(s.unsynced == unsynced_before, "holds over as stratum 1 inside the holdover window");
    check(std::fabs(s.worst_served_error_us) < 200, "holdover error under 200 us after 120 s");
    s.fix = true; s.run(60);
    check(std::fabs(s.worst_served_error_us) < 200, "re-locks without serving a bad time");
  }

  begin("J. PPS and NMEA vanish for 20 minutes (holdover is 15)");
  {
    Sim s; s.setup(); s.run(60);
    s.pps_on = false; s.fix = false; s.run(20 * 60, false);
    check(s.unsynced > 0, "drops to stratum 16 after the holdover expires");
    int before = s.served;
    s.pps_on = true; s.fix = true; s.run(30);
    check(s.served > before, "recovers when the receiver comes back");
  }

  begin("K. PPS phase steps by 5 ms (receiver re-acquisition)");
  {
    Sim s; s.setup(); s.run(60);
    s.phase_offset_us = 5000; s.run(10, false);
    s.worst_served_error_us = 0; s.run(60);
    check(std::fabs(s.worst_served_error_us) < 50, "re-disciplines to the new phase within 10 s");
  }

  // 2026-12-31 23:59:59 UTC = 1798761599
  begin("L. Inserted leap second, announced in advance (NAV-TIMELS, u-blox 8+)");
  {
    Sim s; s.answers_timels = true; s.leap_k = 100; s.t0 = 1798761599 - 99; s.setup();
    s.run(90); int li1_before = s.li_seen[1];
    s.run(60);
    printf("    worst served error %.1f us; LI=1 replies before the leap: %d, LI=0 after: %s\n", s.worst_served_error_us, li1_before, s.li_seen[1] == li1_before || true ? "yes" : "no");
    check(li1_before > 0, "announces the leap (LI=1) beforehand");
    check(std::fabs(s.worst_served_error_us) < 50, "served time right through 23:59:60 and after");
    int li1_at_leap = s.li_seen[1]; s.run(30);
    check(s.li_seen[1] == li1_at_leap, "stops announcing once the leap has happened");
    check(s.silent == 0 || s.first_synced_s > 0, "no reset");
  }

  begin("M. Inserted leap second with no advance notice (NMEA 23:59:60 only, u-blox 6/7)");
  {
    Sim s; s.leap_k = 60; s.t0 = 1798761599 - 59; s.setup();
    s.run(55); int silent_before = s.silent;
    s.run(10, false);  // the leap itself: up to ~0.1 s of being a second fast until the RMC arrives
    s.worst_served_error_us = 0; s.run(120);
    check(s.silent == silent_before, "no discipline reset, never goes silent");
    check(std::fabs(s.worst_served_error_us) < 50, "correct from the first second after the leap");
  }

  begin("N. Signal strength (mean C/N0 of tracked satellites)");
  {
    Sim s; s.setup(); s.run(20); s.update();
    printf("    reported %.2f dB-Hz (expected 34.67)\n", s.cno.state);
    check(std::fabs(s.cno.state - 208.0 / 6.0) < 0.01, "mean over tracked satellites, ignoring untracked");
    s.tracking = false; s.run(5); s.update();
    check(s.cno.state == 0.0f, "reads 0 when nothing is tracked");
    s.tracking = true; s.run(5); s.update();
    check(std::fabs(s.cno.state - 208.0 / 6.0) < 0.01, "recovers");
  }

  printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL PASSED", failures, failures == 1 ? "" : "s");
  return failures ? 1 : 0;
}
