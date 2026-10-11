# Changelog

Hardware status: run on an ESP32-S3-ETH against the `gnss_sim` emulator. Not yet run against a real receiver.

## Unreleased
- Fix: with MCPWM hardware capture, served time ran about 0.47 µs early. Since v0.7.0 PPS edges were placed on esp_timer's exact counter, but every NTP receive and transmit stamp is a whole-µs `esp_timer_get_time()`, which truncates. The edges are now placed on that same truncated scale.
- The raw esp_timer counter is private ESP-IDF API. It is now declared weak and checked at boot against `esp_timer_get_time()` (an integer ratio, no offset). If it's missing or doesn't match, the reference latch falls back to whole-microsecond reads with a warning, instead of failing to link or placing edges on a shifted timeline.
- `max_residual` defaults to 200 µs only with MCPWM hardware capture. The GPIO interrupt, including the IDF < 6 fallback added in v0.8.1, gets 1000 µs again, because its latency can exceed 200 µs for several pulses running while flash is busy, which dropped the server to stratum 16. An explicit `max_residual` is used as given.
- The "has ever synchronised" flag the reply path reads from the NTP task is now atomic.
- Host tests for `rx_delay` / `tx_delay`, the `max_residual` default and the counter-to-stamp scale.
- Fix: a fit labelled from NMEA while the receiver's UTC was not yet valid was served at stratum 1 the moment NAV-TIMEUTC confirmed UTC, without checking its labels. On a cold start with NMEA whole seconds off, that served time two seconds wrong for about a second, until three RMC disagreements reset it. The labels now need 3 RMC confirmations sent after UTC was confirmed; if those disagree, the fit is relabelled first.
- Fix: an RMC with status `V` now stops pulses entering the fit at once. Before, up to 3 more free-running pulses were used until the last good fix went stale.
- Fix: after a leap second, a PPS gap long enough to relabel (over 600 s) reset the leap adjustment while the held-over model still used it, so any republish during the relock (a NAV-TIMEUTC or NAV-TIMELS answer) served time a second off. The adjustment is now kept.
- Fix: an edge is accepted as on the second within `max_residual` of it (plus 100 ppm of the time since the last pulse), capped at 100 ms. The fixed 300 µs gate reset the discipline on three GPIO edges 400-1000 µs late, inside the 1000 µs interrupt default.
- Fix: a model that becomes trusted between pulses now marks the server as having synchronised, so a lost lock right after answers stratum 16 rather than nothing.
- Fix: with MCPWM capture, a pulse relabelled after an RMC disagreement was counted as accepted twice (sub-µs time against its rounded copy), which could underflow `rejected_pulses`. The candidate pulse keeps its exact time, which also keeps its sub-µs precision.
- NMEA parsing is stricter: the checksum must be exactly two hex digits at the end of the sentence, RMC times must be `hhmmss` or `hhmmss.` followed by zeros, and dates must exist (29 February 2026 was read as 1 March).
- Driver-level receive stamps: the key now includes the client's address as well as its transmit timestamp and port, each stamp answers one request, and the ring's payload is read and written as atomics inside its seqlock (it was formally a data race). Frames are checked against their declared IP and UDP lengths.
- `UBX-NAV-TIMELS` is polled as soon as UTC is trusted, rather than first at a minute after boot.
- Validation rejects `enable_lwip_tcpip_core_locking: false` (and `CONFIG_LWIP_TCPIP_CORE_LOCKING: n`): the ARP refresh and `raw_lwip` call lwIP from the main loop and need that lock.
- README and the example now pin v0.8.1 (v0.7.0 doesn't build on IDF 5.5).
- `tools/ntpprobe.py` only counts a reply from the server it asked, of full length, echoing its transmit timestamp, and handles NTP timestamps past the 2036 rollover.
- Tests: new scenarios for each fix above. The simulated receiver now keeps sending NMEA without PPS; scenario I really free-runs the pulses; J and J2 send RMCs before PPS returns, so the long-gap relock path runs; L and M fail on any drop to stratum 16. `invalid-rx-reference-idf55.yaml` pins ESP-IDF 5.5.5 instead of relying on ESPHome's default.

## v0.8.1 - 2026-09-29
- MCPWM hardware capture now needs ESP-IDF 6.0 or newer, which is where `mcpwm_capture_get_latched_value()` first appears. On an older framework the component falls back to the GPIO interrupt with a compile-time warning instead of failing to build, and `rx_reference_pin` is rejected with a clear message. Builds on IDF 6.x are unchanged. Every release since v0.7.0 failed to compile on IDF 5.5.
- `gnss_sim` builds on IDF 5.5 again (the UART signal table moved between 5.5 and 6.0).
- CI: the compile step now fails when the build fails. It piped through `tee` without `pipefail`, so failed builds showed green. That is how the two problems above went unnoticed.

## v0.8.0 - 2026-09-29
- New `latitude`, `longitude` and `altitude` sensors: the receiver's position from the GGA sentences the component already reads, published as the mean of each update interval's fixes. Fixes with GGA quality 0 are ignored, since receivers keep repeating their last position after losing the fix. No extra serial traffic, and nothing changes when the sensors aren't configured.

## v0.7.0 - 2026-09-23
- A lost lock no longer silences the server. A PPS gap keeps serving stratum 1 on the held-over model while the fit rebuilds (it was only ever as stale as its holdover); a phase step, rate error or NMEA disagreement answers with stratum 16 (LI=3) until the new fit is ready, so clients see a reply instead of a timeout. Silence is now only for a node that has never synchronised.
- `rx_delay` / `tx_delay`: measured fixed delays in the node's network path, taken off T2 and added to T3. Default 0.
- The reference latch reads esp_timer's raw 16 MHz counter instead of whole microseconds, placing each PPS edge to ~60 ns instead of ±0.5 µs. Pulse times and the fit are sub-microsecond throughout; NTP timestamps are still whole µs.
- Precision is reported as 2^-20 (the nearest power of two at or below the 1 µs clock step) rather than 2^-19.
- `max_residual` default 200 µs (was 1000): with hardware capture jitter under 1 µs, a bad pulse is caught five times sooner.

## v0.6.1 - 2026-09-21
- Fix: `rejected_pulses` could underflow to 4294967295 (shown as 4294967296). When three RMCs disagreed with the pulse count, the fit reset and the same pulse was re-labelled and counted as accepted twice. Each edge is now counted at most once. Served time was never affected. Seen on hardware (weak-signal cold start); a sim check now runs across every scenario.

## v0.6.0 - 2026-09-21
Ideas from [tiehfood/esphome-gps-pps-ntp-server](https://github.com/tiehfood/esphome-gps-pps-ntp-server). Not yet run on hardware.
- **Driver-level receive timestamps** (`driver_rx_timestamp`, default on, Ethernet only): NTP requests are stamped in the Ethernet driver's input path before lwIP, and replies use that stamp as T2. Switchable at runtime; new `rx_timestamp_gain` sensor reports what it removes.
- **ARP priming:** recent IPv4 clients (or the gateway, for off-subnet clients) are re-ARPed every 2 minutes so a slow-polling client's reply never waits on ARP. New `arp_clients` sensor.
- `root_dispersion` (default 250 µs) replaces the fixed 20 µs, which claimed far more than an SPI Ethernet path can deliver.
- Precision is measured from the clock at boot (normally 2^-19 s) instead of being assumed from the capture mode.
- `rx_reference_pin` (diagnostic): MCPWM-capture the SPI Ethernet chip's interrupt line; new `rx_interrupt_lead` sensor.
- Status line gains `rx_hook=hits/misses` and `arp`.

## v0.5.2 - 2026-09-20
- Repeat the constellation report in `dump_config()`. ESPHome replays the config dump to every log client that connects, so a one-shot INFO line at boot was in practice invisible.

## v0.5.1 - 2026-09-20
- Fix: the "receiver doesn't speak UBX" fallback keyed off *any* UBX frame, so a receiver that answered the new `CFG-GNSS` poll but not `NAV-TIMEUTC` would never be trusted and never serve. It now keys off `NAV-TIMEUTC` specifically.

## v0.5.0 - 2026-09-20
- `stationary` (default on): put the receiver in its stationary dynamic model at boot.
- `trim_nmea` (default off): silence the NMEA sentences the component doesn't read.
- Poll `UBX-CFG-GNSS` once and report which constellations the receiver has and which are enabled.
- Log a warning when the receiver rejects one of our commands (UBX-ACK-NAK).

## v0.4.0 - 2026-09-20
- New sensors: `strong_satellites` (with `strong_signal_threshold`), `hdop`, `rejected_pulses`, `nmea_errors` and `pulse_age`.
- The status line reports `strong` and `hdop`, and `stack_free` prints `n/a` on the raw lwIP transport instead of 0.
- First run against a real receiver (u-blox on an ESP32-S3-ETH): 420/420 pulses accepted, 0 NMEA checksum errors, PPS jitter 0.5-0.8 us, crystal offset steady to 0.15 ppm.

## v0.3.3 - 2026-09-19
- New `signal_strength` sensor: mean C/N0 of the tracked satellites, parsed from GSV. Intended for comparing antenna positions.
- CI: host simulation, config validation (including a config that must be rejected), and compiles for ESP32-S3 (ESP-IDF default and 6.1.0), S3 with raw lwIP + IPv6 + `gnss_sim`, ESP32-C3 and ESP32-P4.

## v0.3.2 - 2026-09-19
- IPv6 on both transports (needs `network: enable_ipv6: true`). The socket transport uses one dual-stack socket.
- `tools/ntpprobe.py` accepts IPv6 addresses.

## v0.3.1 - 2026-09-19
- `gnss_sim` falls back to a fixed fake date when the node has no time source.
- `tools/ntpprobe.py`; measured round-trip times for both transports documented in the README.

## v0.3.0 - 2026-09-19
- Leap seconds: schedule from `UBX-NAV-TIMELS` with LI announced to clients; fallback to the receiver's `23:59:60` sentence on u-blox 6/7. The fit no longer resets across a leap.
- `gnss_sim` can inject a leap second, with or without advance notice.

## v0.2.3 - 2026-09-19
- `gnss_sim`: test-only u-blox emulator (PPS, NMEA, UBX) with fault injection. Runs on a second board, or on the same chip looped back through the GPIO matrix.

## v0.2.2 - 2026-09-19
- A stray PPS edge no longer resets the fit; three misaligned edges in a row re-label.
- The server doesn't answer until 3 RMC sentences have agreed with the pulse count.
- RMC fields are range-checked; dates before 2026 are treated as a GPS week rollover.
- Hardware capture honours the pin's pull mode and `inverted`.
- Status line every `update_interval`; WARN when PPS or NMEA go quiet.
- The target baud is probed before a switch is attempted, so the receiver's config isn't re-saved on every boot.
- New: `require_utc_valid`, experimental `transport: raw_lwip`.
- `tests/`: host-side simulation of the state machine.

## v0.2.1 - 2026-09-19
- New options: `fit_window`, `max_residual`, `refid`, `task_core` (rejected on single-core chips).

## v0.2.0 - 2026-09-19
- PPS timestamps taken by the MCPWM capture unit where the chip has one.
- NTP task pinned to the core the ESPHome loop isn't using.

## v0.1.1 - 2026-09-19
- Fix a boot loop: the NTP socket was opened before the network stack was up.

## v0.1.0 - 2026-09-19
- First version.
