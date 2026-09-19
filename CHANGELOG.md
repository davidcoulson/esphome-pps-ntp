# Changelog

Hardware status: run on an ESP32-S3-ETH against the `gnss_sim` emulator. Not yet run against a real receiver.

## Unreleased

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
