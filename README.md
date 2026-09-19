# esphome-pps-ntp

[![CI](https://github.com/davidcoulson/esphome-pps-ntp/actions/workflows/ci.yml/badge.svg)](https://github.com/davidcoulson/esphome-pps-ntp/actions/workflows/ci.yml)

An [ESPHome](https://esphome.io) external component that turns an ESP32 with Ethernet and a GNSS receiver into a **stratum-1 NTP server** disciplined by the receiver's **PPS (pulse-per-second)** output.

> **Status: experimental.** Compiles against ESPHome 2026.9 on ESP-IDF 5.5 and 6.1 for ESP32-S3, ESP32-C3 and ESP32-P4 (including pre-v3 silicon via `engineering_sample: true`). Hardware testing is in progress.

## How it works

```
GNSS PPS ──► MCPWM capture latch ──► timestamp (80 MHz, mapped to esp_timer µs)
                                     │
GNSS UART ─► NMEA RMC (which second) ┤
          └► UBX NAV-TIMEUTC (UTC valid?)
                                     ▼
                        clock model: local µs ⇄ UTC
                     (least-squares fit over 64 pulses)
                                     │
                    NTP task (UDP :123) ◄── clients
```

1. **PPS capture.** On chips with an MCPWM unit (ESP32, S3, C5, C6, H2, P4), the capture hardware latches an 80 MHz timer at the moment the PPS edge arrives. No interrupt is involved, so interrupt latency and flash-write stalls (NVS, OTA) can't delay the timestamp. The ESPHome loop reads the latch, then software-latches the same timer between two `esp_timer` reads to map the edge onto the system timeline. Chips without MCPWM, or `hardware_capture: false`, fall back to an IRAM GPIO interrupt.
2. **Labelling.** The RMC sentence that follows each pulse says which UTC second it marked. After that, pulses are counted, and RMC is used to cross-check.
   - The server doesn't answer until 3 later RMCs have independently agreed with the count. That guards against a first label that is off by a whole second.
   - Stray edges (noise on the PPS line) are ignored. Three misaligned edges in a row, or three RMC disagreements, reset the fit.
   - Dates before 2026 are treated as a GPS week-number rollover (old and clone receivers) and moved forward by whole multiples of 1024 weeks.
3. **Discipline.** A least-squares fit over the last 64 pulses gives the crystal's rate and phase. The ESP32 system clock is left alone; NTP timestamps are computed straight from the fit.
4. **Leap-second safety.** The receiver is polled with `UBX-NAV-TIMEUTC`, and the server reports itself as unsynchronised until the receiver confirms UTC is valid. (After a cold start, u-blox receivers can report time with the wrong leap-second count for up to about 12.5 minutes.) Receivers that don't answer UBX fall back to NMEA-only after 60 s.
5. **Leap seconds.** The fit runs on a continuous count of seconds, so its history stays linear across a leap, and UTC is derived from it with an adjustment that changes by one at the leap.
   - **u-blox 8 and later:** the schedule comes from `UBX-NAV-TIMELS`. Clients get LI=1 (or 2) during the final day, and the step happens exactly at midnight. An inserted second is served as a repeat of 23:59:59, as NTP servers conventionally do.
   - **u-blox 6/7 (no NAV-TIMELS):** the leap is taken from the receiver's own `23:59:60` sentence. There is no advance LI, and for roughly 0.1 s (until that sentence arrives) replies are one second fast.
   - Either way the fit is not reset and the server doesn't go silent.
6. **Serving.** A dedicated FreeRTOS task, pinned to the core the ESPHome loop isn't using, answers NTPv3 and NTPv4 client requests. It timestamps each request as soon as it arrives and each reply just before sending.
   - **Synced:** stratum 1, refid `GPS` (configurable). The precision field is 2^-20 s (about 1 µs) with hardware capture and 2^-18 s with the GPIO interrupt.
   - **PPS lost:** keeps serving on the local crystal for `holdover` (stratum 1, with dispersion growing over time).
   - **After holdover:** replies with LI=3 and stratum 16.
   - **Never synced:** doesn't reply at all.

The ESPHome main loop only handles parsing and the fit. Nothing time-critical depends on how often it runs.

## Hardware

| Part | Notes |
|---|---|
| ESP32 with **Ethernet** | Tested target: Waveshare ESP32-S3-ETH (W5500, optional PoE). Any ESPHome-supported Ethernet board should work. Wi-Fi works but adds milliseconds of jitter, so use Ethernet. |
| GNSS receiver **with PPS** | u-blox (NEO-6M/7M/M8/M9/M10) or a u-blox clone. The PPS must be **3.3 V logic**. |
| Active GNSS antenna | Recommended for indoor installs. Put it at a window, in the attic, or outdoors, and watch the `signal_strength` sensor to compare spots. Its amplifier sits ahead of the cable, so extension loss costs gain rather than signal-to-noise: at 1575 MHz budget roughly 1.1 dB/m for RG174, 0.65 dB/m for RG58 and 0.4 dB/m for LMR-240, and keep 15 dB or more of gain at the module. |

Wiring (example config):

| GNSS | ESP32-S3-ETH |
|---|---|
| TX | GPIO17 (UART RX) |
| RX | GPIO16 (UART TX) |
| PPS | GPIO15 |
| VCC | 3V3 |
| GND | GND |

## Usage

```yaml
external_components:
  - source: github://davidcoulson/esphome-pps-ntp@v0.3.3
    components: [pps_ntp]

uart:
  id: gnss_uart
  rx_pin: GPIO17
  tx_pin: GPIO16
  baud_rate: 9600        # module's factory baud
  rx_buffer_size: 1024

pps_ntp:
  uart_id: gnss_uart
  pps_pin: GPIO15
  gnss_baud_rate: 115200 # optional
  holdover: 15min
  satellites:
    name: Satellites
  frequency_offset:
    name: Crystal Offset
  pps_jitter:
    name: PPS Jitter
  requests:
    name: NTP Requests
  synced:
    name: GPS Synced
```

For a full config, see [`examples/waveshare-esp32-s3-eth.yaml`](examples/waveshare-esp32-s3-eth.yaml).

### Configuration variables

| Key | Default | Description |
|---|---|---|
| `uart_id` | — | UART connected to the receiver. Needs both RX and TX, because the component sends UBX commands. |
| `pps_pin` | **required** | GPIO connected to the receiver's PPS output. The pin's `mode` (pull-up or pull-down) and `inverted: true` (active-low PPS) apply in both capture modes. |
| `port` | `123` | UDP port to listen on. |
| `hardware_capture` | `true` | Timestamp PPS with the MCPWM capture unit where the chip has one. Set to `false` to force the GPIO interrupt. |
| `gnss_baud_rate` | none | If set, the component first listens at this rate for 1.5 s (the receiver keeps its setting across ESP reboots). Only if that stays quiet does it send the legacy `UBX-CFG-PRT` command at the UART's configured rate, verify the switch, save it with `UBX-CFG-CFG`, and fall back to the original rate if the switch fails. Useful for NEO-6M/7M/M8 modules stuck at 9600. |
| `holdover` | `15min` | How long to keep serving stratum 1 after PPS or the fix is lost. |
| `fit_window` | `64` | Pulses in the least-squares fit (8–256). Longer windows average out more noise; shorter ones track temperature changes in the crystal faster. |
| `max_residual` | `1000us` | A pulse further than this from the model is an outlier; three in a row reset the fit. With hardware capture, about `50us` is a reasonable tighter setting once the node has proven stable. |
| `refid` | `GPS` | NTP reference ID sent to clients (1–4 ASCII characters, e.g. `PPS`). |
| `require_utc_valid` | `false` | If `true`, never claim stratum 1 until the receiver confirms UTC over UBX. By default a receiver that doesn't answer UBX is trusted after 60 s, which can be one leap-second count off for up to about 12.5 minutes after a cold start. |
| `transport` | `socket` | `raw_lwip` is **experimental**: it has served a few thousand requests on an ESP32-S3-ETH without trouble, and cut the median round trip by about 0.45 ms and the tail by more (see Limitations), but it hasn't had a long soak. It answers from lwIP's tcpip thread instead of a socket task, which removes the socket mailbox and a task wake-up from the timestamp path. `task_core` can't be combined with it. |
| `task_core` | auto | Pins the NTP task to core `0` or `1`. By default it runs on the core the ESPHome loop isn't using. Rejected at validation on single-core chips (C2/C3/C5/C6/C61/H2/S2) and on an ESP32 built with `CONFIG_FREERTOS_UNICORE`. A compile-time `#error` catches anything else that ends up single-core. |
| `update_interval` | `60s` | How often the sensors publish. |

### Sensors

| Key | Type | Meaning |
|---|---|---|
| `satellites` | sensor | Satellites used in the fix (from GGA). |
| `signal_strength` | sensor (dB-Hz) | Mean C/N0 across the satellites the receiver is tracking, from the GSV sentences of one epoch. Satellites in view but not tracked are excluded; 0 means nothing is being tracked at all. **This is the number to compare antenna positions with** — the satellite count saturates long before signal strength does. Roughly: below 30 is poor, 35–40 is a decent indoor install, above 42 is a clear sky view. |
| `frequency_offset` | sensor (ppm) | Measured error of the ESP32 crystal. |
| `pps_jitter` | sensor (µs) | RMS scatter of the pulses around the fitted model. |
| `requests` | sensor | NTP requests served since boot. |
| `synced` | binary sensor | On while serving stratum 1. |

## Diagnostics

Every `update_interval` the component logs a status line at DEBUG:

```
edges=61 accepted=60 nmea=122/0 bad ubx=9 utc_valid=YES fit=60/64 confirmed=57 residual=-0.8us jitter=1.2us sats=9 stack_free=2140
```

- `edges` / `accepted`: PPS edges seen, and pulses that went into the fit.
- `nmea=ok/bad`: sentences with a good and a bad checksum. `ubx`: UBX frames received.
- `confirmed`: RMC sentences that agreed with the pulse count since the last reset (3 are needed to serve).
- `stack_free`: the NTP task's stack high-water mark, in bytes.

At WARN level (so it survives a fleet-wide `logger: level: WARN`), it reports when no PPS edges or no valid NMEA arrived during the interval. Those are the two symptoms of a wiring fault, and nothing else would log them.

## Tests

`tests/run.sh` builds the component against stub headers on the host and runs it against a scripted receiver: a normal start with a baud switch, a receiver already at the target baud, glitch edges, a stalled loop with a lost sentence at the first label, a GPS week rollover, an oversized UBX frame, a receiver without UBX (with and without `require_utc_valid`), a cold start with UTC not yet valid, loss of fix, a 20-minute outage, a 5 ms PPS phase step, an inserted leap second both with and without advance notice, and the signal-strength calculation. Each scenario checks the time the server would hand out against the simulated truth. It exercises the logic only; it says nothing about real capture jitter or network delay.

## Testing without a receiver: `gnss_sim`

`components/gnss_sim` is a **test-only** emulator of a u-blox receiver. It produces a 1 Hz PPS pulse, the default NMEA sentence set, and the UBX messages `pps_ntp` uses (NAV-TIMEUTC polls, CFG-PRT baud changes, CFG-CFG saves).

- **Two boards:** run it on a spare ESP32 and wire PPS, TX, RX and GND to the node under test. The two crystals differ, so the node's frequency-offset and jitter sensors show real numbers.
- **One board:** give it the same pins `pps_ntp` uses (its `tx_pin` is the node's UART RX pin, and so on). The signals are looped back inside the chip through the GPIO matrix, with no wires. Both ends share one crystal, so set `ppm:` to fake a frequency difference.

```yaml
external_components:
  - source: github://davidcoulson/esphome-pps-ntp@v0.3.3
    components: [pps_ntp, gnss_sim]

gnss_sim:
  id: sim
  pps_pin: 15   # plain numbers, so ESPHome allows sharing the pin with pps_ntp
  tx_pin: 17
  rx_pin: 16
  ppm: 12
```

Faults can be injected from lambdas: `set_pps_enabled`, `set_nmea_enabled`, `set_fix`, `set_utc_valid`, `set_answer_ubx`, `set_week_rollover`, `inject_glitch`, `step_phase_us`, `step_time_s`, `set_answer_timels` and `arm_leap(seconds)` (the reported date jumps to 31 December and `23:59:60` follows).

The time it reports is the node's own system clock at start-up (so the node needs a `time:` source), free-running after that. **Never point real NTP clients at a node fed by the emulator.**

## Verifying

From any machine on the LAN:

```bash
ntpdate -q <device-ip>          # quick check
sntp -d <device-ip>             # macOS
chronyc sources -v              # after adding "server <device-ip> iburst" to chrony.conf
```

## Limitations

- IPv6 needs `network: enable_ipv6: true` in the node's config, which compiles IPv6 into lwIP. Both transports then answer on IPv4 and IPv6 (the socket transport with one dual-stack socket).
- Leap seconds are only announced in advance (LI bits) with a receiver that supports `UBX-NAV-TIMELS` (u-blox 8 and later).
- NTP packet timestamps are taken in software, not by the Ethernet hardware, so whatever the network path inside the node costs is invisible to them. Measured on an ESP32-S3-ETH (W5500 over SPI) from a wired host one router hop away, 400 requests per run:

  | | median RTT | p95 RTT |
  |---|---|---|
  | ICMP ping (the floor for this path) | 1.8 ms | 2.0 ms |
  | `transport: socket` | 2.43 ms | 3.3 ms |
  | `transport: raw_lwip` | 1.98 ms | 2.15 ms |

  The router hop is about 0.35 ms of that, which leaves roughly 0.6–0.7 ms each way inside the W5500, its SPI driver and lwIP. NTP only suffers from the part of that which differs between the two directions, which these numbers can't show; assume a few hundred microseconds of possible error on SPI Ethernet. PPS capture itself is good to about a microsecond, so the network path is the whole error budget. An RMII MAC (ESP32-P4, or a classic ESP32 with a LAN8720) should be much better. `tools/ntpprobe.py` reproduces the measurement.
- Assumes the receiver's PPS rising edge marks the start of the UTC second (the u-blox default).

## Credits

Inspired by [roblatour/ESP32TimeServer](https://github.com/roblatour/ESP32TimeServer).

## License

MIT
