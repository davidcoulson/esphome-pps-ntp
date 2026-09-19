# esphome-pps-ntp

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
3. **Discipline.** A least-squares fit over the last 64 pulses gives the crystal's rate and phase. The ESP32 system clock is left alone; NTP timestamps are computed straight from the fit.
4. **Leap-second safety.** The receiver is polled with `UBX-NAV-TIMEUTC`, and the server reports itself as unsynchronised until the receiver confirms UTC is valid. (After a cold start, u-blox receivers can report time with the wrong leap-second count for up to about 12.5 minutes.) Receivers that don't answer UBX fall back to NMEA-only after 60 s.
5. **Serving.** A dedicated FreeRTOS task, pinned to the core the ESPHome loop isn't using, answers NTPv3 and NTPv4 client requests. It timestamps each request as soon as it arrives and each reply just before sending.
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
| Active GNSS antenna | Recommended for indoor installs. Put it at a window or outdoors. |

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
  - source: github://davidcoulson/esphome-pps-ntp@v0.2.1
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
| `pps_pin` | **required** | GPIO connected to the receiver's PPS output. |
| `port` | `123` | UDP port to listen on. |
| `hardware_capture` | `true` | Timestamp PPS with the MCPWM capture unit where the chip has one. Set to `false` to force the GPIO interrupt. |
| `gnss_baud_rate` | none | If set, sends the legacy `UBX-CFG-PRT` command at boot to move the receiver to this baud rate. It verifies the switch, saves it with `UBX-CFG-CFG`, and falls back to the original rate if the switch fails. Useful for NEO-6M/7M/M8 modules stuck at 9600. |
| `holdover` | `15min` | How long to keep serving stratum 1 after PPS or the fix is lost. |
| `fit_window` | `64` | Pulses in the least-squares fit (8–256). Longer windows average out more noise; shorter ones track temperature changes in the crystal faster. |
| `max_residual` | `1000us` | A pulse further than this from the model is an outlier; three in a row reset the fit. With hardware capture, about `50us` is a reasonable tighter setting once the node has proven stable. |
| `refid` | `GPS` | NTP reference ID sent to clients (1–4 ASCII characters, e.g. `PPS`). |
| `task_core` | auto | Pins the NTP task to core `0` or `1`. By default it runs on the core the ESPHome loop isn't using. Rejected at validation on single-core chips (C2/C3/C5/C6/C61/H2/S2) and on an ESP32 built with `CONFIG_FREERTOS_UNICORE`. A compile-time `#error` catches anything else that ends up single-core. |
| `update_interval` | `60s` | How often the sensors publish. |

### Sensors

| Key | Type | Meaning |
|---|---|---|
| `satellites` | sensor | Satellites used in the fix (from GGA). |
| `frequency_offset` | sensor (ppm) | Measured error of the ESP32 crystal. |
| `pps_jitter` | sensor (µs) | RMS scatter of the pulses around the fitted model. |
| `requests` | sensor | NTP requests served since boot. |
| `synced` | binary sensor | On while serving stratum 1. |

## Verifying

From any machine on the LAN:

```bash
ntpdate -q <device-ip>          # quick check
sntp -d <device-ip>             # macOS
chronyc sources -v              # after adding "server <device-ip> iburst" to chrony.conf
```

## Limitations

- IPv4 only.
- No leap-second announcements (the LI bits are never set to 1 or 2).
- NTP packet timestamps are taken in the socket task, not in hardware. On SPI Ethernet (W5500) that means roughly 50–200 µs of asymmetry, from the chip's interrupt, the SPI frame read and lwIP. This dominates the error budget: PPS capture is accurate to about 1 µs. It's far better than Wi-Fi or internet NTP, but not PTP-grade. An RMII MAC (for example ESP32-P4 or a classic ESP32 with a LAN8720) shortens this path.
- Assumes the receiver's PPS rising edge marks the start of the UTC second (the u-blox default).

## Credits

Inspired by [roblatour/ESP32TimeServer](https://github.com/roblatour/ESP32TimeServer).

## License

MIT
