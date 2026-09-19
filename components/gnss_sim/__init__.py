"""Test-only emulator of a u-blox GNSS receiver: 1 Hz PPS, NMEA, and the UBX messages pps_ntp uses.

Runs on its own board wired to the node under test, or on the SAME board as pps_ntp: give it the
very pins pps_ntp uses (its tx_pin = pps_ntp's UART rx pin, and so on) and the signals are looped
back inside the chip through the GPIO matrix, with no wires. Pins are plain numbers on purpose,
so ESPHome's pin-reuse check doesn't object to that sharing.

The time it reports is the system clock at start-up (so give the node a `time:` source) and then
free-runs. It is only as right as that clock: never point real NTP clients at a node fed by this.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@davidcoulson"]

CONF_PPS_PIN = "pps_pin"
CONF_TX_PIN = "tx_pin"
CONF_RX_PIN = "rx_pin"
CONF_UART_NUM = "uart_num"
CONF_BAUD_RATE = "baud_rate"
CONF_PPM = "ppm"
CONF_PULSE_WIDTH = "pulse_width"
CONF_NMEA_DELAY = "nmea_delay"
CONF_SATELLITES = "satellites"

gnss_sim_ns = cg.esphome_ns.namespace("gnss_sim")
GNSSSim = gnss_sim_ns.class_("GNSSSim", cg.Component)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(GNSSSim),
            cv.Required(CONF_PPS_PIN): cv.int_range(min=0, max=54),
            cv.Required(CONF_TX_PIN): cv.int_range(min=0, max=54),
            cv.Required(CONF_RX_PIN): cv.int_range(min=0, max=54),
            # Hardware UART the emulator drives directly; ESPHome's own uart: components count up from 0
            cv.Optional(CONF_UART_NUM, default=2): cv.int_range(min=0, max=4),
            cv.Optional(CONF_BAUD_RATE, default=9600): cv.positive_int,
            # Stretch the pulse period, as if this board's crystal differed from the node's by this much
            cv.Optional(CONF_PPM, default=0): cv.int_range(min=-200, max=200),
            cv.Optional(CONF_PULSE_WIDTH, default="10ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_NMEA_DELAY, default="100ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_SATELLITES, default=9): cv.int_range(min=0, max=32),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on_esp32,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_pins(config[CONF_PPS_PIN], config[CONF_TX_PIN], config[CONF_RX_PIN]))
    cg.add(var.set_uart_num(config[CONF_UART_NUM]))
    cg.add(var.set_baud_rate(config[CONF_BAUD_RATE]))
    cg.add(var.set_ppm(config[CONF_PPM]))
    cg.add(var.set_pulse_width_ms(config[CONF_PULSE_WIDTH].total_milliseconds))
    cg.add(var.set_nmea_delay_ms(config[CONF_NMEA_DELAY].total_milliseconds))
    cg.add(var.set_satellites(config[CONF_SATELLITES]))
