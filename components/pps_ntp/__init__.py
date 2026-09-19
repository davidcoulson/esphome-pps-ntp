import esphome.codegen as cg
from esphome import pins
from esphome.components import binary_sensor, sensor, uart
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_PORT,
    CONF_SATELLITES,
    DEVICE_CLASS_CONNECTIVITY,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_PARTS_PER_MILLION,
)

DEPENDENCIES = ["uart", "network"]
AUTO_LOAD = ["sensor", "binary_sensor"]
CODEOWNERS = ["@davidcoulson"]

CONF_PPS_PIN = "pps_pin"
CONF_GNSS_BAUD_RATE = "gnss_baud_rate"
CONF_HOLDOVER = "holdover"
CONF_SYNCED = "synced"
CONF_FREQUENCY_OFFSET = "frequency_offset"
CONF_PPS_JITTER = "pps_jitter"
CONF_REQUESTS = "requests"

UNIT_MICROSECOND = "µs"

pps_ntp_ns = cg.esphome_ns.namespace("pps_ntp")
PPSNTPServer = pps_ntp_ns.class_(
    "PPSNTPServer", cg.PollingComponent, uart.UARTDevice
)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(PPSNTPServer),
            cv.Required(CONF_PPS_PIN): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_PORT, default=123): cv.port,
            # Raise an older u-blox module (NEO-6M/7M/M8, GT-U7) to this baud using legacy UBX-CFG-PRT
            cv.Optional(CONF_GNSS_BAUD_RATE): cv.one_of(
                9600, 19200, 38400, 57600, 115200, 230400, int=True
            ),
            # How long to keep serving stratum 1 on the local crystal after PPS is lost
            cv.Optional(
                CONF_HOLDOVER, default="15min"
            ): cv.positive_time_period_seconds,
            cv.Optional(CONF_SATELLITES): sensor.sensor_schema(
                icon="mdi:satellite-variant",
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_FREQUENCY_OFFSET): sensor.sensor_schema(
                unit_of_measurement=UNIT_PARTS_PER_MILLION,
                icon="mdi:sine-wave",
                accuracy_decimals=3,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_PPS_JITTER): sensor.sensor_schema(
                unit_of_measurement=UNIT_MICROSECOND,
                icon="mdi:pulse",
                accuracy_decimals=1,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_REQUESTS): sensor.sensor_schema(
                icon="mdi:clock-check-outline",
                accuracy_decimals=0,
                state_class=STATE_CLASS_TOTAL_INCREASING,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_SYNCED): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_CONNECTIVITY,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(uart.UART_DEVICE_SCHEMA),
    cv.only_on_esp32,
)
FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "pps_ntp", require_rx=True, require_tx=True
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    pin = await cg.gpio_pin_expression(config[CONF_PPS_PIN])
    cg.add(var.set_pps_pin(pin))
    cg.add(var.set_port(config[CONF_PORT]))
    cg.add(var.set_holdover_s(config[CONF_HOLDOVER].total_seconds))
    if CONF_GNSS_BAUD_RATE in config:
        cg.add(var.set_gnss_baud_rate(config[CONF_GNSS_BAUD_RATE]))

    for key, setter in (
        (CONF_SATELLITES, "set_satellites_sensor"),
        (CONF_FREQUENCY_OFFSET, "set_frequency_offset_sensor"),
        (CONF_PPS_JITTER, "set_pps_jitter_sensor"),
        (CONF_REQUESTS, "set_requests_sensor"),
    ):
        if conf := config.get(key):
            sens = await sensor.new_sensor(conf)
            cg.add(getattr(var, setter)(sens))

    if conf := config.get(CONF_SYNCED):
        bs = await binary_sensor.new_binary_sensor(conf)
        cg.add(var.set_synced_binary_sensor(bs))
