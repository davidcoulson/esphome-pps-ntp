import esphome.codegen as cg
from esphome import pins
from esphome.components import binary_sensor, esp32, sensor, uart
from esphome.components.esp32 import (
    VARIANT_ESP32,
    VARIANT_ESP32C5,
    VARIANT_ESP32C6,
    VARIANT_ESP32H2,
    VARIANT_ESP32P4,
    VARIANT_ESP32S3,
)
import esphome.config_validation as cv
import esphome.final_validate as fv
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
CONF_HARDWARE_CAPTURE = "hardware_capture"
CONF_GNSS_BAUD_RATE = "gnss_baud_rate"
CONF_HOLDOVER = "holdover"
CONF_SYNCED = "synced"
CONF_FREQUENCY_OFFSET = "frequency_offset"
CONF_PPS_JITTER = "pps_jitter"
CONF_REQUESTS = "requests"
CONF_FIT_WINDOW = "fit_window"
CONF_MAX_RESIDUAL = "max_residual"
CONF_REFID = "refid"
CONF_TASK_CORE = "task_core"

UNIT_MICROSECOND = "µs"

# Variants with an MCPWM capture unit, which latches the PPS edge in hardware
MCPWM_VARIANTS = (
    VARIANT_ESP32,
    VARIANT_ESP32S3,
    VARIANT_ESP32C5,
    VARIANT_ESP32C6,
    VARIANT_ESP32H2,
    VARIANT_ESP32P4,
)

# Variants with two FreeRTOS cores (the classic ESP32 only unless built with CONFIG_FREERTOS_UNICORE)
DUAL_CORE_VARIANTS = (VARIANT_ESP32, VARIANT_ESP32S3, VARIANT_ESP32P4)


def validate_refid(value):
    value = cv.string_strict(value)
    if not 1 <= len(value) <= 4 or not value.isascii() or not value.isprintable():
        raise cv.Invalid("refid must be 1-4 printable ASCII characters, e.g. GPS or PPS")
    return value


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
            # Timestamp PPS with the MCPWM capture unit where the chip has one; false forces the GPIO interrupt
            cv.Optional(CONF_HARDWARE_CAPTURE, default=True): cv.boolean,
            # Pulses in the least-squares fit: longer is quieter, shorter tracks temperature faster
            cv.Optional(CONF_FIT_WINDOW, default=64): cv.int_range(min=8, max=256),
            # A pulse further than this from the model is an outlier (3 in a row reset the fit)
            cv.Optional(CONF_MAX_RESIDUAL, default="1000us"): cv.All(
                cv.positive_time_period_microseconds,
                cv.Range(min=cv.TimePeriod(microseconds=10)),
            ),
            # NTP reference identifier sent to clients
            cv.Optional(CONF_REFID, default="GPS"): validate_refid,
            # Pin the NTP task to a core; by default it goes on the core the ESPHome loop isn't using
            cv.Optional(CONF_TASK_CORE): cv.int_range(min=0, max=1),
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


def _final_validate_task_core(config):
    if CONF_TASK_CORE not in config:
        return config
    variant = esp32.get_esp32_variant()
    if variant not in DUAL_CORE_VARIANTS:
        raise cv.Invalid(
            f"task_core needs a dual-core chip; {variant} has a single core",
            path=[CONF_TASK_CORE],
        )
    sdkconfig = (
        fv.full_config.get()
        .get("esp32", {})
        .get("framework", {})
        .get("sdkconfig_options", {})
    )
    if str(sdkconfig.get("CONFIG_FREERTOS_UNICORE", "n")).lower() in ("y", "true", "1"):
        raise cv.Invalid(
            "task_core needs two FreeRTOS cores; this build sets CONFIG_FREERTOS_UNICORE",
            path=[CONF_TASK_CORE],
        )
    return config


FINAL_VALIDATE_SCHEMA = cv.All(
    uart.final_validate_device_schema("pps_ntp", require_rx=True, require_tx=True),
    _final_validate_task_core,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    pin = await cg.gpio_pin_expression(config[CONF_PPS_PIN])
    cg.add(var.set_pps_pin(pin))
    if config[CONF_HARDWARE_CAPTURE] and esp32.get_esp32_variant() in MCPWM_VARIANTS:
        esp32.include_builtin_idf_component("esp_driver_mcpwm")
        cg.add_define("USE_PPS_NTP_MCPWM")
    cg.add(var.set_port(config[CONF_PORT]))
    cg.add(var.set_holdover_s(config[CONF_HOLDOVER].total_seconds))
    cg.add(var.set_fit_window(config[CONF_FIT_WINDOW]))
    cg.add(var.set_max_residual_us(config[CONF_MAX_RESIDUAL].total_microseconds))
    cg.add(var.set_refid(config[CONF_REFID]))
    if CONF_TASK_CORE in config:
        # Also checked at compile time against the build's FreeRTOS core count
        cg.add_define("USE_PPS_NTP_TASK_CORE", config[CONF_TASK_CORE])
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
