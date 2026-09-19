#pragma once
namespace esphome::binary_sensor { struct BinarySensor { bool state{false}; void publish_state(bool v) { state = v; } }; }
