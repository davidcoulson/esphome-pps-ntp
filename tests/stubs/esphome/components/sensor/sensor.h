#pragma once
namespace esphome::sensor { struct Sensor { float state{0}; void publish_state(float v) { state = v; } }; }
