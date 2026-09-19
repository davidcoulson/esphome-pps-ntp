#pragma once
namespace esphome {
class Component { public: virtual ~Component() = default; virtual void setup() {} virtual void loop() {} virtual void dump_config() {} void mark_failed() {} };
class PollingComponent : public Component { public: virtual void update() {} };
}  // namespace esphome
