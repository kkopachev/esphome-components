#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "i2c_encoder_v2.h"

namespace esphome {
namespace i2c_encoder_v2 {

template<typename... Ts> class I2CEncoderSetPositionAction : public Action<Ts...> {
 public:
  I2CEncoderSetPositionAction(I2CEncoderV2Component *parent) : parent_(parent) {}

  void set_position(float position) { this->position_ = position; }
  void set_position(std::function<float(Ts...)> &&f) { this->position_func_ = f; }

  void play(Ts... args) override {
    float pos;
    if (this->position_func_.has_value()) {
      pos = (*this->position_func_)(args...);
    } else {
      pos = this->position_;
    }
    this->parent_->set_position(pos);
  }

 protected:
  I2CEncoderV2Component *parent_;
  float position_{0};
  optional<std::function<float(Ts...)>> position_func_{};
};

template<typename... Ts> class I2CEncoderIncrementAction : public Action<Ts...> {
 public:
  I2CEncoderIncrementAction(I2CEncoderV2Component *parent) : parent_(parent) {}

  void play(Ts... args) override { this->parent_->increment(); }

 protected:
  I2CEncoderV2Component *parent_;
};

template<typename... Ts> class I2CEncoderDecrementAction : public Action<Ts...> {
 public:
  I2CEncoderDecrementAction(I2CEncoderV2Component *parent) : parent_(parent) {}

  void play(Ts... args) override { this->parent_->decrement(); }

 protected:
  I2CEncoderV2Component *parent_;
};

}  // namespace i2c_encoder_v2
}  // namespace esphome
