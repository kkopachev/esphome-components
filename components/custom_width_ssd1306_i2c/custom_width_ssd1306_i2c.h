#pragma once

#include "esphome/core/component.h"
#include "esphome/components/ssd1306_i2c/ssd1306_i2c.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace custom_width_ssd1306_i2c {

class CustomWidthSH1106 : public ssd1306_i2c::I2CSSD1306 {
 public:
  // Set custom width
  void set_custom_width(uint8_t width) { this->custom_width_ = width; }

  // Override get_width_internal to use our custom width
  int get_width_internal() override;

  // For dumping config information
  void dump_config() override;

 protected:
  uint8_t custom_width_{128};  // Default to standard 128 width
};

}  // namespace custom_width_ssd1306_i2c
}  // namespace esphome
