#pragma once

#include "esphome/components/ssd1306_i2c/ssd1306_i2c.h"

namespace esphome {
namespace custom_width_ssd1306_i2c {

// Simply extend the existing I2CSSD1306 class with our custom width functionality
class CustomWidthSSD1306 : public ssd1306_i2c::I2CSSD1306 {
 public:
  // Set custom width
  void set_custom_width(uint8_t width) { this->custom_width_ = width; }

  // Override get_width_internal to use our custom width
  int get_width_internal() override;

  // For logging the custom width
  void dump_config() override;

 protected:
  uint8_t custom_width_{128};  // Default to standard 128 width
};

}  // namespace custom_width_ssd1306_i2c
}  // namespace esphome
