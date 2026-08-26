#pragma once

#include "esphome/core/component.h"
#include "esphome/components/ssd1306_base/ssd1306_base.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace custom_width_ssd1306_i2c {

// ssd1306_i2c::I2CSSD1306 is declared `final` upstream, so this mirrors its I2C
// transport on top of ssd1306_base::SSD1306 instead of deriving from it.
class CustomWidthSSD1306 : public ssd1306_base::SSD1306, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;

  // Set custom width
  void set_custom_width(uint8_t width) { this->custom_width_ = width; }

  // Override get_width_internal to use our custom width
  int get_width_internal() override;

 protected:
  void command(uint8_t value) override;
  void write_display_data() override;

  enum ErrorCode { NONE = 0, COMMUNICATION_FAILED } error_code_{NONE};

  uint8_t custom_width_{128};  // Default to standard 128 width
};

}  // namespace custom_width_ssd1306_i2c
}  // namespace esphome
