#include "custom_width_ssd1306_i2c.h"
#include "esphome/core/log.h"

namespace esphome {
namespace custom_width_ssd1306_i2c {

static const char *const TAG = "custom_width_sh1106";

// Override the get_width_internal method to use our custom width
int CustomWidthSH1106::get_width_internal() { return this->custom_width_; }

void CustomWidthSH1106::dump_config() {
  // First call parent dump_config to get standard information
  ssd1306_i2c::I2CSSD1306::dump_config();

  // Add our custom width information
  ESP_LOGCONFIG(TAG, "  Custom Width: %d", this->custom_width_);
}

}  // namespace custom_width_ssd1306_i2c
}  // namespace esphome
