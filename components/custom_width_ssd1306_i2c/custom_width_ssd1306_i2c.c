#include "custom_width_ssd1306_i2c.h"
#include "esphome/core/log.h"

namespace esphome {
namespace custom_width_ssd1306_i2c {

static const char *const TAG = "custom_width_ssd1306_i2c";

// Override to return our custom width
int CustomWidthSSD1306::get_width_internal() { return this->custom_width_; }

// Override dump_config to also show our custom width
void CustomWidthSSD1306::dump_config() {
  // First call the parent's dump_config to show the standard information
  ssd1306_i2c::I2CSSD1306::dump_config();

  // Then add our custom width information
  ESP_LOGCONFIG(TAG, "  Custom Width: %d", this->custom_width_);
}

}  // namespace custom_width_ssd1306_i2c
}  // namespace esphome
