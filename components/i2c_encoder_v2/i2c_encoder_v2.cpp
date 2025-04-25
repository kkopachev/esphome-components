#include "i2c_encoder_v2.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace i2c_encoder_v2 {

static const char *const TAG = "i2c_encoder_v2";
static const uint8_t ENCODER_ID_CODE = 0x53;

void I2CEncoderV2Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up I2C Encoder V2...");

  // Check device ID
  uint8_t id_code = this->read_encoder_byte(REG_IDCODE);
  if (id_code != ENCODER_ID_CODE) {
    ESP_LOGE(TAG, "Wrong device ID: expected 0x%02X, got 0x%02X", ENCODER_ID_CODE, id_code);
    this->mark_failed();
    return;
  }

  // Initialize encoder
  uint16_t conf = 0;

  // Configure the encoder based on options
  conf |= this->use_float_ ? FLOAT_DATA : INT_DATA;
  conf |= this->wrap_enabled_ ? WRAP_ENABLE : WRAP_DISABLE;
  conf |= this->direction_left_ ? DIRE_LEFT : DIRE_RIGHT;
  conf |= this->rgb_encoder_ ? RGB_ENCODER : STD_ENCODER;

  // GCONF2 register options
  conf |= CLK_STRECH_DISABLE;  // Default to no clock stretching
  conf |= this->relative_mode_ ? REL_MODE_ENABLE : REL_MODE_DISABLE;

  this->begin(conf);

  // Initialize parameters
  if (this->use_float_) {
    this->write_min(this->min_value_);
    this->write_max(this->max_value_);
    this->write_step(this->step_value_);
    this->write_counter(this->initial_position_);
  } else {
    this->write_min(static_cast<int32_t>(this->min_value_));
    this->write_max(static_cast<int32_t>(this->max_value_));
    this->write_step(static_cast<int32_t>(this->step_value_));
    this->write_counter(static_cast<int32_t>(this->initial_position_));
  }

  // Configure double push
  this->write_encoder_byte(REG_DPPERIOD, this->double_push_period_);

  // Configure anti-bouncing period
  this->write_encoder_byte(REG_ANTBOUNC, this->anti_bouncing_);

  // Configure fade effects
  this->write_encoder_byte(REG_FADERGB, this->fade_rgb_);

  // Configure gamma correction
  this->write_encoder_byte(REG_GAMRLED, this->gamma_r_);
  this->write_encoder_byte(REG_GAMGLED, this->gamma_g_);
  this->write_encoder_byte(REG_GAMBLED, this->gamma_b_);

  // Configure interrupt pin if available
  if (this->interrupt_pin_ != nullptr) {
    this->interrupt_pin_->setup();
    // Configure interrupts
    this->auto_config_interrupt();
  }

  this->position_sensor_->publish_state(this->initial_position_);

  ESP_LOGD(TAG, "I2C Encoder V2 initialized");
}

void I2CEncoderV2Component::loop() {
  // Poll periodically
  uint32_t now = millis();
  if (now - this->last_update_time_ >= UPDATE_INTERVAL_MS) {
    this->update_status();
    this->last_update_time_ = now;
  }
}

void I2CEncoderV2Component::dump_config() {
  ESP_LOGCONFIG(TAG, "I2C Encoder V2:");
  LOG_I2C_DEVICE(this);

  // Log configuration
  LOG_SENSOR("  ", "Position", this->position_sensor_);
  LOG_BINARY_SENSOR("  ", "Button", this->button_sensor_);

  ESP_LOGCONFIG(TAG, "  Min Value: %.2f", this->min_value_);
  ESP_LOGCONFIG(TAG, "  Max Value: %.2f", this->max_value_);
  ESP_LOGCONFIG(TAG, "  Step Value: %.2f", this->step_value_);
  ESP_LOGCONFIG(TAG, "  Initial Position: %.2f", this->initial_position_);
  ESP_LOGCONFIG(TAG, "  Use Float: %s", YESNO(this->use_float_));
  ESP_LOGCONFIG(TAG, "  Wrap Enabled: %s", YESNO(this->wrap_enabled_));
  ESP_LOGCONFIG(TAG, "  Direction Left: %s", YESNO(this->direction_left_));
  ESP_LOGCONFIG(TAG, "  RGB Encoder: %s", YESNO(this->rgb_encoder_));
  ESP_LOGCONFIG(TAG, "  Relative Mode: %s", YESNO(this->relative_mode_));
  ESP_LOGCONFIG(TAG, "  Double Push Enabled: %s", YESNO(this->double_push_enabled_));

  if (this->double_push_enabled_)
    ESP_LOGCONFIG(TAG, "  Double Push Period: %d ms", this->double_push_period_ * 10);

  ESP_LOGCONFIG(TAG, "  Anti-bouncing: %d ms", this->anti_bouncing_ * 192 / 1000);

  if (this->rgb_encoder_) {
    ESP_LOGCONFIG(TAG, "  Fade RGB: %d ms", this->fade_rgb_);
    ESP_LOGCONFIG(TAG, "  Gamma R: %d", this->gamma_r_);
    ESP_LOGCONFIG(TAG, "  Gamma G: %d", this->gamma_g_);
    ESP_LOGCONFIG(TAG, "  Gamma B: %d", this->gamma_b_);
  }

  LOG_PIN("  Interrupt Pin: ", this->interrupt_pin_);
}

void I2CEncoderV2Component::begin(uint16_t conf) {
  // Write configuration registers
  this->write_encoder_byte(REG_GCONF, static_cast<uint8_t>(conf & 0xFF));
  this->write_encoder_byte(REG_GCONF2, static_cast<uint8_t>((conf >> 8) & 0xFF));
  this->gconf_ = conf;
}

void I2CEncoderV2Component::reset() {
  this->write_encoder_byte(REG_GCONF, 0x80);
  delay(10);
}

bool I2CEncoderV2Component::update_status() {
  this->status_ = this->read_encoder_byte(REG_ESTATUS);
  this->status2_ = 0;

  if (this->status_ == 0) {
    return false;
  }

  // Handle encoder button events
  if (this->status_ & PUSHR) {
    this->on_button_released_callback_.call();
    if (this->button_sensor_ != nullptr) {
      this->button_sensor_->publish_state(false);
    }
  }

  if (this->status_ & PUSHP) {
    this->on_button_pressed_callback_.call();
    if (this->button_sensor_ != nullptr) {
      this->button_sensor_->publish_state(true);
    }
  }

  if (this->status_ & PUSHD) {
    this->on_button_double_pressed_callback_.call();
  }

  // Handle encoder rotation events
  if (this->status_ & RINC) {
    this->on_increment_callback_.call();
    if (this->position_sensor_ != nullptr) {
      float value = this->use_float_ ? this->read_counter_float() : static_cast<float>(this->read_counter_long());
      this->position_sensor_->publish_state(value);
    }
  }

  if (this->status_ & RDEC) {
    this->on_decrement_callback_.call();
    if (this->position_sensor_ != nullptr) {
      float value = this->use_float_ ? this->read_counter_float() : static_cast<float>(this->read_counter_long());
      this->position_sensor_->publish_state(value);
    }
  }

  // Handle min/max events
  if (this->status_ & RMAX) {
    this->on_max_callback_.call();
  }

  if (this->status_ & RMIN) {
    this->on_min_callback_.call();
  }

  // Handle INT2 events
  if ((this->status_ & INT_2) != 0) {
    this->status2_ = this->read_encoder_byte(REG_I2STATUS);
  }

  return true;
}

uint8_t I2CEncoderV2Component::read_button_status() { return this->status_ & (PUSHR | PUSHP | PUSHD); }

void I2CEncoderV2Component::set_rgb_led_r(uint8_t r) { this->write_encoder_byte(REG_RLED, r); }

void I2CEncoderV2Component::set_rgb_led_g(uint8_t g) { this->write_encoder_byte(REG_GLED, g); }

void I2CEncoderV2Component::set_rgb_led_b(uint8_t b) { this->write_encoder_byte(REG_BLED, b); }

void I2CEncoderV2Component::set_rgb_led(uint8_t r, uint8_t g, uint8_t b) {
  this->set_rgb_led_r(r);
  this->set_rgb_led_g(g);
  this->set_rgb_led_b(b);
}

void I2CEncoderV2Component::increment() {
  if (this->use_float_) {
    float current = this->read_counter_float();
    current += this->step_value_;
    if (current > this->max_value_) {
      current = this->wrap_enabled_ ? this->min_value_ : this->max_value_;
    }
    this->write_counter(current);
  } else {
    int32_t current = this->read_counter_long();
    int32_t step = static_cast<int32_t>(this->step_value_);
    current += step;
    if (current > static_cast<int32_t>(this->max_value_)) {
      current = this->wrap_enabled_ ? static_cast<int32_t>(this->min_value_) : static_cast<int32_t>(this->max_value_);
    }
    this->write_counter(current);
  }

  if (this->position_sensor_ != nullptr) {
    float value = this->use_float_ ? this->read_counter_float() : static_cast<float>(this->read_counter_long());
    this->position_sensor_->publish_state(value);
  }
}

void I2CEncoderV2Component::decrement() {
  if (this->use_float_) {
    float current = this->read_counter_float();
    current -= this->step_value_;
    if (current < this->min_value_) {
      current = this->wrap_enabled_ ? this->max_value_ : this->min_value_;
    }
    this->write_counter(current);
  } else {
    int32_t current = this->read_counter_long();
    int32_t step = static_cast<int32_t>(this->step_value_);
    current -= step;
    if (current < static_cast<int32_t>(this->min_value_)) {
      current = this->wrap_enabled_ ? static_cast<int32_t>(this->max_value_) : static_cast<int32_t>(this->min_value_);
    }
    this->write_counter(current);
  }

  if (this->position_sensor_ != nullptr) {
    float value = this->use_float_ ? this->read_counter_float() : static_cast<float>(this->read_counter_long());
    this->position_sensor_->publish_state(value);
  }
}

void I2CEncoderV2Component::set_position(float position) {
  if (this->use_float_) {
    this->write_counter(position);
  } else {
    this->write_counter(static_cast<int32_t>(position));
  }

  if (this->position_sensor_ != nullptr) {
    this->position_sensor_->publish_state(position);
  }
}

float I2CEncoderV2Component::read_counter_float() { return this->read_encoder_float(REG_CVALB4); }

int32_t I2CEncoderV2Component::read_counter_long() { return this->read_encoder_long(REG_CVALB4); }

void I2CEncoderV2Component::write_counter(int32_t counter) { this->write_encoder_long(REG_CVALB4, counter); }

void I2CEncoderV2Component::write_counter(float counter) { this->write_encoder_float(REG_CVALB4, counter); }

void I2CEncoderV2Component::write_max(int32_t max) { this->write_encoder_long(REG_CMAXB4, max); }

void I2CEncoderV2Component::write_max(float max) { this->write_encoder_float(REG_CMAXB4, max); }

void I2CEncoderV2Component::write_min(int32_t min) { this->write_encoder_long(REG_CMINB4, min); }

void I2CEncoderV2Component::write_min(float min) { this->write_encoder_float(REG_CMINB4, min); }

void I2CEncoderV2Component::write_step(int32_t step) { this->write_encoder_long(REG_ISTEPB4, step); }

void I2CEncoderV2Component::write_step(float step) { this->write_encoder_float(REG_ISTEPB4, step); }

void I2CEncoderV2Component::write_interrupt_config(uint8_t interrupt) {
  this->write_encoder_byte(REG_INTCONF, interrupt);
}

void I2CEncoderV2Component::auto_config_interrupt() {
  uint8_t reg = 0;

  // Enable all relevant interrupts
  reg |= PUSHR | PUSHP;

  if (this->double_push_enabled_)
    reg |= PUSHD;

  reg |= RINC | RDEC;
  reg |= RMAX | RMIN;

  this->write_interrupt_config(reg);
}

// Low-level I2C functions
uint8_t I2CEncoderV2Component::read_encoder_byte(uint8_t reg) {
  uint8_t data;
  auto ret = this->read_register(reg, &data, 1);
  if (ret != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "Failed to read register 0x%02X", reg);
    return 0;
  }
  return data;
}

int16_t I2CEncoderV2Component::read_encoder_int(uint8_t reg) {
  uint8_t data[2];
  auto ret = this->read_register(reg, data, 2);
  if (ret != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "Failed to read register 0x%02X", reg);
    return 0;
  }
  return (data[0] << 8) | data[1];
}

int32_t I2CEncoderV2Component::read_encoder_long(uint8_t reg) {
  uint8_t data[4];
  auto ret = this->read_register(reg, data, 4);
  if (ret != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "Failed to read register 0x%02X", reg);
    return 0;
  }
  return (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
}

float I2CEncoderV2Component::read_encoder_float(uint8_t reg) {
  uint8_t data[4];
  auto ret = this->read_register(reg, data, 4);
  if (ret != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "Failed to read register 0x%02X", reg);
    return 0.0f;
  }

  Data_v temp_data;
  temp_data.bval[3] = data[0];
  temp_data.bval[2] = data[1];
  temp_data.bval[1] = data[2];
  temp_data.bval[0] = data[3];

  return temp_data.fval;
}

void I2CEncoderV2Component::write_encoder_byte(uint8_t reg, uint8_t data) {
  auto ret = this->write_register(reg, &data, 1);

  if (ret != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Failed to write register 0x%02X Err=%d", reg, ret);
  }
}

void I2CEncoderV2Component::write_encoder_long(uint8_t reg, int32_t data) {
  Data_v temp_data;
  temp_data.val = data;

  uint8_t buffer[4];
  buffer[0] = temp_data.bval[3];
  buffer[1] = temp_data.bval[2];
  buffer[2] = temp_data.bval[1];
  buffer[3] = temp_data.bval[0];

  auto ret = this->write_register(reg, buffer, 4);

  if (ret != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "Failed to write register 0x%02X", reg);
  }
}

void I2CEncoderV2Component::write_encoder_float(uint8_t reg, float data) {
  Data_v temp_data;
  temp_data.fval = data;

  uint8_t buffer[4];
  buffer[0] = temp_data.bval[3];
  buffer[1] = temp_data.bval[2];
  buffer[2] = temp_data.bval[1];
  buffer[3] = temp_data.bval[0];

  auto ret = this->write_register(reg, buffer, 4);
  if (ret != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "Failed to write register 0x%02X", reg);
  }
}

void I2CEncoderV2Component::write_encoder_rgb(uint8_t reg, uint32_t rgb) {
  uint8_t buffer[3];
  buffer[0] = (rgb >> 16) & 0xFF;  // R
  buffer[1] = (rgb >> 8) & 0xFF;   // G
  buffer[2] = rgb & 0xFF;          // B

  auto ret = this->write_register(reg, buffer, 3);
  if (ret != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "Failed to write register 0x%02X", reg);
  }
}

void I2CEncoderV2LightOutput::write_state(light::LightState *state) {
  float red, green, blue;
  state->current_values_as_rgb(&red, &green, &blue);

  // Scale to 0-255
  uint8_t r = static_cast<uint8_t>(red * 255);
  uint8_t g = static_cast<uint8_t>(green * 255);
  uint8_t b = static_cast<uint8_t>(blue * 255);

  this->parent_->set_rgb_led(r, g, b);
}

}  // namespace i2c_encoder_v2
}  // namespace esphome
