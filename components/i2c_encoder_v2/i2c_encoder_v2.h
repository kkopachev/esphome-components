#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/automation.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/light/light_output.h"
#include "esphome/core/gpio.h"

namespace esphome {
namespace i2c_encoder_v2 {

// Register definitions
enum I2C_Register {
  REG_GCONF = 0x00,
  REG_GP1CONF = 0x01,
  REG_GP2CONF = 0x02,
  REG_GP3CONF = 0x03,
  REG_INTCONF = 0x04,
  REG_ESTATUS = 0x05,
  REG_I2STATUS = 0x06,
  REG_FSTATUS = 0x07,
  REG_CVALB4 = 0x08,
  REG_CVALB3 = 0x09,
  REG_CVALB2 = 0x0A,
  REG_CVALB1 = 0x0B,
  REG_CMAXB4 = 0x0C,
  REG_CMAXB3 = 0x0D,
  REG_CMAXB2 = 0x0E,
  REG_CMAXB1 = 0x0F,
  REG_CMINB4 = 0x10,
  REG_CMINB3 = 0x11,
  REG_CMINB2 = 0x12,
  REG_CMINB1 = 0x13,
  REG_ISTEPB4 = 0x14,
  REG_ISTEPB3 = 0x15,
  REG_ISTEPB2 = 0x16,
  REG_ISTEPB1 = 0x17,
  REG_RLED = 0x18,
  REG_GLED = 0x19,
  REG_BLED = 0x1A,
  REG_GP1REG = 0x1B,
  REG_GP2REG = 0x1C,
  REG_GP3REG = 0x1D,
  REG_ANTBOUNC = 0x1E,
  REG_DPPERIOD = 0x1F,
  REG_FADERGB = 0x20,
  REG_FADEGP = 0x21,
  REG_GAMRLED = 0x27,
  REG_GAMGLED = 0x28,
  REG_GAMBLED = 0x29,
  REG_GAMMAGP1 = 0x2A,
  REG_GAMMAGP2 = 0x2B,
  REG_GAMMAGP3 = 0x2C,
  REG_GCONF2 = 0x30,
  REG_IDCODE = 0x70,
  REG_VERSION = 0x71,
  REG_EEPROMS = 0x80
};

// Encoder configuration bits
enum GCONF_PARAMETER {
  FLOAT_DATA = 0x0001,
  INT_DATA = 0x0000,
  WRAP_ENABLE = 0x0002,
  WRAP_DISABLE = 0x0000,
  DIRE_LEFT = 0x0004,
  DIRE_RIGHT = 0x0000,
  IPUP_DISABLE = 0x0008,
  IPUP_ENABLE = 0x0000,
  RMOD_X2 = 0x0010,
  RMOD_X1 = 0x0000,
  RGB_ENCODER = 0x0020,
  STD_ENCODER = 0x0000,
  EEPROM_BANK1 = 0x0040,
  EEPROM_BANK2 = 0x0000,
  RESET = 0x0080,
  CLK_STRECH_ENABLE = 0x0100,
  CLK_STRECH_DISABLE = 0x0000,
  REL_MODE_ENABLE = 0x0200,
  REL_MODE_DISABLE = 0x0000
};

// Encoder status bits
enum Int_Status {
  PUSHR = 0x01,
  PUSHP = 0x02,
  PUSHD = 0x04,
  RINC = 0x08,
  RDEC = 0x10,
  RMAX = 0x20,
  RMIN = 0x40,
  INT_2 = 0x80
};

// Secondary interrupt status
enum Int2_Status {
  GP1_POS = 0x01,
  GP1_NEG = 0x02,
  GP2_POS = 0x04,
  GP2_NEG = 0x08,
  GP3_POS = 0x10,
  GP3_NEG = 0x20,
  FADE_INT = 0x40
};

// Fade status
enum Fade_Status { FADE_R = 0x01, FADE_G = 0x02, FADE_B = 0x04, FADE_GP1 = 0x08, FADE_GP2 = 0x10, FADE_GP3 = 0x20 };

// GPIO configuration
enum GP_PARAMETER {
  GP_PWM = 0x00,
  GP_OUT = 0x01,
  GP_AN = 0x02,
  GP_IN = 0x03,
  GP_PULL_EN = 0x04,
  GP_PULL_DI = 0x00,
  GP_INT_DI = 0x00,
  GP_INT_PE = 0x08,
  GP_INT_NE = 0x10,
  GP_INT_BE = 0x18
};

// Gamma parameters
enum GAMMA_PARAMETER {
  GAMMA_OFF = 0,
  GAMMA_1 = 1,
  GAMMA_1_8 = 2,
  GAMMA_2 = 3,
  GAMMA_2_2 = 4,
  GAMMA_2_4 = 5,
  GAMMA_2_6 = 6,
  GAMMA_2_8 = 7
};

// Data union for 32-bit values
union Data_v {
  float fval;
  int32_t val;
  uint8_t bval[4];
};

class I2CEncoderV2Component : public Component, public i2c::I2CDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Configuration methods
  void begin(uint16_t conf);
  void reset();

  // Set entities
  void set_position_sensor(sensor::Sensor *position_sensor) { this->position_sensor_ = position_sensor; }
  void set_button_sensor(binary_sensor::BinarySensor *button_sensor) { this->button_sensor_ = button_sensor; }
  void set_interrupt_pin(GPIOPin *pin) { this->interrupt_pin_ = pin; }

  // Configuration methods
  void set_min_value(float min_value) { this->min_value_ = min_value; }
  void set_max_value(float max_value) { this->max_value_ = max_value; }
  void set_step_value(float step_value) { this->step_value_ = step_value; }
  void set_initial_position(float position) { this->initial_position_ = position; }
  void set_use_float(bool use_float) { this->use_float_ = use_float; }
  void set_wrap_enabled(bool wrap_enabled) { this->wrap_enabled_ = wrap_enabled; }
  void set_direction_left(bool direction_left) { this->direction_left_ = direction_left; }
  void set_double_push_enabled(bool enabled) { this->double_push_enabled_ = enabled; }
  void set_double_push_period(uint8_t period) { this->double_push_period_ = period; }
  void set_anti_bouncing(uint8_t anti_bouncing) { this->anti_bouncing_ = anti_bouncing; }
  void set_rgb_encoder(bool rgb_encoder) { this->rgb_encoder_ = rgb_encoder; }
  void set_relative_mode(bool relative_mode) { this->relative_mode_ = relative_mode; }
  void set_fade_rgb(uint8_t fade_ms) { this->fade_rgb_ = fade_ms; }
  void set_gamma_rgb(GAMMA_PARAMETER r, GAMMA_PARAMETER g, GAMMA_PARAMETER b) {
    this->gamma_r_ = r;
    this->gamma_g_ = g;
    this->gamma_b_ = b;
  }

  // RGB LED control
  void set_rgb_led_r(uint8_t r);
  void set_rgb_led_g(uint8_t g);
  void set_rgb_led_b(uint8_t b);
  void set_rgb_led(uint8_t r, uint8_t g, uint8_t b);

  // Direct control actions for automation
  void increment();
  void decrement();
  void set_position(float position);

  // Event callbacks registration
  void add_on_button_released_callback(std::function<void()> callback) {
    this->on_button_released_callback_.add(std::move(callback));
  }
  void add_on_button_pressed_callback(std::function<void()> callback) {
    this->on_button_pressed_callback_.add(std::move(callback));
  }
  void add_on_button_double_pressed_callback(std::function<void()> callback) {
    this->on_button_double_pressed_callback_.add(std::move(callback));
  }
  void add_on_increment_callback(std::function<void()> callback) {
    this->on_increment_callback_.add(std::move(callback));
  }
  void add_on_decrement_callback(std::function<void()> callback) {
    this->on_decrement_callback_.add(std::move(callback));
  }
  void add_on_min_callback(std::function<void()> callback) { this->on_min_callback_.add(std::move(callback)); }
  void add_on_max_callback(std::function<void()> callback) { this->on_max_callback_.add(std::move(callback)); }

  // Read methods
  bool update_status();
  float read_counter_float();
  int32_t read_counter_long();
  uint8_t read_button_status();

  // Write methods
  void write_counter(int32_t counter);
  void write_counter(float counter);
  void write_max(int32_t max);
  void write_max(float max);
  void write_min(int32_t min);
  void write_min(float min);
  void write_step(int32_t step);
  void write_step(float step);
  void write_interrupt_config(uint8_t interrupt);
  void auto_config_interrupt();

 protected:
  // Entities
  sensor::Sensor *position_sensor_{nullptr};
  binary_sensor::BinarySensor *button_sensor_{nullptr};
  GPIOPin *interrupt_pin_{nullptr};

  // Configuration options
  float min_value_{0};
  float max_value_{100};
  float step_value_{1};
  float initial_position_{0};
  bool use_float_{false};
  bool wrap_enabled_{true};
  bool direction_left_{false};
  bool double_push_enabled_{false};
  uint8_t double_push_period_{0};
  uint8_t anti_bouncing_{1};
  bool rgb_encoder_{false};
  bool relative_mode_{false};
  uint8_t fade_rgb_{0};
  GAMMA_PARAMETER gamma_r_{GAMMA_OFF};
  GAMMA_PARAMETER gamma_g_{GAMMA_OFF};
  GAMMA_PARAMETER gamma_b_{GAMMA_OFF};

  // Status registers
  uint8_t status_{0};
  uint8_t status2_{0};
  uint16_t gconf_{0};

  // Update timings
  uint32_t last_update_time_{0};
  static constexpr uint32_t UPDATE_INTERVAL_MS = 50;

  // Low level I2C functions
  uint8_t read_encoder_byte(uint8_t reg);
  int16_t read_encoder_int(uint8_t reg);
  int32_t read_encoder_long(uint8_t reg);
  float read_encoder_float(uint8_t reg);
  void write_encoder_byte(uint8_t reg, uint8_t data);
  void write_encoder_long(uint8_t reg, int32_t data);
  void write_encoder_float(uint8_t reg, float data);
  void write_encoder_rgb(uint8_t reg, uint32_t rgb);

  // Callback collections
  CallbackManager<void()> on_button_released_callback_;
  CallbackManager<void()> on_button_pressed_callback_;
  CallbackManager<void()> on_button_double_pressed_callback_;
  CallbackManager<void()> on_increment_callback_;
  CallbackManager<void()> on_decrement_callback_;
  CallbackManager<void()> on_min_callback_;
  CallbackManager<void()> on_max_callback_;
};

// RGB Light Output class for ESPHome integration
class I2CEncoderV2LightOutput : public light::LightOutput {
 public:
  void set_parent(I2CEncoderV2Component *parent) { this->parent_ = parent; }

  light::LightTraits get_traits() override {
    auto traits = light::LightTraits();
    traits.set_supported_color_modes({light::ColorMode::RGB});
    return traits;
  }

  void write_state(light::LightState *state) override;

 protected:
  I2CEncoderV2Component *parent_{nullptr};
};

// Triggers for various encoder events
class I2CEncoderButtonReleasedTrigger : public Trigger<> {
 public:
  explicit I2CEncoderButtonReleasedTrigger(I2CEncoderV2Component *parent) {
    parent->add_on_button_released_callback([this]() { this->trigger(); });
  }
};

class I2CEncoderButtonPressedTrigger : public Trigger<> {
 public:
  explicit I2CEncoderButtonPressedTrigger(I2CEncoderV2Component *parent) {
    parent->add_on_button_pressed_callback([this]() { this->trigger(); });
  }
};

class I2CEncoderButtonDoublePressedTrigger : public Trigger<> {
 public:
  explicit I2CEncoderButtonDoublePressedTrigger(I2CEncoderV2Component *parent) {
    parent->add_on_button_double_pressed_callback([this]() { this->trigger(); });
  }
};

class I2CEncoderIncrementedTrigger : public Trigger<> {
 public:
  explicit I2CEncoderIncrementedTrigger(I2CEncoderV2Component *parent) {
    parent->add_on_increment_callback([this]() { this->trigger(); });
  }
};

class I2CEncoderDecrementedTrigger : public Trigger<> {
 public:
  explicit I2CEncoderDecrementedTrigger(I2CEncoderV2Component *parent) {
    parent->add_on_decrement_callback([this]() { this->trigger(); });
  }
};

class I2CEncoderMinReachedTrigger : public Trigger<> {
 public:
  explicit I2CEncoderMinReachedTrigger(I2CEncoderV2Component *parent) {
    parent->add_on_min_callback([this]() { this->trigger(); });
  }
};

class I2CEncoderMaxReachedTrigger : public Trigger<> {
 public:
  explicit I2CEncoderMaxReachedTrigger(I2CEncoderV2Component *parent) {
    parent->add_on_max_callback([this]() { this->trigger(); });
  }
};

}  // namespace i2c_encoder_v2
}  // namespace esphome
