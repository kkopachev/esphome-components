from esphome import automation  # , pins
import esphome.codegen as cg
from esphome.components import binary_sensor, i2c, light, sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_MAX_VALUE,
    CONF_MIN_VALUE,
    CONF_OUTPUT_ID,
    CONF_STEP,
    DEVICE_CLASS_EMPTY,
    STATE_CLASS_MEASUREMENT,
    UNIT_EMPTY,
)

DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["sensor", "light", "binary_sensor"]
MULTI_CONF = True

CONF_TRIGGER_ID = "trigger_id"

# Component namespace
i2c_encoder_v2_ns = cg.esphome_ns.namespace("i2c_encoder_v2")
I2CEncoderV2Component = i2c_encoder_v2_ns.class_(
    "I2CEncoderV2Component", cg.Component, i2c.I2CDevice
)
I2CEncoderV2LightOutput = i2c_encoder_v2_ns.class_(
    "I2CEncoderV2LightOutput", light.LightOutput
)

# Automation action types
I2CEncoderSetPositionAction = i2c_encoder_v2_ns.class_(
    "I2CEncoderSetPositionAction", automation.Action.template()
)
I2CEncoderIncrementAction = i2c_encoder_v2_ns.class_(
    "I2CEncoderIncrementAction", automation.Action.template()
)
I2CEncoderDecrementAction = i2c_encoder_v2_ns.class_(
    "I2CEncoderDecrementAction", automation.Action.template()
)

# Triggers
I2CEncoderButtonReleasedTrigger = i2c_encoder_v2_ns.class_(
    "I2CEncoderButtonReleasedTrigger", automation.Trigger.template()
)
I2CEncoderButtonPressedTrigger = i2c_encoder_v2_ns.class_(
    "I2CEncoderButtonPressedTrigger", automation.Trigger.template()
)
I2CEncoderButtonDoublePressedTrigger = i2c_encoder_v2_ns.class_(
    "I2CEncoderButtonDoublePressedTrigger", automation.Trigger.template()
)
I2CEncoderIncrementedTrigger = i2c_encoder_v2_ns.class_(
    "I2CEncoderIncrementedTrigger", automation.Trigger.template()
)
I2CEncoderDecrementedTrigger = i2c_encoder_v2_ns.class_(
    "I2CEncoderDecrementedTrigger", automation.Trigger.template()
)
I2CEncoderMinReachedTrigger = i2c_encoder_v2_ns.class_(
    "I2CEncoderMinReachedTrigger", automation.Trigger.template()
)
I2CEncoderMaxReachedTrigger = i2c_encoder_v2_ns.class_(
    "I2CEncoderMaxReachedTrigger", automation.Trigger.template()
)

# Configuration constants
CONF_POSITION = "position"
CONF_BUTTON = "button"
CONF_RGB_LED = "rgb_led"
CONF_USE_FLOAT = "use_float"
CONF_WRAP_ENABLED = "wrap_enabled"
CONF_DIRECTION_LEFT = "direction_left"
CONF_DOUBLE_PUSH_ENABLED = "double_push_enabled"
CONF_DOUBLE_PUSH_PERIOD = "double_push_period"
CONF_ANTI_BOUNCING = "anti_bouncing"
CONF_RGB_ENCODER = "rgb_encoder"
CONF_RELATIVE_MODE = "relative_mode"
CONF_INTERRUPT_PIN = "interrupt_pin"
CONF_INITIAL_POSITION = "initial_position"
CONF_FADE_RGB = "fade_rgb"
CONF_GAMMA_RGB = "gamma_rgb"

# Triggers configuration
CONF_ON_BUTTON_RELEASED = "on_button_released"
CONF_ON_BUTTON_PRESSED = "on_button_pressed"
CONF_ON_BUTTON_DOUBLE_PRESSED = "on_button_double_pressed"
CONF_ON_INCREMENTED = "on_incremented"
CONF_ON_DECREMENTED = "on_decremented"
CONF_ON_MIN_REACHED = "on_min_reached"
CONF_ON_MAX_REACHED = "on_max_reached"

# RGB Gamma config
GAMMA_CONFIG_SCHEMA = cv.Schema(
    {
        cv.Optional("r", default=0): cv.int_range(min=0, max=7),
        cv.Optional("g", default=0): cv.int_range(min=0, max=7),
        cv.Optional("b", default=0): cv.int_range(min=0, max=7),
    }
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(I2CEncoderV2Component),
        cv.Optional(CONF_POSITION): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_EMPTY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_BUTTON): binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_RGB_LED): light.RGB_LIGHT_SCHEMA.extend(
            {
                cv.GenerateID(CONF_OUTPUT_ID): cv.declare_id(I2CEncoderV2LightOutput),
            }
        ),
        cv.Optional(CONF_MIN_VALUE, default=0): cv.float_,
        cv.Optional(CONF_MAX_VALUE, default=100): cv.float_,
        cv.Optional(CONF_STEP, default=1): cv.float_,
        cv.Optional(CONF_INITIAL_POSITION, default=0): cv.float_,
        cv.Optional(CONF_USE_FLOAT, default=False): cv.boolean,
        cv.Optional(CONF_WRAP_ENABLED, default=True): cv.boolean,
        cv.Optional(CONF_DIRECTION_LEFT, default=False): cv.boolean,
        cv.Optional(CONF_DOUBLE_PUSH_ENABLED, default=False): cv.boolean,
        cv.Optional(CONF_DOUBLE_PUSH_PERIOD, default=0): cv.int_range(min=0, max=255),
        cv.Optional(CONF_ANTI_BOUNCING, default=1): cv.int_range(min=1, max=255),
        cv.Optional(CONF_RGB_ENCODER, default=False): cv.boolean,
        cv.Optional(CONF_RELATIVE_MODE, default=False): cv.boolean,
        # cv.Optional(CONF_INTERRUPT_PIN): pins.gpio_input_pin_schema,
        cv.Optional(CONF_FADE_RGB, default=0): cv.int_range(min=0, max=255),
        cv.Optional(CONF_GAMMA_RGB): GAMMA_CONFIG_SCHEMA,
        # Triggers
        cv.Optional(CONF_ON_BUTTON_RELEASED): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                    I2CEncoderButtonReleasedTrigger
                ),
            }
        ),
        cv.Optional(CONF_ON_BUTTON_PRESSED): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                    I2CEncoderButtonPressedTrigger
                ),
            }
        ),
        cv.Optional(CONF_ON_BUTTON_DOUBLE_PRESSED): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                    I2CEncoderButtonDoublePressedTrigger
                ),
            }
        ),
        cv.Optional(CONF_ON_INCREMENTED): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                    I2CEncoderIncrementedTrigger
                ),
            }
        ),
        cv.Optional(CONF_ON_DECREMENTED): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                    I2CEncoderDecrementedTrigger
                ),
            }
        ),
        cv.Optional(CONF_ON_MIN_REACHED): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                    I2CEncoderMinReachedTrigger
                ),
            }
        ),
        cv.Optional(CONF_ON_MAX_REACHED): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                    I2CEncoderMaxReachedTrigger
                ),
            }
        ),
    }
).extend(i2c.i2c_device_schema(None))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    # Configure parameters
    cg.add(var.set_min_value(config[CONF_MIN_VALUE]))
    cg.add(var.set_max_value(config[CONF_MAX_VALUE]))
    cg.add(var.set_step_value(config[CONF_STEP]))
    cg.add(var.set_initial_position(config[CONF_INITIAL_POSITION]))
    cg.add(var.set_use_float(config[CONF_USE_FLOAT]))
    cg.add(var.set_wrap_enabled(config[CONF_WRAP_ENABLED]))
    cg.add(var.set_direction_left(config[CONF_DIRECTION_LEFT]))
    cg.add(var.set_double_push_enabled(config[CONF_DOUBLE_PUSH_ENABLED]))
    cg.add(var.set_double_push_period(config[CONF_DOUBLE_PUSH_PERIOD]))
    cg.add(var.set_anti_bouncing(config[CONF_ANTI_BOUNCING]))
    cg.add(var.set_rgb_encoder(config[CONF_RGB_ENCODER]))
    cg.add(var.set_relative_mode(config[CONF_RELATIVE_MODE]))
    cg.add(var.set_fade_rgb(config[CONF_FADE_RGB]))

    if CONF_GAMMA_RGB in config:
        gamma = config[CONF_GAMMA_RGB]
        cg.add(var.set_gamma_rgb(gamma["r"], gamma["g"], gamma["b"]))

    # if CONF_INTERRUPT_PIN in config:
    #    pin = await cg.gpio_pin_expression(config[CONF_INTERRUPT_PIN])
    #    cg.add(var.set_interrupt_pin(pin))

    # Register entities
    if CONF_POSITION in config:
        sens = await sensor.new_sensor(config[CONF_POSITION])
        cg.add(var.set_position_sensor(sens))

    if CONF_BUTTON in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_BUTTON])
        cg.add(var.set_button_sensor(sens))

    if CONF_RGB_LED in config:
        conf = config[CONF_RGB_LED]
        light_var = cg.new_Pvariable(conf[CONF_OUTPUT_ID])
        await light.register_light(light_var, conf)
        cg.add(light_var.set_parent(var))

    # Register triggers
    if CONF_ON_BUTTON_RELEASED in config:
        for conf in config[CONF_ON_BUTTON_RELEASED]:
            trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
            await automation.build_automation(trigger, [], conf)

    if CONF_ON_BUTTON_PRESSED in config:
        for conf in config[CONF_ON_BUTTON_PRESSED]:
            trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
            await automation.build_automation(trigger, [], conf)

    if CONF_ON_BUTTON_DOUBLE_PRESSED in config:
        for conf in config[CONF_ON_BUTTON_DOUBLE_PRESSED]:
            trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
            await automation.build_automation(trigger, [], conf)

    if CONF_ON_INCREMENTED in config:
        for conf in config[CONF_ON_INCREMENTED]:
            trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
            await automation.build_automation(trigger, [], conf)

    if CONF_ON_DECREMENTED in config:
        for conf in config[CONF_ON_DECREMENTED]:
            trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
            await automation.build_automation(trigger, [], conf)

    if CONF_ON_MIN_REACHED in config:
        for conf in config[CONF_ON_MIN_REACHED]:
            trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
            await automation.build_automation(trigger, [], conf)

    if CONF_ON_MAX_REACHED in config:
        for conf in config[CONF_ON_MAX_REACHED]:
            trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
            await automation.build_automation(trigger, [], conf)


@automation.register_action(
    "i2c_encoder_v2_set_position",
    I2CEncoderSetPositionAction,
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(I2CEncoderV2Component),
            cv.Required("position"): cv.templatable(cv.float_),
        }
    ),
)
async def i2c_encoder_v2_set_position_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, parent)

    template_ = await cg.templatable(config["position"], args, float)
    cg.add(var.set_position(template_))
    return var


@automation.register_action(
    "i2c_encoder_v2_increment",
    I2CEncoderIncrementAction,
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(I2CEncoderV2Component),
        }
    ),
)
async def i2c_encoder_v2_increment_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, parent)
    return var


@automation.register_action(
    "i2c_encoder_v2_decrement",
    I2CEncoderDecrementAction,
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(I2CEncoderV2Component),
        }
    ),
)
async def i2c_encoder_v2_decrement_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, parent)
    return var
