import esphome.codegen as cg
import esphome.components import ssd1306_i2c
import esphome.config_validation as cv
from esphome.const import CONF_ID

DEPENDENCIES = ["ssd1306_i2c"]
AUTO_LOAD = ["ssd1306_i2c"]

custom_width_ssd1306_i2c_ns = cg.esphome_ns.namespace("custom_width_ssd1306_i2c")
CustomWidthSH1106 = custom_width_ssd1306_i2c_ns.class_(
    "CustomWidthSH1106", ssd1306_i2c.I2CSSD1306
)

CONF_CUSTOM_WIDTH = "custom_width"

# Simply extend the existing SSD1306 I2C config with our custom width parameter
CONFIG_SCHEMA = ssd1306_i2c.SSD1306_I2C_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(CustomWidthSH1106),
        cv.Required(CONF_CUSTOM_WIDTH): cv.int_range(min=1, max=255),
    }
)


async def to_code(config):
    # First use the parent component's to_code to set up everything standard
    await ssd1306_i2c.setup_ssd1306_i2c(config)

    # Then set our custom width
    var = cg.new_Pvariable(config[CONF_ID])
    cg.add(var.set_custom_width(config[CONF_CUSTOM_WIDTH]))
