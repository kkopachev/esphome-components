import esphome.codegen as cg
from esphome.components import i2c, ssd1306_base
from esphome.components.ssd1306_base import _validate
from esphome.components.ssd1306_i2c import display
import esphome.config_validation as cv
from esphome.const import CONF_ID

# We depend on the ssd1306_i2c component
DEPENDENCIES = ["ssd1306_i2c"]
AUTO_LOAD = ["ssd1306_i2c"]

# Add our custom width config parameter
CONF_CUSTOM_WIDTH = "custom_width"

# Define our namespace and component class
custom_width_ssd1306_i2c_ns = cg.esphome_ns.namespace("custom_width_ssd1306_i2c")
CustomWidthSSD1306 = custom_width_ssd1306_i2c_ns.class_(
    "CustomWidthSSD1306", display.I2CSSD1306
)

# Extend the existing SSD1306 I2C schema with our custom width parameter
CONFIG_SCHEMA = cv.All(
    ssd1306_base.SSD1306_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(CustomWidthSSD1306),
            cv.Required(CONF_CUSTOM_WIDTH): cv.int_range(min=1, max=255),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x3C)),
    cv.has_at_most_one_key(display.CONF_PAGES, display.CONF_LAMBDA),
    _validate,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await ssd1306_base.setup_ssd1306(var, config)
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_custom_width(config[CONF_CUSTOM_WIDTH]))
