"""Nartis RF Meter — ESPHome external component for Nartis meters via CMT2300A RF433."""

from esphome import pins
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@latonita"]
AUTO_LOAD = ["sensor", "text_sensor"]
MULTI_CONF = True

CONF_PIN_SDIO = "pin_sdio"
CONF_PIN_SCLK = "pin_sclk"
CONF_PIN_CSB = "pin_csb"
CONF_PIN_FCSB = "pin_fcsb"
CONF_PIN_GPIO1 = "pin_gpio1"
CONF_METER_SERIAL = "meter_serial"
CONF_CIU_SERIAL = "ciu_serial"
CONF_AES_KEY = "aes_key"

nartis_rf_meter_ns = cg.esphome_ns.namespace("nartis_rf_meter")
NartisRfMeterComponent = nartis_rf_meter_ns.class_(
    "NartisRfMeterComponent", cg.PollingComponent
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(NartisRfMeterComponent),
            # CMT2300A bit-bang SPI pins
            cv.Required(CONF_PIN_SDIO): pins.internal_gpio_output_pin_schema,
            cv.Required(CONF_PIN_SCLK): pins.internal_gpio_output_pin_schema,
            cv.Required(CONF_PIN_CSB): pins.internal_gpio_output_pin_schema,
            cv.Required(CONF_PIN_FCSB): pins.internal_gpio_output_pin_schema,
            # CMT2300A interrupt pin (input, active high)
            cv.Required(CONF_PIN_GPIO1): pins.internal_gpio_input_pin_schema,
            # Meter serial number (from meter nameplate)
            cv.Required(CONF_METER_SERIAL): cv.string_strict,
            # CIU serial — for replacing an existing CIU unit;
            # if omitted, ESP32 MAC address is used (fresh pairing)
            cv.Optional(CONF_CIU_SERIAL, default=""): cv.string_strict,
            # AES key — factory default from firmware, override only if meter has custom key
            cv.Optional(CONF_AES_KEY, default="ZCZfuT666iRdgPNH"): cv.string_strict,
        }
    )
    .extend(cv.polling_component_schema("60s"))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Pins
    sdio = await cg.gpio_pin_expression(config[CONF_PIN_SDIO])
    cg.add(var.set_pin_sdio(sdio))
    sclk = await cg.gpio_pin_expression(config[CONF_PIN_SCLK])
    cg.add(var.set_pin_sclk(sclk))
    csb = await cg.gpio_pin_expression(config[CONF_PIN_CSB])
    cg.add(var.set_pin_csb(csb))
    fcsb = await cg.gpio_pin_expression(config[CONF_PIN_FCSB])
    cg.add(var.set_pin_fcsb(fcsb))
    gpio1 = await cg.gpio_pin_expression(config[CONF_PIN_GPIO1])
    cg.add(var.set_pin_gpio1(gpio1))

    # Meter identification
    cg.add(var.set_meter_serial(config[CONF_METER_SERIAL]))
    if config[CONF_CIU_SERIAL]:
        cg.add(var.set_ciu_serial(config[CONF_CIU_SERIAL]))

    # AES key
    cg.add(var.set_aes_key(config[CONF_AES_KEY]))
