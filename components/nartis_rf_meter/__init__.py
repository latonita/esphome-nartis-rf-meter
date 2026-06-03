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
CONF_PIN_GPIO3 = "pin_gpio3"
CONF_METER_SERIAL = "meter_serial"
CONF_CIU_SERIAL = "ciu_serial"
CONF_CIU_ADDRESS = "ciu_address"
CONF_SNIFF_MODE = "sniff_mode"
CONF_SNIFF_CHANNEL = "sniff_channel"

nartis_rf_meter_ns = cg.esphome_ns.namespace("nartis_rf_meter")
NartisRfMeterComponent = nartis_rf_meter_ns.class_(
    "NartisRfMeterComponent", cg.PollingComponent
)


def validate_meter_serial(value):
    """Meter serial is the 12-digit number printed on the meter nameplate."""
    s = cv.string_strict(value)
    if not s.isdigit() or len(s) != 12:
        raise cv.Invalid(
            f"meter_serial must be exactly 12 digits (as printed on the meter "
            f"nameplate, e.g. '012345678901'); got {len(s)} characters '{s}'"
        )
    return s


def validate_ciu_address(value):
    """Full 8-byte CIU RF address as 16 hex chars, e.g. 'CD2C0000026B5025'."""
    s = cv.string_strict(value).replace(" ", "").replace(":", "")
    if len(s) != 16 or any(c not in "0123456789abcdefABCDEF" for c in s):
        raise cv.Invalid(
            "ciu_address must be 8 bytes as 16 hex chars "
            "(e.g. 'CD2C0000026B5025')"
        )
    return s.upper()


def _require_meter_serial_unless_sniff(config):
    """meter_serial is mandatory for active polling, but not for passive sniffing."""
    if not config.get(CONF_SNIFF_MODE) and CONF_METER_SERIAL not in config:
        raise cv.Invalid(
            f"'{CONF_METER_SERIAL}' is required unless '{CONF_SNIFF_MODE}' is enabled"
        )
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(NartisRfMeterComponent),
            # CMT2300A bit-bang SPI pins
            cv.Required(CONF_PIN_SDIO): pins.internal_gpio_output_pin_schema,
            cv.Required(CONF_PIN_SCLK): pins.internal_gpio_output_pin_schema,
            cv.Required(CONF_PIN_CSB): pins.internal_gpio_output_pin_schema,
            cv.Required(CONF_PIN_FCSB): pins.internal_gpio_output_pin_schema,
            # CMT2300A interrupt pin (input, active high). The firmware drives
            # the same RX/TX interrupt onto NIRQ (chip GPIO1/GPIO2, via INT1)
            # AND onto the GPIO3 pad (via INT2), so wire whichever your module
            # exposes and declare the matching ESP pin here. Provide exactly
            # one of pin_gpio1 (NIRQ) or pin_gpio3 (GPIO3 pad).
            cv.Optional(CONF_PIN_GPIO1): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_PIN_GPIO3): pins.internal_gpio_input_pin_schema,
            # Meter serial number (12 digits, printed on the meter nameplate).
            # Required for active polling; optional in sniff_mode (see validator below).
            cv.Optional(CONF_METER_SERIAL): validate_meter_serial,
            # Passive sniff mode: never transmit — just park the radio in RX and
            # log every received frame as raw hex (no CRC/address/AES decoding).
            cv.Optional(CONF_SNIFF_MODE, default=False): cv.boolean,
            # Frequency bank (0..3) to camp on while sniffing. The live CIU↔meter
            # link runs on channel 0 (RX 434.1026 MHz), so 0 is the default.
            cv.Optional(CONF_SNIFF_CHANNEL, default=0): cv.int_range(min=0, max=3),
            # CIU serial — for replacing an existing CIU unit;
            # if omitted, ESP32 MAC address is used (fresh pairing).
            cv.Optional(CONF_CIU_SERIAL, default=""): cv.string_strict,
            # Full CIU RF address override (16 hex chars). Use to impersonate the
            # exact CIU a meter is already paired with — meters typically only
            # answer their paired CIU's address. Overrides ciu_serial/MAC.
            cv.Optional(CONF_CIU_ADDRESS): validate_ciu_address,
        }
    ).extend(cv.polling_component_schema("60s")),
    # Exactly one interrupt pin must be wired/declared.
    cv.has_exactly_one_key(CONF_PIN_GPIO1, CONF_PIN_GPIO3),
    _require_meter_serial_unless_sniff,
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
    # Interrupt pin: NIRQ (pin_gpio1) or the GPIO3 pad (pin_gpio3) — whichever
    # is declared. Both carry the same INT signal; the HAL polls this one pin.
    irq_conf = config.get(CONF_PIN_GPIO1, config.get(CONF_PIN_GPIO3))
    irq = await cg.gpio_pin_expression(irq_conf)
    cg.add(var.set_pin_gpio1(irq))

    if config[CONF_SNIFF_MODE]:
        cg.add(var.set_sniff_mode(True))
        cg.add(var.set_sniff_channel(config[CONF_SNIFF_CHANNEL]))

    if CONF_METER_SERIAL in config:
        cg.add(var.set_meter_serial(config[CONF_METER_SERIAL]))
    if config[CONF_CIU_SERIAL]:
        cg.add(var.set_ciu_serial(config[CONF_CIU_SERIAL]))
    if CONF_CIU_ADDRESS in config:
        cg.add(var.set_ciu_address(config[CONF_CIU_ADDRESS]))
