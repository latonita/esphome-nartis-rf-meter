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
CONF_PIN_GPIO3 = "pin_gpio3"
CONF_METER_SERIAL = "meter_serial"
CONF_CIU_SERIAL = "ciu_serial"
CONF_CIU_ADDRESS = "ciu_address"
CONF_FIX_CHANNEL = "fix_channel"
CONF_USE_NON_STANDARD_CHANNELS = "use_non_standard_channels"
CONF_INITIAL_TIME = "initial_time"
CONF_BATCH_SIZE = "batch_size"
CONF_RX_TIMEOUT = "rx_timeout"
CONF_RX_REPLY_TIMEOUT = "rx_reply_timeout"


def validate_initial_time(value):
    """Default wall-clock to seed the system clock with at boot (no NTP/WiFi).
    Accepts 'YYYY-MM-DD HH:MM:SS' (interpreted as UTC) and stores epoch seconds.
    The meter requires a live, advancing clock in the beacon before it delivers
    data; this seeds one so the timestamp ticks even without a network time source."""
    import datetime

    s = cv.string_strict(value)
    try:
        dt = datetime.datetime.strptime(s, "%Y-%m-%d %H:%M:%S")
    except ValueError as err:
        raise cv.Invalid(
            f"initial_time must be 'YYYY-MM-DD HH:MM:SS' (UTC); got '{s}'"
        ) from err
    return int(dt.replace(tzinfo=datetime.timezone.utc).timestamp())

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


CONFIG_SCHEMA = cv.Schema(
    {
            cv.GenerateID(): cv.declare_id(NartisRfMeterComponent),
            # CMT2300A bit-bang SPI pins
            cv.Required(CONF_PIN_SDIO): pins.internal_gpio_output_pin_schema,
            cv.Required(CONF_PIN_SCLK): pins.internal_gpio_output_pin_schema,
            cv.Required(CONF_PIN_CSB): pins.internal_gpio_output_pin_schema,
            cv.Required(CONF_PIN_FCSB): pins.internal_gpio_output_pin_schema,
            cv.Required(CONF_PIN_GPIO3): pins.internal_gpio_input_pin_schema,

            cv.Required(CONF_METER_SERIAL): validate_meter_serial,

            cv.Optional(CONF_FIX_CHANNEL): cv.int_range(min=0, max=3),

            # Use the non-standard ("invented") frequency presets placed on the
            # meter's observed reply frequencies (RX 434.30/433.70/434.00/434.55,
            # TX held at 433.82) instead of the firmware channel grid. The firmware
            # RX presets are 97..278 kHz off where this meter actually replies and
            # miss it; these centres + 100 kHz BW + AFC capture it. Default: off.
            cv.Optional(CONF_USE_NON_STANDARD_CHANNELS, default=False): cv.boolean,
            # Default wall-clock (UTC) to seed the system clock with at boot when
            # there's no NTP/RTC source. The beacon carries a live clock the meter
            # checks for freshness — without an advancing clock it refuses data.
            cv.Optional(CONF_INITIAL_TIME): validate_initial_time,
            # CIU serial — for replacing an existing CIU unit;
            # if omitted, ESP32 MAC address is used (fresh pairing).
            cv.Optional(CONF_CIU_SERIAL, default=""): cv.string_strict,
            # Full CIU RF address override (16 hex chars). Use to impersonate the
            # exact CIU a meter is already paired with — meters typically only
            # answer their paired CIU's address. Overrides ciu_serial/MAC.
            cv.Optional(CONF_CIU_ADDRESS): validate_ciu_address,
            # --- Tuning (optional; sensible defaults — leave unset for normal use) ---
            # How many user-defined OBIS attributes to read per get-request-with-list.
            # Larger = fewer poll cycles but bigger 0x43 replies (DLMS caps at 10/list).
            cv.Optional(CONF_BATCH_SIZE, default=1): cv.int_range(min=1, max=10),
            # General RX wait before "no response": governs pairing waits and frame
            # completion once a reply has started.
            cv.Optional(
                CONF_RX_TIMEOUT, default="3000ms"
            ): cv.positive_time_period_milliseconds,
            # Short RX wait for the steady-state GET reply (applied only until the
            # first byte arrives). A request dropped in the meter's post-TX deaf
            # window re-sends after this instead of stalling the full rx_timeout.
            cv.Optional(
                CONF_RX_REPLY_TIMEOUT, default="900ms"
            ): cv.positive_time_period_milliseconds,
        }
).extend(cv.polling_component_schema("300s"))


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
    # Interrupt pin: the chip's GPIO3 pad (INT2). The HAL polls this pin for the
    # FIFO-threshold signal that drives RX/TX FIFO draining.
    irq = await cg.gpio_pin_expression(config[CONF_PIN_GPIO3])
    cg.add(var.set_pin_gpio3(irq))

    if CONF_FIX_CHANNEL in config:
        cg.add(var.set_fix_channel(config[CONF_FIX_CHANNEL]))

    cg.add(var.set_use_non_standard_channels(config[CONF_USE_NON_STANDARD_CHANNELS]))

    if CONF_INITIAL_TIME in config:
        cg.add(var.set_initial_epoch(config[CONF_INITIAL_TIME]))

    cg.add(var.set_batch_size(config[CONF_BATCH_SIZE]))
    cg.add(var.set_rx_timeout_ms(config[CONF_RX_TIMEOUT]))
    cg.add(var.set_rx_reply_timeout_ms(config[CONF_RX_REPLY_TIMEOUT]))

    cg.add(var.set_meter_serial(config[CONF_METER_SERIAL]))
    if config[CONF_CIU_SERIAL]:
        cg.add(var.set_ciu_serial(config[CONF_CIU_SERIAL]))
    if CONF_CIU_ADDRESS in config:
        cg.add(var.set_ciu_address(config[CONF_CIU_ADDRESS]))
