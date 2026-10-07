"""Nartis RF Meter - ESPHome external component for Nartis I100/I300 meters over RF433-1.

The component emulates a Nartis D101 remote display: it pairs with the meter and
then polls it over DLMS/COSEM, carried inside a proprietary wM-Bus-flavoured RF
envelope on a 433 MHz link.

The radio is a replaceable backend, picked with the `radio:` block:

    cmt2300a   CMOSTEK CMT2300A on a bit-banged 3-wire SPI. The original, proven
               against live meters.
    cc1101     TI CC1101 on a normal 4-wire SPI. Same PHY, different silicon;
               new, NOT yet confirmed on hardware, and ~4 dB down on TX power.

Everything above the radio - the length-prefixed RF envelope, the software
CRC-16/DNP, AES-128-GCM and the DLMS payload - is radio independent, so the two
backends differ only in how the bytes reach the antenna.
"""

import logging

from esphome import pins
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_TYPE

_LOGGER = logging.getLogger(__name__)

CODEOWNERS = ["@latonita"]
AUTO_LOAD = ["sensor", "text_sensor", "button"]
MULTI_CONF = True

# --- Configuration keys -----------------------------------------------------
CONF_RADIO = "radio"

RADIO_TYPE_CMT2300A = "cmt2300a"
RADIO_TYPE_CC1101 = "cc1101"

# CMT2300A. These five names are also the pre-`radio:` flat keys, kept working by
# migrate_legacy_radio() below.
CONF_PIN_SDIO = "pin_sdio"
CONF_PIN_SCLK = "pin_sclk"
CONF_PIN_CSB = "pin_csb"
CONF_PIN_FCSB = "pin_fcsb"
CONF_PIN_GPIO3 = "pin_gpio3"

# CC1101. pin_sclk is shared with the CMT2300A spelling on purpose - it is the
# same wire on the same bus, whichever chip is on the far end.
CONF_PIN_MISO = "pin_miso"
CONF_PIN_MOSI = "pin_mosi"
CONF_PIN_CS = "pin_cs"
CONF_PIN_GDO0 = "pin_gdo0"
CONF_RX_OFFSET = "rx_offset"
CONF_OUTPUT_POWER = "output_power"
CONF_BANDWIDTH = "bandwidth"
CONF_DEVIATION = "deviation"
CONF_MODULATION = "modulation"
# Whether the CC1101's frequency-offset and bit-sync loops are gated on carrier
# sense (FOCCFG 0x36) or run freely (0x16). A bring-up knob; see the long note at
# the schema entry for why neither value is obviously right here.
CONF_FOC_CARRIER_SENSE_GATE = "foc_carrier_sense_gate"
# The 16 bits the CC1101 correlator matches, high byte on the air first. Exposed
# because this PHY's sync byte order is derived rather than SDR-confirmed - see
# validate_sync_word().
CONF_SYNC_WORD = "sync_word"

CONF_METER_SERIAL = "meter_serial"
CONF_CIU_SERIAL = "ciu_serial"
CONF_CIU_ADDRESS = "ciu_address"
CONF_FIX_CHANNEL = "fix_channel"
CONF_USE_ALTERNATIVE_CHANNELS = "use_alternative_channels"
CONF_BATCH_SIZE = "batch_size"
CONF_RX_TIMEOUT = "rx_timeout"
CONF_RX_REPLY_TIMEOUT = "rx_reply_timeout"
CONF_SKIP_FIN_BEACONS = "skip_fin_beacons"

nartis_rf_meter_ns = cg.esphome_ns.namespace("nartis_rf_meter")
NartisRfMeterComponent = nartis_rf_meter_ns.class_(
    "NartisRfMeterComponent", cg.PollingComponent
)
RadioHal = nartis_rf_meter_ns.class_("RadioHal")
Cmt2300aHal = nartis_rf_meter_ns.class_("Cmt2300aHal", RadioHal)
Cc1101Hal = nartis_rf_meter_ns.class_("Cc1101Hal", RadioHal)

# Cc1101Hal::Modulation - MDMCFG2.MOD_FORMAT. The two are equivalent on receive;
# this is a transmit-side choice. See set_modulation() in cc1101_hal.h.
Cc1101Modulation = Cc1101Hal.enum("Modulation", is_class=True)
CC1101_MODULATIONS = {
    "2fsk": Cc1101Modulation.FSK2,
    "gfsk": Cc1101Modulation.GFSK,
}


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


# --------------------------------------------------------------------------- #
# radio: - the replaceable RF backend
# --------------------------------------------------------------------------- #

# The CC1101 PA table for the 433 MHz band stops at +10 dBm (the +12 dBm entry in
# the datasheet is a 868/915 MHz figure). A real Nartis D101 display runs +14 dBm,
# so the uplink is roughly 4 dB down no matter what is asked for here; the cure is
# a module with an external PA, not a larger number.
CC1101_POWER_MIN_DBM = -30
CC1101_POWER_MAX_DBM = 10
# What a genuine D101 CIU transmits (RF_PHY_SPEC.md sec. 4.9).
CIU_REFERENCE_POWER_DBM = 14

# Channel filter range the CC1101 can actually synthesise from its 26 MHz crystal:
# 26e6 / (8 * (4 + CHANBW_M) * 2^CHANBW_E), i.e. 58035 Hz .. 812500 Hz.
CC1101_BW_MIN_HZ = 58000
CC1101_BW_MAX_HZ = 812000

# The PHY is +-6 kHz at 2400 bps (SDR-measured; the real CIU's RFPDK profile says
# "Deviation = 6.0 kHz" outright). 6 kHz is the one correct value for `deviation`;
# the range exists only so a different Nartis variant could be tried.
DEVIATION_MIN_HZ = 500
DEVIATION_MAX_HZ = 200000

# The meter may answer a few kHz off where we transmitted, mostly crystal error.
# Beyond a couple of hundred kHz the request would no longer be the same channel.
RX_OFFSET_MAX_HZ = 200000


def validate_output_power(value):
    """TX power in dBm, an integer. The CC1101 cannot be pushed past ~+10 dBm here."""
    dbm = cv.int_(value)
    if not CC1101_POWER_MIN_DBM <= dbm <= CC1101_POWER_MAX_DBM:
        raise cv.Invalid(
            f"output_power is in dBm and the CC1101 PA table for the 433 MHz band "
            f"runs from {CC1101_POWER_MIN_DBM} to +{CC1101_POWER_MAX_DBM} dBm; got "
            f"{dbm}. The chip cannot exceed about +10 dBm in this band - a real "
            f"Nartis D101 display transmits +{CIU_REFERENCE_POWER_DBM} dBm, so if "
            f"the meter does not hear us the answer is a CC1101 module with an "
            f"external PA, not a bigger number."
        )
    return dbm


def _frequency_hz(value):
    """`cv.frequency` with the result pinned to a whole number of Hz.

    `cv.frequency` lets an empty mantissa through its regex and then trips over
    `float("")`, so the ValueError is turned into a sentence here.
    """
    try:
        return int(round(cv.frequency(value)))
    except ValueError as err:
        raise cv.Invalid(
            f"expected a frequency, with or without a unit ('6kHz', '101562Hz', "
            f"'58035'); got '{value}'"
        ) from err


def validate_rx_bandwidth(value):
    """RX channel filter width. Accepts `101562Hz`, `101.5kHz`, or a bare number of Hz."""
    hz = _frequency_hz(value)
    if not CC1101_BW_MIN_HZ <= hz <= CC1101_BW_MAX_HZ:
        raise cv.Invalid(
            f"bandwidth must be between {CC1101_BW_MIN_HZ} Hz and {CC1101_BW_MAX_HZ} "
            f"Hz; got {hz} Hz - that is the whole range the CC1101 channel filter "
            f"can synthesise. Note the default 101562 Hz is NOT sized for the "
            f"signal: the link is +-6 kHz at 2400 bps, which occupies only ~17 kHz, "
            f"so even the chip's 58035 Hz floor would pass it. It is sized for the "
            f"crystal offset, because frequency offset compensation only reaches "
            f"+-bandwidth/4 and a worst-case 20 ppm + 20 ppm pair at 433.8 MHz is "
            f"+-17.4 kHz. Narrowing to 58035 Hz buys ~2.4 dB of sensitivity and "
            f"gives up that margin - worth an A/B test on a strong link, not a "
            f"default."
        )
    return hz


def validate_deviation(value):
    """FSK deviation. 6 kHz is the protocol's value; anything else is an experiment."""
    hz = _frequency_hz(value)
    if not DEVIATION_MIN_HZ <= hz <= DEVIATION_MAX_HZ:
        raise cv.Invalid(
            f"deviation must be between {DEVIATION_MIN_HZ} Hz and {DEVIATION_MAX_HZ} "
            f"Hz; got {hz} Hz. The RF433-1 PHY is +-6 kHz - SDR-measured, and stated "
            f"outright in the real display's RFPDK profile - so `6kHz` is the value "
            f"that works; the chip rounds it to 5950 Hz (DEVIATN = 0x17). Change it "
            f"only to chase a meter that provably does not follow the spec."
        )
    return hz


def validate_rx_offset(value):
    """Signed RX offset in Hz.

    `cv.frequency` has no notion of a negative frequency, so the sign is peeled off
    here and reapplied: `-3kHz`, `+0Hz`, `2500` and `0` all work, and a bare number
    is read as Hz.
    """
    if isinstance(value, str):
        text = value.strip()
        sign = 1
        if text[:1] in ("+", "-"):
            sign = -1 if text[0] == "-" else 1
            text = text[1:].lstrip()
        try:
            hz = int(round(sign * cv.frequency(text)))
        except ValueError as err:
            # cv.frequency's regex matches an empty mantissa, e.g. a bare "-".
            raise cv.Invalid(
                f"rx_offset must be a signed number of Hz, optionally with a unit "
                f"('-3kHz', '0Hz', '2500'); got '{value}'"
            ) from err
    else:
        hz = int(round(cv.float_(value)))
    if abs(hz) > RX_OFFSET_MAX_HZ:
        raise cv.Invalid(
            f"rx_offset is a signed number of Hz and must be within "
            f"+-{RX_OFFSET_MAX_HZ} Hz; got {hz} Hz. It nudges the receiver off the "
            f"channel's reply carrier to chase crystal error - a correction of a few "
            f"kHz, not a channel change. The default is 0 because the chip's own "
            f"frequency offset compensation covers +-25 kHz at the default bandwidth."
        )
    return hz


def validate_sync_word(value):
    """The 16 bits the CC1101 correlator matches: SYNC1 = high byte, SYNC0 = low byte.

    WHY THIS IS A KNOB AT ALL. The RF433-1 sync word is four bytes and the on-air
    order is `55 55 F6 72`. That order is DERIVED, not observed: it follows from the
    CMOSTEK AN143 SYNC_VALUE fill rule (sync bytes fill from the top, so CUS_PKT13 is
    the first byte on air and CUS_PKT10 the last) applied to the real CIU's register
    capture, and it is cross-validated against the hardware-proven RF433-2 driver in
    the sibling projects, which writes PKT13/PKT12/PKT11 in on-air order. It is also
    confirmed exactly by the RFPDK export's own `Sync Value = 0x555555F6`, whose
    register bank is byte-for-byte the capture's default. But no RF433-1 SDR decode
    has read it off the air, and fw/RF_PHY_SPEC.md sec. 11 still lists it as open.

    It CONTRADICTS fw/RF_PHY_SPEC.md sec. 4.4 and fw/FRAME_HEADER_SPEC.md:823, which
    print `72 F6 55 55` - the PKT10..PKT13 register listing read as if it were an air
    sequence. Those two documents should be treated as carrying a documentation bug.

    So the default is 0xF672 (the leading `55 55` merges with the preamble and needs
    no register of its own), and if the meter never answers and nothing is ever
    received, `sync_word: 0x72F6` is the one alternative worth trying - a line of YAML
    instead of a rebuild.

    WHY SIXTEEN BITS AND NOT THIRTY-TWO. RF_PHY_SPEC.md sec. 4.4 offers a 32-bit sync
    word (`0x72F65555`) as "Option A" and sec. 11 lists 16-vs-32 as an open item. The
    CC1101 datasheet closes it regardless of byte order: there is no 32-bit sync
    REGISTER, and MDMCFG2.SYNC_MODE = 011/111 emulates one by REPEATING SYNC1:SYNC0,
    so the correlator would be looking for `F6 72 F6 72` - something the meter never
    transmits. A genuine four-byte sync word is not programmable on this part. With
    the byte order above that costs nothing: the two bytes it could not express are
    `55 55`, and `55 55` is the preamble.
    """
    word = cv.hex_uint16_t(value)
    if word == 0x5555:
        raise cv.Invalid(
            f"`{CONF_SYNC_WORD}: 0x5555` is the preamble pattern, so the correlator "
            f"would fire somewhere inside the preamble and every frame would be cut "
            f"at a random offset. The sync word has to be the part of the header that "
            f"is NOT 0x55 - the default 0xF672 is exactly that."
        )
    return word


CMT2300A_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(Cmt2300aHal),
        cv.Required(CONF_PIN_SDIO): pins.internal_gpio_output_pin_schema,
        cv.Required(CONF_PIN_SCLK): pins.internal_gpio_output_pin_schema,
        cv.Required(CONF_PIN_CSB): pins.internal_gpio_output_pin_schema,
        cv.Required(CONF_PIN_FCSB): pins.internal_gpio_output_pin_schema,
        # The chip's GPIO3 pad (INT2). The HAL polls this pin for the FIFO-threshold
        # signal that drives RX draining - it is the ONLY receive-side signal this
        # board has, because the SPI FIFO-flag registers read 0xFF here, so it is
        # required and a mis-wire is fatal. Cmt2300aHal::self_test() checks it.
        cv.Required(CONF_PIN_GPIO3): pins.internal_gpio_input_pin_schema,
    }
)

CC1101_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(Cc1101Hal),
        cv.Required(CONF_PIN_SCLK): pins.internal_gpio_output_pin_schema,
        cv.Required(CONF_PIN_MISO): pins.internal_gpio_input_pin_schema,
        cv.Required(CONF_PIN_MOSI): pins.internal_gpio_output_pin_schema,
        cv.Required(CONF_PIN_CS): pins.internal_gpio_output_pin_schema,
        # Optional: the backend falls back to polling MARCSTATE/TXBYTES over SPI when
        # GDO0 is not wired. Leaving it out costs a little TX latency, not function -
        # unlike the CMT2300A's pin_gpio3, which is mandatory.
        cv.Optional(CONF_PIN_GDO0): pins.internal_gpio_input_pin_schema,
        cv.Optional(CONF_RX_OFFSET, default=0): validate_rx_offset,
        cv.Optional(CONF_OUTPUT_POWER, default=10): validate_output_power,
        cv.Optional(CONF_BANDWIDTH, default="101562Hz"): validate_rx_bandwidth,
        cv.Optional(CONF_DEVIATION, default="6kHz"): validate_deviation,
        # Transmit waveform shaping. `2fsk` (MOD_FORMAT = 000) is the default because
        # RF_PHY_SPEC.md sec. 5 recommends it and because the on-air signal is
        # SDR-confirmed to demodulate as plain 2-FSK. `gfsk` (MOD_FORMAT = 001,
        # Gaussian BT = 0.5) is what the REAL display transmits - its RFPDK profile
        # says "Demodulation = GFSK, Gaussian BT = 0.5" - and the CC1101's GFSK is
        # BT = 0.5 too, so it reproduces the reference transmitter exactly.
        #
        # This only matters for TRANSMIT: MOD_FORMAT shapes the outgoing waveform and
        # the two demodulate identically, so switching it cannot improve reception.
        # Try `gfsk` if the meter does not hear us - GFSK puts the same power into a
        # narrower spectrum, which helps if the meter's receive filter is tighter
        # than ours.
        cv.Optional(CONF_MODULATION, default="2fsk"): cv.enum(
            CC1101_MODULATIONS, lower=True
        ),
        # Gate the frequency-offset compensation and bit-synchronisation loops on
        # carrier sense. `false` (FOCCFG = 0x16) lets both loops track all the time;
        # `true` (FOCCFG = 0x36) freezes them until carrier sense asserts.
        #
        # The datasheet cannot settle which is right here. Left ungated, the loops
        # spend the whole wait for a reply tracking noise and can wander to their
        # limits (+-25 kHz at the default bandwidth) before the preamble ever
        # arrives - and the waits on this protocol are long, seconds at a time during
        # pairing. Gated, a weak preamble may never trip carrier sense and the reply
        # is dropped whole; the meter sends 10 preamble bytes, 33 ms at 2400 baud, so
        # there is something to spend but not much. The default is `false` because
        # that matches the C++ default and the free-running case at least cannot
        # silence a reply outright.
        #
        # This is a hardware-tuning knob, so decide it by measurement: count
        # CRC-valid replies with each value over many cycles. Bytes landing in the
        # FIFO are not the test - a slipped loop still fills the FIFO, with garbage.
        cv.Optional(CONF_FOC_CARRIER_SENSE_GATE, default=False): cv.boolean,
        # The two bytes SYNC1/SYNC0 hold, high byte first on the air. The default
        # 0xF672 is the F6 72 of the on-air sync word `55 55 F6 72`, whose leading
        # 55 55 is byte-identical to the preamble and merges with it - so a CC1101
        # with NUM_PREAMBLE = 12 puts `55 x12 | F6 72 | LEN ...` on the air, which is
        # byte-identical to a real CIU (10 preamble bytes + the 4-byte sync word).
        #
        # It is settable because that byte order is DERIVED - CMOSTEK AN143's
        # SYNC_VALUE fill rule applied to the CIU register capture, cross-validated on
        # the RF433-2 siblings - and has never been confirmed on air for THIS
        # protocol; fw/RF_PHY_SPEC.md sec. 11 lists it as an open item, and sec. 4.4
        # of that file (and FRAME_HEADER_SPEC.md:823) states the opposite order, which
        # should be read as a documentation bug. If the meter never answers and
        # nothing is ever received, try `sync_word: 0x72F6`. The validator carries the
        # full argument, including why a 32-bit sync word is not an option here.
        cv.Optional(CONF_SYNC_WORD, default=0xF672): validate_sync_word,
    }
)

RADIO_SCHEMA = cv.typed_schema(
    {
        RADIO_TYPE_CMT2300A: CMT2300A_SCHEMA,
        RADIO_TYPE_CC1101: CC1101_SCHEMA,
    },
    key=CONF_TYPE,
    default_type=RADIO_TYPE_CMT2300A,
    lower=True,
)


# The flat keys this component used before the radio became selectable. All five
# pins were required, so the whole set folds into exactly one `radio:` block of
# type cmt2300a.
LEGACY_PIN_KEYS = (
    CONF_PIN_SDIO,
    CONF_PIN_SCLK,
    CONF_PIN_CSB,
    CONF_PIN_FCSB,
    CONF_PIN_GPIO3,
)


def migrate_legacy_radio(config):
    """Fold the pre-`radio:` flat pin keys into an implicit cmt2300a radio block.

    Runs before the schema, so by the time anything else looks at the config there
    is exactly one way to spell a radio. Non-dict input is handed straight through
    for the schema to reject with its usual message.
    """
    if not isinstance(config, dict):
        return config

    legacy = [key for key in LEGACY_PIN_KEYS if key in config]
    has_radio = CONF_RADIO in config

    if legacy and has_radio:
        raise cv.Invalid(
            f"`radio:` and the old flat radio keys ({', '.join(legacy)}) are two "
            f"spellings of the same thing and cannot be combined. Delete the flat "
            f"keys - everything they said now lives inside `radio:`."
        )

    if not legacy:
        if not has_radio:
            raise cv.Invalid(
                f"`radio:` is required. The component no longer owns the radio pins; "
                f"it is handed a radio object. Add a block with "
                f"`type: {RADIO_TYPE_CMT2300A}` (the CMT2300A, proven on hardware) or "
                f"`type: {RADIO_TYPE_CC1101}`, and put the pins inside it."
            )
        return config

    missing = [key for key in LEGACY_PIN_KEYS if key not in config]
    if missing:
        raise cv.Invalid(
            f"the flat CMT2300A pin keys are the pre-`radio:` spelling and all five "
            f"have to be given together; {', '.join(missing)} "
            f"{'is' if len(missing) == 1 else 'are'} missing. Rather than completing "
            f"the set, move the pins into a `radio:` block with "
            f"`type: {RADIO_TYPE_CMT2300A}` - the flat form is deprecated."
        )

    config = config.copy()
    radio = {CONF_TYPE: RADIO_TYPE_CMT2300A}
    for key in LEGACY_PIN_KEYS:
        radio[key] = config.pop(key)
    config[CONF_RADIO] = radio

    _LOGGER.warning(
        "nartis_rf_meter: the flat radio pin keys (%s) are deprecated and will be "
        "removed in a future release. They have been read as a `radio:` block with "
        "`type: %s`. Please rewrite them as:\n"
        "  nartis_rf_meter:\n"
        "    radio:\n"
        "      type: %s\n"
        "%s",
        ", ".join(legacy),
        RADIO_TYPE_CMT2300A,
        RADIO_TYPE_CMT2300A,
        "".join(f"      {key}: ...\n" for key in legacy),
    )
    return config


CONFIG_SCHEMA = cv.All(
    migrate_legacy_radio,
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(NartisRfMeterComponent),
            # The radio and its wiring (see RADIO_SCHEMA above).
            cv.Required(CONF_RADIO): RADIO_SCHEMA,

            cv.Required(CONF_METER_SERIAL): validate_meter_serial,

            cv.Optional(CONF_FIX_CHANNEL): cv.int_range(min=0, max=3),

            # Use the alternative channel mode: transmit always on 433.82 MHz - the
            # frequency the meter wakes on - while the advertised channel index
            # commands the meter to reply on that channel's carrier, where we listen.
            # This is a COMPONENT-level setting, not a radio one: it feeds
            # channel_tx_hz() in radio_hal.h, which is where both backends get the
            # carriers they are told to tune to. Default: off.
            cv.Optional(CONF_USE_ALTERNATIVE_CHANNELS, default=False): cv.boolean,
            # CIU serial - for replacing an existing CIU unit;
            # if omitted, ESP32 MAC address is used (fresh pairing).
            cv.Optional(CONF_CIU_SERIAL, default=""): cv.string_strict,
            # Full CIU RF address override (16 hex chars). Use to impersonate the
            # exact CIU a meter is already paired with - meters typically only
            # answer their paired CIU's address. Overrides ciu_serial/MAC.
            cv.Optional(CONF_CIU_ADDRESS): validate_ciu_address,
            # --- Tuning (optional; sensible defaults - leave unset for normal use) ---
            # How many user-defined OBIS attributes to read per get-request-with-list.
            # Larger = fewer poll cycles but bigger 0x43 replies (DLMS caps at 10/list).
            cv.Optional(CONF_BATCH_SIZE, default=5): cv.int_range(min=1, max=10),
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
            # EXPERIMENT: skip the closing ("FIN") beacon after each GET response
            # and proceed straight to the next batch. Saves ~4.5 s per batch.
            # Default off.
            cv.Optional(CONF_SKIP_FIN_BEACONS, default=False): cv.boolean,
        }
    ).extend(cv.polling_component_schema("300s")),
)


async def radio_to_code(radio_config):
    """Build the RadioHal subclass this config asks for and return the pointer.

    Tunables that carry a schema default matching the C++ default are emitted
    unconditionally - the generated code then says plainly what the radio is running
    with, instead of leaving it implicit in the header.

    `set_pins()` is NOT part of RadioHal - the bus shape is each chip's own business
    - so it is called on the concrete type, which is what cv.declare_id() in each
    branch's schema has already made the generated variable.
    """
    hal = cg.new_Pvariable(radio_config[CONF_ID])

    if radio_config[CONF_TYPE] == RADIO_TYPE_CMT2300A:
        pin_exprs = [
            await cg.gpio_pin_expression(radio_config[key])
            for key in (
                CONF_PIN_SDIO,
                CONF_PIN_SCLK,
                CONF_PIN_CSB,
                CONF_PIN_FCSB,
                CONF_PIN_GPIO3,
            )
        ]
        cg.add(hal.set_pins(*pin_exprs))
    else:
        pin_exprs = [
            await cg.gpio_pin_expression(radio_config[key])
            for key in (CONF_PIN_SCLK, CONF_PIN_MISO, CONF_PIN_MOSI, CONF_PIN_CS)
        ]
        # GDO0 is optional; the backend takes nullptr to mean "poll over SPI".
        if (gdo0 := radio_config.get(CONF_PIN_GDO0)) is not None:
            pin_exprs.append(await cg.gpio_pin_expression(gdo0))
        else:
            pin_exprs.append(cg.nullptr)
        cg.add(hal.set_pins(*pin_exprs))
        cg.add(hal.set_rx_offset_hz(radio_config[CONF_RX_OFFSET]))
        cg.add(hal.set_output_power_dbm(radio_config[CONF_OUTPUT_POWER]))
        cg.add(hal.set_rx_bandwidth_hz(radio_config[CONF_BANDWIDTH]))
        cg.add(hal.set_deviation_hz(radio_config[CONF_DEVIATION]))
        cg.add(hal.set_modulation(radio_config[CONF_MODULATION]))
        cg.add(hal.set_foc_cs_gate(radio_config[CONF_FOC_CARRIER_SENSE_GATE]))
        cg.add(hal.set_sync_word(radio_config[CONF_SYNC_WORD]))

    return hal


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_radio(await radio_to_code(config[CONF_RADIO])))

    if CONF_FIX_CHANNEL in config:
        cg.add(var.set_fix_channel(config[CONF_FIX_CHANNEL]))

    # Component-level, not radio-level: it selects which carrier channel_tx_hz()
    # hands the backend, and both backends are simply told a frequency.
    cg.add(var.set_use_alternative_channels(config[CONF_USE_ALTERNATIVE_CHANNELS]))

    cg.add(var.set_batch_size(config[CONF_BATCH_SIZE]))
    cg.add(var.set_rx_timeout_ms(config[CONF_RX_TIMEOUT]))
    cg.add(var.set_rx_reply_timeout_ms(config[CONF_RX_REPLY_TIMEOUT]))
    cg.add(var.set_skip_fin_beacons(config[CONF_SKIP_FIN_BEACONS]))

    cg.add(var.set_meter_serial(config[CONF_METER_SERIAL]))
    if config[CONF_CIU_SERIAL]:
        cg.add(var.set_ciu_serial(config[CONF_CIU_SERIAL]))
    if CONF_CIU_ADDRESS in config:
        cg.add(var.set_ciu_address(config[CONF_CIU_ADDRESS]))
