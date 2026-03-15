"""Nartis RF Meter — Numeric sensor platform."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID

from . import NartisRfMeterComponent, nartis_rf_meter_ns

CONF_NARTIS_RF_METER_ID = "nartis_rf_meter_id"
CONF_OBIS_CODE = "obis_code"
CONF_OBIS_CLASS = "obis_class"
CONF_OBIS_ATTR = "obis_attr"

ObisCode = nartis_rf_meter_ns.struct("ObisCode")

DEPENDENCIES = ["nartis_rf_meter"]


def validate_obis_code(value):
    """Validate and normalize OBIS code format: A.B.C.D.E.F"""
    value = str(value)
    # Support multiple formats: "1.0.1.7.0.255", "1-0:1.7.0.255", "1-0:1.7.0*255"
    value = value.replace("-", ".").replace(":", ".").replace("*", ".")
    parts = value.split(".")
    if len(parts) != 6:
        raise cv.Invalid(f"OBIS code must have 6 parts, got {len(parts)}: {value}")
    try:
        parsed = [int(p) for p in parts]
    except ValueError:
        raise cv.Invalid(f"OBIS code parts must be integers: {value}")
    for i, p in enumerate(parsed):
        if not 0 <= p <= 255:
            raise cv.Invalid(f"OBIS code part {i} out of range (0-255): {p}")
    return ".".join(str(p) for p in parsed)


CONFIG_SCHEMA = (
    sensor.sensor_schema()
    .extend(
        {
            cv.GenerateID(CONF_NARTIS_RF_METER_ID): cv.use_id(NartisRfMeterComponent),
            cv.Required(CONF_OBIS_CODE): validate_obis_code,
            cv.Optional(CONF_OBIS_CLASS, default=3): cv.int_range(min=1, max=255),
            cv.Optional(CONF_OBIS_ATTR, default=2): cv.int_range(min=1, max=255),
        }
    )
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_NARTIS_RF_METER_ID])
    var = await sensor.new_sensor(config)

    # Parse OBIS code string to ObisCode struct
    obis_parts = [int(p) for p in config[CONF_OBIS_CODE].split(".")]
    obis = cg.StructInitializer(
        ObisCode,
        ("bytes", obis_parts),
    )

    cg.add(
        parent.register_sensor(
            var,
            obis,
            config[CONF_OBIS_CLASS],
            config[CONF_OBIS_ATTR],
        )
    )
