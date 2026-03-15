"""Nartis RF Meter — Text sensor platform."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import CONF_ID

from . import NartisRfMeterComponent, nartis_rf_meter_ns
from .sensor import validate_obis_code, CONF_NARTIS_RF_METER_ID, CONF_OBIS_CODE, CONF_OBIS_CLASS, CONF_OBIS_ATTR

ObisCode = nartis_rf_meter_ns.struct("ObisCode")

DEPENDENCIES = ["nartis_rf_meter"]

CONFIG_SCHEMA = (
    text_sensor.text_sensor_schema()
    .extend(
        {
            cv.GenerateID(CONF_NARTIS_RF_METER_ID): cv.use_id(NartisRfMeterComponent),
            cv.Required(CONF_OBIS_CODE): validate_obis_code,
            cv.Optional(CONF_OBIS_CLASS, default=1): cv.int_range(min=1, max=255),
            cv.Optional(CONF_OBIS_ATTR, default=2): cv.int_range(min=1, max=255),
        }
    )
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_NARTIS_RF_METER_ID])
    var = await text_sensor.new_text_sensor(config)

    obis_parts = [int(p) for p in config[CONF_OBIS_CODE].split(".")]
    obis = cg.StructInitializer(
        ObisCode,
        ("bytes", obis_parts),
    )

    cg.add(
        parent.register_text_sensor(
            var,
            obis,
            config[CONF_OBIS_CLASS],
            config[CONF_OBIS_ATTR],
        )
    )
