"""Nartis RF Meter — Button platform (manual re-pair trigger)."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button

from . import NartisRfMeterComponent, nartis_rf_meter_ns
from .sensor import CONF_NARTIS_RF_METER_ID

DEPENDENCIES = ["nartis_rf_meter"]

NartisRepairButton = nartis_rf_meter_ns.class_("NartisRepairButton", button.Button)

CONFIG_SCHEMA = button.button_schema(
    NartisRepairButton,
    entity_category="config",
    icon="mdi:link-variant",
).extend(
    {
        cv.GenerateID(CONF_NARTIS_RF_METER_ID): cv.use_id(NartisRfMeterComponent),
    }
)


async def to_code(config):
    var = await button.new_button(config)
    parent = await cg.get_variable(config[CONF_NARTIS_RF_METER_ID])
    cg.add(var.set_parent(parent))
