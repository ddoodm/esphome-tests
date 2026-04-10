import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, remote_transmitter
from esphome.const import CONF_ID

actron_b812_ns = cg.esphome_ns.namespace("actron_b812")
ActronB812Climate = actron_b812_ns.class_(
    "ActronB812Climate", climate.Climate, cg.PollingComponent
)

CONF_TRANSMITTER_ID = "transmitter_id"
CONF_COMPRESSOR_COOLDOWN = "compressor_cooldown"
CONF_VALVE_SETTLE_TIME = "valve_settle_time"

CONFIG_SCHEMA = climate.climate_schema(ActronB812Climate).extend({
    cv.Required(CONF_TRANSMITTER_ID): cv.use_id(
        remote_transmitter.RemoteTransmitterComponent
    ),
    cv.Optional(CONF_COMPRESSOR_COOLDOWN, default="3min"): cv.positive_time_period_milliseconds,
    cv.Optional(CONF_VALVE_SETTLE_TIME, default="30s"): cv.positive_time_period_milliseconds,
}).extend(cv.polling_component_schema("222ms"))

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await climate.register_climate(var, config)

    tx = await cg.get_variable(config[CONF_TRANSMITTER_ID])
    cg.add(var.set_transmitter(tx))
    cg.add(var.set_compressor_cooldown(config[CONF_COMPRESSOR_COOLDOWN]))
    cg.add(var.set_valve_settle_time(config[CONF_VALVE_SETTLE_TIME]))