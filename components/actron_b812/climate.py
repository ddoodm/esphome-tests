import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor, climate, remote_transmitter, sensor, text_sensor
from esphome.const import CONF_ID, UNIT_SECOND, STATE_CLASS_MEASUREMENT

AUTO_LOAD = ["sensor", "binary_sensor", "text_sensor"]

actron_b812_ns = cg.esphome_ns.namespace("actron_b812")
ActronB812Climate = actron_b812_ns.class_(
    "ActronB812Climate", climate.Climate, cg.PollingComponent
)

CONF_TRANSMITTER_ID = "transmitter_id"
CONF_COMPRESSOR_COOLDOWN = "compressor_cooldown"
CONF_VALVE_SETTLE_TIME = "valve_settle_time"
CONF_TEMPERATURE_SENSOR = "temperature_sensor_id"
CONF_HYSTERESIS = "hysteresis"
CONF_COMPRESSOR_RUNNING = "compressor_running"
CONF_STATE_SENSOR = "state"
CONF_TIMER_REMAINING = "timer_remaining"

CONFIG_SCHEMA = climate.climate_schema(ActronB812Climate).extend({
    cv.Required(CONF_TRANSMITTER_ID): cv.use_id(
        remote_transmitter.RemoteTransmitterComponent
    ),
    cv.Optional(CONF_COMPRESSOR_COOLDOWN, default="3min"): cv.positive_time_period_milliseconds,
    cv.Optional(CONF_VALVE_SETTLE_TIME, default="30s"): cv.positive_time_period_milliseconds,
    cv.Optional(CONF_TEMPERATURE_SENSOR): cv.use_id(sensor.Sensor),
    cv.Optional(CONF_HYSTERESIS, default=0.5): cv.positive_float,
    cv.Optional(CONF_COMPRESSOR_RUNNING): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_STATE_SENSOR): text_sensor.text_sensor_schema(),
    cv.Optional(CONF_TIMER_REMAINING): sensor.sensor_schema(
        unit_of_measurement=UNIT_SECOND,
        accuracy_decimals=0,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
}).extend(cv.polling_component_schema("222ms"))

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await climate.register_climate(var, config)

    tx = await cg.get_variable(config[CONF_TRANSMITTER_ID])
    cg.add(var.set_transmitter(tx))
    cg.add(var.set_compressor_cooldown(config[CONF_COMPRESSOR_COOLDOWN]))
    cg.add(var.set_valve_settle_time(config[CONF_VALVE_SETTLE_TIME]))

    if CONF_TEMPERATURE_SENSOR in config:
        s = await cg.get_variable(config[CONF_TEMPERATURE_SENSOR])
        cg.add(var.set_temperature_sensor(s))
    cg.add(var.set_hysteresis(config[CONF_HYSTERESIS]))

    if CONF_COMPRESSOR_RUNNING in config:
        bs = await binary_sensor.new_binary_sensor(config[CONF_COMPRESSOR_RUNNING])
        cg.add(var.set_compressor_running_sensor(bs))

    if CONF_STATE_SENSOR in config:
        ts = await text_sensor.new_text_sensor(config[CONF_STATE_SENSOR])
        cg.add(var.set_state_sensor(ts))

    if CONF_TIMER_REMAINING in config:
        s = await sensor.new_sensor(config[CONF_TIMER_REMAINING])
        cg.add(var.set_timer_remaining_sensor(s))