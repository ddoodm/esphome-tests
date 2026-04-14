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
CONF_AUTO_DEADBAND = "auto_deadband"
CONF_AUTO_DEADBAND_TIMEOUT = "auto_deadband_timeout"
CONF_COMPRESSOR_RUNNING = "compressor_running"
CONF_STATE_SENSOR = "state"
CONF_TIMER_REMAINING = "timer_remaining"
CONF_THERMOSTAT_DIRECTION = "thermostat_direction"
CONF_DEADBAND_ACTIVE = "deadband_active"
CONF_DEADBAND_EXPIRES_IN = "deadband_expires_in"
CONF_REVERSING_VALVE = "reversing_valve"
CONF_CALL_ACTIVE = "call_active"

CONFIG_SCHEMA = climate.climate_schema(ActronB812Climate).extend({
    cv.Required(CONF_TRANSMITTER_ID): cv.use_id(
        remote_transmitter.RemoteTransmitterComponent
    ),
    cv.Optional(CONF_COMPRESSOR_COOLDOWN, default="3min"): cv.positive_time_period_milliseconds,
    cv.Optional(CONF_VALVE_SETTLE_TIME, default="30s"): cv.positive_time_period_milliseconds,
    cv.Optional(CONF_TEMPERATURE_SENSOR): cv.use_id(sensor.Sensor),
    cv.Optional(CONF_HYSTERESIS, default=0.5): cv.positive_float,
    cv.Optional(CONF_AUTO_DEADBAND, default=1.0): cv.positive_float,
    cv.Optional(CONF_AUTO_DEADBAND_TIMEOUT, default="20min"): cv.positive_time_period_milliseconds,
    cv.Optional(CONF_COMPRESSOR_RUNNING): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_STATE_SENSOR): text_sensor.text_sensor_schema(),
    cv.Optional(CONF_TIMER_REMAINING): sensor.sensor_schema(
        unit_of_measurement=UNIT_SECOND,
        accuracy_decimals=0,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_THERMOSTAT_DIRECTION): text_sensor.text_sensor_schema(),
    cv.Optional(CONF_DEADBAND_ACTIVE): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_DEADBAND_EXPIRES_IN): sensor.sensor_schema(
        unit_of_measurement=UNIT_SECOND,
        accuracy_decimals=0,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_REVERSING_VALVE): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_CALL_ACTIVE): binary_sensor.binary_sensor_schema(),
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
    cg.add(var.set_auto_deadband(config[CONF_AUTO_DEADBAND]))
    cg.add(var.set_auto_deadband_timeout(config[CONF_AUTO_DEADBAND_TIMEOUT]))

    if CONF_COMPRESSOR_RUNNING in config:
        bs = await binary_sensor.new_binary_sensor(config[CONF_COMPRESSOR_RUNNING])
        cg.add(var.set_compressor_running_sensor(bs))

    if CONF_STATE_SENSOR in config:
        ts = await text_sensor.new_text_sensor(config[CONF_STATE_SENSOR])
        cg.add(var.set_state_sensor(ts))

    if CONF_TIMER_REMAINING in config:
        s = await sensor.new_sensor(config[CONF_TIMER_REMAINING])
        cg.add(var.set_timer_remaining_sensor(s))

    if CONF_THERMOSTAT_DIRECTION in config:
        ts = await text_sensor.new_text_sensor(config[CONF_THERMOSTAT_DIRECTION])
        cg.add(var.set_thermostat_direction_sensor(ts))

    if CONF_DEADBAND_ACTIVE in config:
        bs = await binary_sensor.new_binary_sensor(config[CONF_DEADBAND_ACTIVE])
        cg.add(var.set_deadband_active_sensor(bs))

    if CONF_DEADBAND_EXPIRES_IN in config:
        s = await sensor.new_sensor(config[CONF_DEADBAND_EXPIRES_IN])
        cg.add(var.set_deadband_expires_in_sensor(s))

    if CONF_REVERSING_VALVE in config:
        bs = await binary_sensor.new_binary_sensor(config[CONF_REVERSING_VALVE])
        cg.add(var.set_reversing_valve_sensor(bs))

    if CONF_CALL_ACTIVE in config:
        bs = await binary_sensor.new_binary_sensor(config[CONF_CALL_ACTIVE])
        cg.add(var.set_call_active_sensor(bs))