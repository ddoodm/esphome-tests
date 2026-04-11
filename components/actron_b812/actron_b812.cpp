#include "actron_b812.hpp"
#include "esphome/core/log.h"

namespace esphome {
namespace actron_b812 {

static const char *TAG = "actron_b812";

climate::ClimateTraits ActronB812Climate::traits() {
  auto traits = climate::ClimateTraits();
  traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE |
                           climate::CLIMATE_SUPPORTS_TWO_POINT_TARGET_TEMPERATURE);
  traits.set_supported_modes({
    climate::CLIMATE_MODE_OFF,
    climate::CLIMATE_MODE_COOL,
    climate::CLIMATE_MODE_HEAT,
    climate::CLIMATE_MODE_HEAT_COOL,
    climate::CLIMATE_MODE_FAN_ONLY,
  });
  traits.set_supported_fan_modes({
    climate::CLIMATE_FAN_LOW,
    climate::CLIMATE_FAN_MEDIUM,
    climate::CLIMATE_FAN_HIGH,
  });
  return traits;
}

void ActronB812Climate::setup() {
  active_cmd_ = CMD_OFF;
  if (temperature_sensor_) {
    temperature_sensor_->add_on_state_callback([this](float v) {
      this->current_temperature = v;
      evaluate_thermostat_();
      update_action_();
    });
  }
  publish_sensors_();
}

void ActronB812Climate::control(const climate::ClimateCall &call) {
  if (call.get_mode().has_value())
    pending_mode_ = *call.get_mode();
  if (call.get_fan_mode().has_value())
    pending_fan_ = *call.get_fan_mode();
  if (call.get_target_temperature().has_value())
    this->target_temperature = *call.get_target_temperature();
  if (call.get_target_temperature_low().has_value())
    this->target_temperature_low = *call.get_target_temperature_low();
  if (call.get_target_temperature_high().has_value())
    this->target_temperature_high = *call.get_target_temperature_high();

  evaluate_thermostat_();

  // Publish desired state immediately so the HA UI reflects the change
  // even while we may be waiting for the compressor protection timer.
  this->mode = pending_mode_;
  this->fan_mode = pending_fan_;
  this->publish_state();

  update_action_();

  pending_change_ = true;
}

void ActronB812Climate::update() {
  // Fan speed is always applied immediately, even during cooldown waits.
  // Skip when OFF — CMD_OFF zeroes all bits including fan.
  if (pending_mode_ != climate::CLIMATE_MODE_OFF) {
    active_cmd_ &= ~(BIT_FS1 | BIT_FS2 | BIT_FS3);
    switch (pending_fan_) {
      case climate::CLIMATE_FAN_LOW:    active_cmd_ |= BIT_FS1; break;
      case climate::CLIMATE_FAN_MEDIUM: active_cmd_ |= BIT_FS2; break;
      case climate::CLIMATE_FAN_HIGH:   active_cmd_ |= BIT_FS3; break;
      default: break;
    }
  }

  if (pending_change_) {
    uint8_t desired = build_cmd_(effective_mode_(), pending_fan_);
    bool comp_desired = (desired & BIT_COMP) != 0;
    bool heat_direction_change = comp_desired && ((desired & BIT_HEAT) != (active_cmd_ & BIT_HEAT));

    // Branch A: compressor not needed (OFF or FAN_ONLY).
    if (!comp_desired) {
      if (comp_running_) {
        comp_off_time_ = millis();
        comp_timer_armed_ = true;
        comp_running_ = false;
        ESP_LOGD(TAG, "Compressor stopped, cooldown armed");
      }

      // If HEAT is still set, keep it until cooldown elapses so the reversing
      // valve doesn't switch while there's residual refrigerant pressure.
      if ((active_cmd_ & BIT_HEAT) && comp_timer_armed_ && !comp_cooldown_elapsed_()) {
        active_cmd_ = desired | BIT_HEAT;
        ESP_LOGD(TAG, "Keeping HEAT on until cooldown elapses (%.0fs remaining)",
                 (comp_cooldown_ms_ - (millis() - comp_off_time_)) / 1000.0f);
      } else {
        active_cmd_ = desired;
        pending_change_ = false;
        ESP_LOGD(TAG, "Applying command 0x%02X", active_cmd_);
      }

    // Step 1: If the heat direction is changing and comp is still running, stop it first.
    } else if (comp_running_ && heat_direction_change) {
      active_cmd_ = (active_cmd_ & ~BIT_COMP) & ~BIT_CALL;
      comp_running_ = false;
      comp_off_time_ = millis();
      comp_timer_armed_ = true;
      ESP_LOGW(TAG, "Heat direction change — stopping compressor first");

    // Step 2: Wait for compressor cooldown before restarting.
    } else if (comp_timer_armed_ && !comp_cooldown_elapsed_()) {
      ESP_LOGD(TAG, "Waiting for compressor cooldown (%.0fs remaining)",
               (comp_cooldown_ms_ - (millis() - comp_off_time_)) / 1000.0f);

    // Step 3: HEAT going ON→OFF (Heat→Cool direction) — clear bit and arm valve timer.
    } else if ((active_cmd_ & BIT_HEAT) && !(desired & BIT_HEAT) && !valve_timer_armed_) {
      active_cmd_ &= ~BIT_HEAT;
      valve_switch_time_ = millis();
      valve_timer_armed_ = true;
      ESP_LOGD(TAG, "Reversing valve switching to cool — waiting for valve to settle");

    // Step 4: Wait for reversing valve to settle after going ON→OFF.
    } else if (valve_timer_armed_ && !valve_settled_()) {
      ESP_LOGD(TAG, "Waiting for valve settle (%.0fs remaining)",
               (valve_settle_ms_ - (millis() - valve_switch_time_)) / 1000.0f);

    // Step 5: All clear — apply the full desired command.
    } else {
      active_cmd_ = desired;
      comp_running_ = true;
      comp_timer_armed_ = false;
      valve_timer_armed_ = false;
      pending_change_ = false;
      ESP_LOGD(TAG, "Applying command 0x%02X", active_cmd_);
    }
  }

  // Override CALL bit based on thermostat desire, independent of compressor state.
  // This signals to the AC unit that conditioning is wanted even during cooldown waits.
  if (thermostat_direction_ != THERMO_OFF)
    active_cmd_ |= BIT_CALL;
  else
    active_cmd_ &= ~BIT_CALL;

  send_frame_(active_cmd_);
  publish_sensors_();
}

bool ActronB812Climate::comp_cooldown_elapsed_() {
  return (millis() - comp_off_time_) >= comp_cooldown_ms_;
}

bool ActronB812Climate::valve_settled_() {
  return (millis() - valve_switch_time_) >= valve_settle_ms_;
}

std::string ActronB812Climate::compute_state_() {
  // Timers take priority — describe what we're waiting for.
  if (comp_timer_armed_ && !comp_cooldown_elapsed_()) {
    // If HEAT bit is still held (Branch A waiting to clear valve), label differently.
    return (active_cmd_ & BIT_HEAT) ? "heat_valve_hold" : "comp_cooldown";
  }
  if (valve_timer_armed_ && !valve_settled_())
    return "valve_settling";
  if (active_cmd_ & BIT_COMP)
    return (active_cmd_ & BIT_HEAT) ? "heating" : "cooling";
  if (active_cmd_ & (BIT_FS1 | BIT_FS2 | BIT_FS3))
    return "fan_only";
  return "off";
}

float ActronB812Climate::timer_remaining_s_() {
  if (comp_timer_armed_ && !comp_cooldown_elapsed_())
    return (comp_cooldown_ms_ - (millis() - comp_off_time_)) / 1000.0f;
  if (valve_timer_armed_ && !valve_settled_())
    return (valve_settle_ms_ - (millis() - valve_switch_time_)) / 1000.0f;
  return 0.0f;
}

void ActronB812Climate::publish_sensors_() {
  if (compressor_running_sensor_)
    compressor_running_sensor_->publish_state(comp_running_);

  if (state_sensor_)
    state_sensor_->publish_state(compute_state_());

  if (timer_remaining_sensor_) {
    int remaining_s = static_cast<int>(timer_remaining_s_());
    if (remaining_s != timer_remaining_last_s_) {
      timer_remaining_last_s_ = remaining_s;
      timer_remaining_sensor_->publish_state(static_cast<float>(remaining_s));
    }
  }
}

void ActronB812Climate::evaluate_thermostat_() {
  if (std::isnan(this->current_temperature))
    return;
  float t = this->current_temperature;
  ThermostatDirection want = thermostat_direction_;

  if (pending_mode_ == climate::CLIMATE_MODE_COOL) {
    float tgt = this->target_temperature;
    if (std::isnan(tgt)) return;
    if (thermostat_direction_ != THERMO_COOL && t > tgt + hysteresis_)
      want = THERMO_COOL;
    else if (thermostat_direction_ == THERMO_COOL && t < tgt - hysteresis_)
      want = THERMO_OFF;

  } else if (pending_mode_ == climate::CLIMATE_MODE_HEAT) {
    float tgt = this->target_temperature;
    if (std::isnan(tgt)) return;
    if (thermostat_direction_ != THERMO_HEAT && t < tgt - hysteresis_)
      want = THERMO_HEAT;
    else if (thermostat_direction_ == THERMO_HEAT && t > tgt + hysteresis_)
      want = THERMO_OFF;

  } else if (pending_mode_ == climate::CLIMATE_MODE_HEAT_COOL) {
    float lo = this->target_temperature_low;
    float hi = this->target_temperature_high;
    if (std::isnan(lo) || std::isnan(hi)) return;
    if (t > hi + hysteresis_)
      want = THERMO_COOL;
    else if (t < lo - hysteresis_)
      want = THERMO_HEAT;
    else if (thermostat_direction_ == THERMO_COOL && t < hi - hysteresis_)
      want = THERMO_OFF;
    else if (thermostat_direction_ == THERMO_HEAT && t > lo + hysteresis_)
      want = THERMO_OFF;

  } else {
    want = THERMO_OFF;
  }

  if (want != thermostat_direction_) {
    thermostat_direction_ = want;
    pending_change_ = true;
    ESP_LOGD(TAG, "Thermostat -> %s (%.1f°C)",
             want == THERMO_COOL ? "cool" : want == THERMO_HEAT ? "heat" : "idle", t);
  }
}

climate::ClimateMode ActronB812Climate::effective_mode_() {
  if (pending_mode_ == climate::CLIMATE_MODE_COOL ||
      pending_mode_ == climate::CLIMATE_MODE_HEAT ||
      pending_mode_ == climate::CLIMATE_MODE_HEAT_COOL) {
    switch (thermostat_direction_) {
      case THERMO_COOL: return climate::CLIMATE_MODE_COOL;
      case THERMO_HEAT: return climate::CLIMATE_MODE_HEAT;
      default:          return climate::CLIMATE_MODE_FAN_ONLY;
    }
  }
  return pending_mode_;
}

void ActronB812Climate::update_action_() {
  climate::ClimateAction a;
  if (pending_mode_ == climate::CLIMATE_MODE_OFF)
    a = climate::CLIMATE_ACTION_OFF;
  else if (pending_mode_ == climate::CLIMATE_MODE_FAN_ONLY)
    a = climate::CLIMATE_ACTION_FAN;
  else if (thermostat_direction_ == THERMO_COOL)
    a = climate::CLIMATE_ACTION_COOLING;
  else if (thermostat_direction_ == THERMO_HEAT)
    a = climate::CLIMATE_ACTION_HEATING;
  else
    a = climate::CLIMATE_ACTION_IDLE;
  if (this->action != a) {
    this->action = a;
    this->publish_state();
  }
}

uint8_t ActronB812Climate::build_cmd_(climate::ClimateMode mode,
                                      climate::ClimateFanMode fan) {
  uint8_t cmd = BIT_FRM1 | BIT_FRM2;

  // Fan speed
  switch (fan) {
    case climate::CLIMATE_FAN_LOW:    cmd |= BIT_FS1; break;
    case climate::CLIMATE_FAN_MEDIUM: cmd |= BIT_FS2; break;
    case climate::CLIMATE_FAN_HIGH:   cmd |= BIT_FS3; break;
    default: break;
  }

  // Mode
  switch (mode) {
    case climate::CLIMATE_MODE_COOL:
      cmd |= BIT_CALL | BIT_COMP;
      break;
    case climate::CLIMATE_MODE_HEAT:
      cmd |= BIT_CALL | BIT_COMP | BIT_HEAT;
      break;
    case climate::CLIMATE_MODE_FAN_ONLY:
      // Fan bits already set above, no CALL or COMP
      break;
    case climate::CLIMATE_MODE_OFF:
    default:
      cmd = CMD_OFF;  // Clear fan bits too
      break;
  }

  return cmd;
}

void ActronB812Climate::send_frame_(uint8_t data) {
  auto call = transmitter_->transmit();
  auto *d = call.get_data();
  d->set_carrier_frequency(0);

  for (int i = 7; i >= 0; i--) {
    d->mark(150);
    d->space((data >> i) & 1 ? 6000 : 2000);
  }
  d->mark(150);  // trailing pulse

  call.perform();
}

}  // namespace actron_b812
}  // namespace esphome