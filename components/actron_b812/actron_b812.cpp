#include "actron_b812.hpp"
#include "esphome/core/log.h"

namespace esphome {
namespace actron_b812 {

static const char *TAG = "actron_b812";

climate::ClimateTraits ActronB812Climate::traits() {
  auto traits = climate::ClimateTraits();
  traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);
  traits.set_supported_modes({
    climate::CLIMATE_MODE_OFF,
    climate::CLIMATE_MODE_COOL,
    climate::CLIMATE_MODE_HEAT,
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
}

void ActronB812Climate::control(const climate::ClimateCall &call) {
  if (call.get_mode().has_value())
    pending_mode_ = *call.get_mode();
  if (call.get_fan_mode().has_value())
    pending_fan_ = *call.get_fan_mode();
  if (call.get_target_temperature().has_value())
    this->target_temperature = *call.get_target_temperature();

  // Publish desired state immediately so the HA UI reflects the change
  // even while we may be waiting for the compressor protection timer.
  this->mode = pending_mode_;
  this->fan_mode = pending_fan_;
  this->publish_state();

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

  if (!pending_change_) {
    send_frame_(active_cmd_);
    return;
  }

  uint8_t desired = build_cmd_(pending_mode_, pending_fan_);
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
      send_frame_(active_cmd_);
      return;
    }

    active_cmd_ = desired;
    pending_change_ = false;
    ESP_LOGD(TAG, "Applying command 0x%02X", active_cmd_);
    send_frame_(active_cmd_);
    return;
  }

  // From here: compressor is desired (COOL or HEAT).

  // Step 1: If the heat direction is changing and comp is still running, stop it first.
  if (comp_running_ && heat_direction_change) {
    active_cmd_ = (active_cmd_ & ~BIT_COMP) & ~BIT_CALL;
    comp_running_ = false;
    comp_off_time_ = millis();
    comp_timer_armed_ = true;
    ESP_LOGW(TAG, "Heat direction change — stopping compressor first");
    send_frame_(active_cmd_);
    return;
  }

  // Step 2: Wait for compressor cooldown before restarting.
  if (comp_timer_armed_ && !comp_cooldown_elapsed_()) {
    ESP_LOGD(TAG, "Waiting for compressor cooldown (%.0fs remaining)",
             (comp_cooldown_ms_ - (millis() - comp_off_time_)) / 1000.0f);
    send_frame_(active_cmd_);
    return;
  }

  // Step 3: HEAT going ON→OFF (Heat→Cool direction).
  // Clear the HEAT bit now and arm the valve settle timer.
  bool heat_going_off = (active_cmd_ & BIT_HEAT) && !(desired & BIT_HEAT);
  if (heat_going_off && !valve_timer_armed_) {
    active_cmd_ &= ~BIT_HEAT;
    valve_switch_time_ = millis();
    valve_timer_armed_ = true;
    ESP_LOGD(TAG, "Reversing valve switching to cool — waiting for valve to settle");
    send_frame_(active_cmd_);
    return;
  }

  // Step 4: Wait for reversing valve to settle after going ON→OFF.
  if (valve_timer_armed_ && !valve_settled_()) {
    ESP_LOGD(TAG, "Waiting for valve settle (%.0fs remaining)",
             (valve_settle_ms_ - (millis() - valve_switch_time_)) / 1000.0f);
    send_frame_(active_cmd_);
    return;
  }

  // Step 5: All clear — apply the full desired command.
  active_cmd_ = desired;
  comp_running_ = true;
  comp_timer_armed_ = false;
  valve_timer_armed_ = false;
  pending_change_ = false;
  ESP_LOGD(TAG, "Applying command 0x%02X", active_cmd_);
  send_frame_(active_cmd_);
}

bool ActronB812Climate::comp_cooldown_elapsed_() {
  return (millis() - comp_off_time_) >= comp_cooldown_ms_;
}

bool ActronB812Climate::valve_settled_() {
  return (millis() - valve_switch_time_) >= valve_settle_ms_;
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