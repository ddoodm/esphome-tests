#pragma once

#include <cmath>
#include <string>
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/remote_transmitter/remote_transmitter.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/time/real_time_clock.h"
#include "esphome/core/component.h"

namespace esphome {
namespace actron_b812 {

enum ThermostatDirection { THERMO_OFF, THERMO_COOL, THERMO_HEAT };

// Bit positions (MSB first, transmission order)
static const uint8_t BIT_FS3  = (1 << 7);  // Fan high
static const uint8_t BIT_FRM1 = (1 << 6);  // Always 1
static const uint8_t BIT_FRM2 = (1 << 5);  // Always 1
static const uint8_t BIT_CALL = (1 << 4);  // Calling for conditioning
static const uint8_t BIT_COMP = (1 << 3);  // Compressor on
static const uint8_t BIT_FS1  = (1 << 2);  // Fan low
static const uint8_t BIT_HEAT = (1 << 1);  // Heat mode (vs cool)
static const uint8_t BIT_FS2  = (1 << 0);  // Fan mid

static const uint8_t CMD_OFF  = BIT_FRM1 | BIT_FRM2;

class ActronB812Climate : public climate::Climate, public PollingComponent {
 public:
  ActronB812Climate() : PollingComponent(222) {}

  void set_transmitter(remote_transmitter::RemoteTransmitterComponent *tx) {
    transmitter_ = tx;
  }
  void set_compressor_cooldown(uint32_t ms) { comp_cooldown_ms_ = ms; }
  void set_valve_settle_time(uint32_t ms) { valve_settle_ms_ = ms; }
  void set_temperature_sensor(sensor::Sensor *s) { temperature_sensor_ = s; }
  void set_hysteresis(float h) { hysteresis_ = h; }
  void set_auto_deadband(float d) { auto_deadband_ = d; }
  void set_auto_deadband_timeout(uint32_t ms) { auto_deadband_timeout_ms_ = ms; }
  void set_time(time::RealTimeClock *t) { time_ = t; }

  void set_compressor_running_sensor(binary_sensor::BinarySensor *s) { compressor_running_sensor_ = s; }
  void set_state_sensor(text_sensor::TextSensor *s) { state_sensor_ = s; }
  void set_protection_expires_at_sensor(text_sensor::TextSensor *s) { protection_expires_at_sensor_ = s; }
  void set_thermostat_direction_sensor(text_sensor::TextSensor *s) { thermostat_direction_sensor_ = s; }
  void set_deadband_active_sensor(binary_sensor::BinarySensor *s) { deadband_active_sensor_ = s; }
  void set_deadband_expires_at_sensor(text_sensor::TextSensor *s) { deadband_expires_at_sensor_ = s; }
  void set_reversing_valve_sensor(binary_sensor::BinarySensor *s) { reversing_valve_sensor_ = s; }
  void set_call_active_sensor(binary_sensor::BinarySensor *s) { call_active_sensor_ = s; }

  void setup() override;
  void update() override;  // Called every 222ms — sends current frame
  void control(const climate::ClimateCall &call) override;
  climate::ClimateTraits traits() override;

 protected:
  remote_transmitter::RemoteTransmitterComponent *transmitter_{nullptr};
  time::RealTimeClock *time_{nullptr};

  // What we are actually transmitting right now
  uint8_t active_cmd_{CMD_OFF};

  // What HA has asked for — may differ from active if we're waiting on a timer
  climate::ClimateMode pending_mode_{climate::CLIMATE_MODE_OFF};
  climate::ClimateFanMode pending_fan_{climate::CLIMATE_FAN_LOW};
  // Speed to use when pending_fan_ == AUTO; updated whenever a concrete speed is selected.
  climate::ClimateFanMode pending_auto_fan_speed_{climate::CLIMATE_FAN_LOW};
  bool pending_change_{false};

  // Thermostat
  sensor::Sensor *temperature_sensor_{nullptr};
  float hysteresis_{0.5f};
  float auto_deadband_{1.0f};
  // How long after going idle before the cross-mode deadband expires.
  // Prevents slow thermal drift (e.g. sun warming room) from being blocked by overshoot protection.
  // Set to 0 to disable. Default 20 min.
  uint32_t auto_deadband_timeout_ms_{20 * 60 * 1000};
  uint32_t auto_deadband_idle_since_{0};  // millis() when thermostat last went idle
  int32_t  deadband_idle_epoch_{0};       // Unix timestamp matching auto_deadband_idle_since_
  ThermostatDirection thermostat_direction_{THERMO_OFF};
  // In HEAT_COOL mode: tracks which direction was last active so we can apply
  // auto_deadband_ before allowing the *opposite* direction to engage.
  // Cleared to THERMO_OFF on mode/setpoint change so fresh-engage uses hysteresis_ only.
  ThermostatDirection auto_deadband_direction_{THERMO_OFF};

  // Compressor protection
  uint32_t comp_cooldown_ms_{3 * 60 * 1000};  // time comp must be off before restarting
  bool comp_running_{false};
  uint32_t comp_off_time_{0};
  int32_t  comp_off_epoch_{0};       // Unix timestamp matching comp_off_time_
  bool comp_timer_armed_{false};  // only true after comp has actually run and stopped

  // Reversing valve settle (Heat→Cool only)
  uint32_t valve_settle_ms_{30 * 1000};  // extra time after HEAT cleared before comp restarts
  uint32_t valve_switch_time_{0};
  int32_t  valve_switch_epoch_{0};   // Unix timestamp matching valve_switch_time_
  bool valve_timer_armed_{false};

  // Optional diagnostic sensors (all nullptr if not configured in YAML)
  binary_sensor::BinarySensor *compressor_running_sensor_{nullptr};
  text_sensor::TextSensor *state_sensor_{nullptr};
  text_sensor::TextSensor *protection_expires_at_sensor_{nullptr};
  text_sensor::TextSensor *thermostat_direction_sensor_{nullptr};
  binary_sensor::BinarySensor *deadband_active_sensor_{nullptr};
  text_sensor::TextSensor *deadband_expires_at_sensor_{nullptr};
  binary_sensor::BinarySensor *reversing_valve_sensor_{nullptr};
  binary_sensor::BinarySensor *call_active_sensor_{nullptr};
  std::string state_last_{"__unset__"};  // dedup
  int comp_running_last_{-1};            // dedup (-1 = unset)
  std::string thermostat_direction_last_{"__unset__"};  // dedup
  int deadband_active_last_{-1};         // dedup (-1 = unset)
  int32_t protection_expires_at_last_{-1};  // dedup
  int32_t deadband_expires_at_last_{-1}; // dedup
  int reversing_valve_last_{-1};         // dedup (-1 = unset)
  int call_active_last_{-1};             // dedup (-1 = unset)

  bool comp_cooldown_elapsed_();
  bool valve_settled_();
  std::string compute_state_();
  std::string compute_thermostat_direction_str_();
  bool deadband_active_();
  int32_t compute_protection_expires_at_();
  int32_t compute_deadband_expires_at_();
  void publish_sensors_();
  void evaluate_thermostat_();
  climate::ClimateMode effective_mode_();
  void update_action_();
  uint8_t build_cmd_(climate::ClimateMode mode, climate::ClimateFanMode fan);
  void send_frame_(uint8_t cmd);
  int32_t epoch_now_();  // returns current Unix timestamp, or 0 if time not synced
};

}  // namespace actron_b812
}  // namespace esphome
