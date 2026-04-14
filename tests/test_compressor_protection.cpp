#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Fake millis() — controls time for the component under test.
// Must be defined before including the component (the linker resolves calls
// from actron_b812.cpp to this definition).
// ---------------------------------------------------------------------------
static uint32_t s_fake_millis = 0;

namespace esphome {
uint32_t millis() { return s_fake_millis; }
}  // namespace esphome

#include "actron_b812.hpp"

using namespace esphome;
using namespace esphome::actron_b812;

// ---------------------------------------------------------------------------
// TestableClimate — exposes protected members for white-box testing
// ---------------------------------------------------------------------------
class TestableClimate : public ActronB812Climate {
 public:
  using ActronB812Climate::active_cmd_;
  using ActronB812Climate::comp_running_;
  using ActronB812Climate::comp_timer_armed_;
  using ActronB812Climate::valve_timer_armed_;
  using ActronB812Climate::pending_change_;
  using ActronB812Climate::pending_mode_;
  using ActronB812Climate::thermostat_direction_;
  using ActronB812Climate::comp_off_time_;
  using ActronB812Climate::valve_switch_time_;
  using ActronB812Climate::comp_cooldown_ms_;
  using ActronB812Climate::valve_settle_ms_;
};

// ---------------------------------------------------------------------------
// Frame history — one entry per update() tick, for post-hoc invariant checking
// ---------------------------------------------------------------------------
struct Frame {
  uint32_t time_ms;
  uint8_t cmd;
  bool comp_running;
};

static std::string cmd_hex(uint8_t v) {
  char buf[8];
  snprintf(buf, sizeof(buf), "0x%02X", v);
  return buf;
}

// ---------------------------------------------------------------------------
// Harness — drives the component through scenarios and checks invariants
// ---------------------------------------------------------------------------
static constexpr uint32_t COOLDOWN_MS = 180'000;
static constexpr uint32_t SETTLE_MS = 30'000;
static constexpr uint32_t TICK_MS = 222;

class Harness {
 public:
  TestableClimate climate;
  remote_transmitter::RemoteTransmitterComponent tx;
  std::vector<Frame> frames;

  Harness() {
    s_fake_millis = 10'000;  // non-zero start to catch unsigned underflow bugs
    climate.set_transmitter(&tx);
    climate.set_compressor_cooldown(COOLDOWN_MS);
    climate.set_valve_settle_time(SETTLE_MS);
    climate.set_hysteresis(0.5f);
    climate.target_temperature = 22.0f;
  }

  void set_mode(climate::ClimateMode mode) {
    climate::ClimateCall call;
    call.set_mode(mode);
    climate.control(call);
  }

  void tick(uint32_t ms = TICK_MS) {
    s_fake_millis += ms;
    climate.update();
    frames.push_back({s_fake_millis, climate.active_cmd_, climate.comp_running_});
  }

  void run_for(uint32_t total_ms) {
    uint32_t start = s_fake_millis;
    while (s_fake_millis - start < total_ms)
      tick();
  }

  // ------------------------------------------------------------------
  // Post-hoc invariant checker — scans the full frame history and fails
  // on any violation.  Timing-based, so it doesn't rely on internal flags.
  //
  //  1. Valve (HEAT bit) never toggles while the compressor was running
  //  2. Valve never toggles during the compressor cooldown window
  //  3. Compressor never starts before the cooldown has elapsed
  //  4. Compressor never starts before the valve settle time (Heat→Cool)
  // ------------------------------------------------------------------
  void check_invariants() {
    bool had_comp_run = false;
    uint32_t comp_stop_time = 0;

    bool prev_comp = false;
    bool prev_heat = false;
    bool first = true;

    uint32_t last_heat_off_time = 0;
    bool awaiting_valve_settle = false;

    for (auto &f : frames) {
      bool comp = f.comp_running;
      bool heat = (f.cmd & BIT_HEAT) != 0;

      if (!first) {
        // --- Valve (HEAT bit) just changed? ---
        if (heat != prev_heat) {
          INFO("t=" << f.time_ms << "ms: HEAT "
                    << (prev_heat ? "ON" : "OFF") << " -> "
                    << (heat ? "ON" : "OFF")
                    << "  cmd=" << cmd_hex(f.cmd));

          // Invariant 1: must not switch while compressor was already on
          CHECK_FALSE(prev_comp);

          // Invariant 2: must not switch during cooldown
          if (had_comp_run) {
            uint32_t elapsed = f.time_ms - comp_stop_time;
            INFO("  elapsed since comp stop: " << elapsed
                  << "ms  (cooldown=" << COOLDOWN_MS << "ms)");
            CHECK(elapsed >= COOLDOWN_MS);
          }

          // Track Heat→Cool valve switch for invariant 4
          if (prev_heat && !heat) {
            last_heat_off_time = f.time_ms;
            awaiting_valve_settle = true;
          }
        }

        // --- Compressor just started? ---
        if (!prev_comp && comp) {
          INFO("t=" << f.time_ms << "ms: compressor STARTED  cmd=" << cmd_hex(f.cmd));

          // Invariant 3: must wait for cooldown
          if (had_comp_run) {
            uint32_t elapsed = f.time_ms - comp_stop_time;
            INFO("  elapsed since comp stop: " << elapsed
                  << "ms  (cooldown=" << COOLDOWN_MS << "ms)");
            CHECK(elapsed >= COOLDOWN_MS);
          }

          // Invariant 4: must wait for valve settle after Heat→Cool switch
          if (awaiting_valve_settle) {
            uint32_t elapsed = f.time_ms - last_heat_off_time;
            INFO("  elapsed since valve switch: " << elapsed
                  << "ms  (settle=" << SETTLE_MS << "ms)");
            CHECK(elapsed >= SETTLE_MS);
            awaiting_valve_settle = false;
          }
        }

        // --- Compressor just stopped? ---
        if (prev_comp && !comp) {
          comp_stop_time = f.time_ms;
          had_comp_run = true;
        }
      }

      prev_comp = comp;
      prev_heat = heat;
      first = false;
    }
  }
};

// ===========================================================================
// Test cases
// ===========================================================================

TEST_CASE("Heat-to-cool: all four protections hold", "[protection]") {
  Harness h;

  // --- Phase 1: establish HEAT with compressor running ---
  h.climate.current_temperature = 19.0f;  // well below target (22)
  h.set_mode(climate::CLIMATE_MODE_HEAT);
  h.run_for(2000);

  REQUIRE(h.climate.comp_running_);
  REQUIRE((h.climate.active_cmd_ & BIT_HEAT) != 0);
  REQUIRE((h.climate.active_cmd_ & BIT_COMP) != 0);

  // --- Phase 2: switch to COOL ---
  h.climate.current_temperature = 25.0f;  // well above target
  h.set_mode(climate::CLIMATE_MODE_COOL);

  // Run through the full cooldown + valve settle + extra
  h.run_for(COOLDOWN_MS + SETTLE_MS + 5000);

  // Compressor should now be running in cool mode
  REQUIRE(h.climate.comp_running_);
  REQUIRE((h.climate.active_cmd_ & BIT_HEAT) == 0);
  REQUIRE((h.climate.active_cmd_ & BIT_COMP) != 0);

  h.check_invariants();
}

TEST_CASE("Compressor cooldown respected on simple restart", "[protection]") {
  Harness h;

  // Start cooling
  h.climate.current_temperature = 25.0f;
  h.set_mode(climate::CLIMATE_MODE_COOL);
  h.run_for(2000);
  REQUIRE(h.climate.comp_running_);

  // Turn off
  h.set_mode(climate::CLIMATE_MODE_OFF);
  h.tick();
  REQUIRE_FALSE(h.climate.comp_running_);

  // Immediately request COOL again
  h.climate.current_temperature = 25.0f;
  h.set_mode(climate::CLIMATE_MODE_COOL);

  // During cooldown the compressor must stay off
  h.run_for(COOLDOWN_MS - 2000);
  CHECK_FALSE(h.climate.comp_running_);

  // After cooldown it should restart
  h.run_for(5000);
  REQUIRE(h.climate.comp_running_);

  h.check_invariants();
}

TEST_CASE("Cool-to-heat: cooldown but no valve settle required", "[protection]") {
  Harness h;

  // Start cooling
  h.climate.current_temperature = 25.0f;
  h.set_mode(climate::CLIMATE_MODE_COOL);
  h.run_for(2000);
  REQUIRE(h.climate.comp_running_);
  REQUIRE((h.climate.active_cmd_ & BIT_HEAT) == 0);

  // Switch to HEAT
  h.climate.current_temperature = 19.0f;
  h.set_mode(climate::CLIMATE_MODE_HEAT);

  // Compressor should not restart until cooldown elapses...
  h.run_for(COOLDOWN_MS - 2000);
  CHECK_FALSE(h.climate.comp_running_);

  // ...but it should NOT need the extra valve settle time (Cool→Heat direction)
  h.run_for(5000);
  REQUIRE(h.climate.comp_running_);
  REQUIRE((h.climate.active_cmd_ & BIT_HEAT) != 0);

  // Verify the compressor restarted BEFORE cooldown + settle would have elapsed,
  // confirming no valve settle was applied in this direction.
  uint32_t restart_time = 0;
  for (auto &f : h.frames) {
    if (f.comp_running) {
      restart_time = f.time_ms;
    }
  }
  // Find the comp stop time
  bool was_running = false;
  uint32_t stop_time = 0;
  for (auto &f : h.frames) {
    if (was_running && !f.comp_running)
      stop_time = f.time_ms;
    was_running = f.comp_running;
  }
  uint32_t actual_delay = restart_time - stop_time;
  CHECK(actual_delay < COOLDOWN_MS + SETTLE_MS);

  h.check_invariants();
}

TEST_CASE("Valve held during cooldown when switching heat to off", "[protection]") {
  Harness h;

  // Establish HEAT mode
  h.climate.current_temperature = 19.0f;
  h.set_mode(climate::CLIMATE_MODE_HEAT);
  h.run_for(2000);
  REQUIRE(h.climate.comp_running_);
  REQUIRE((h.climate.active_cmd_ & BIT_HEAT) != 0);

  // Switch to OFF
  h.set_mode(climate::CLIMATE_MODE_OFF);
  h.tick();
  REQUIRE_FALSE(h.climate.comp_running_);

  // HEAT bit must stay set during cooldown (protects reversing valve)
  h.run_for(COOLDOWN_MS / 2);
  CHECK((h.climate.active_cmd_ & BIT_HEAT) != 0);

  // After cooldown elapses, HEAT should clear (mode is OFF, no reason to hold)
  h.run_for(COOLDOWN_MS);
  CHECK((h.climate.active_cmd_ & BIT_HEAT) == 0);

  h.check_invariants();
}

TEST_CASE("Rapid mode cycling respects all protections", "[protection]") {
  Harness h;

  // Start cooling
  h.climate.current_temperature = 25.0f;
  h.set_mode(climate::CLIMATE_MODE_COOL);
  h.run_for(3000);
  REQUIRE(h.climate.comp_running_);

  // Barrage of rapid changes — none of these should violate protections
  h.climate.current_temperature = 19.0f;
  h.set_mode(climate::CLIMATE_MODE_HEAT);
  h.run_for(1000);

  h.climate.current_temperature = 25.0f;
  h.set_mode(climate::CLIMATE_MODE_COOL);
  h.run_for(500);

  h.set_mode(climate::CLIMATE_MODE_OFF);
  h.run_for(200);

  h.climate.current_temperature = 19.0f;
  h.set_mode(climate::CLIMATE_MODE_HEAT);
  h.run_for(500);

  h.set_mode(climate::CLIMATE_MODE_FAN_ONLY);
  h.run_for(300);

  h.climate.current_temperature = 25.0f;
  h.set_mode(climate::CLIMATE_MODE_COOL);

  // Let everything settle
  h.run_for(COOLDOWN_MS + SETTLE_MS + 10'000);
  REQUIRE(h.climate.comp_running_);

  h.check_invariants();
}
