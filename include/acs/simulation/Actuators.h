#pragma once

#include "acs/aero/AeroModel.h"

namespace acs::simulation {

struct ActuatorConfig {
  // time constants for first-order actuator dynamics. set to 0 for instantaneous response.
  double surface_tau_s{0.0};
  double throttle_tau_s{0.0};

  // rate limits (slew), applied after the first-order dynamic, in units per second.
  double surface_rate_limit_rad_s{1e9};
  double throttle_rate_limit_per_s{1e9};

  // hard limits.
  double surface_limit_rad{0.35};

  // mechanical play / gearbox backlash. commands inside this band produce no surface output.
  double surface_deadband_rad{0.003};
};

struct ActuatorFaultConfig {
  // if >= 0, latches the surface at its current value at this time (seconds).
  double aileron_stuck_time_s{-1.0};
  double elevator_stuck_time_s{-1.0};
  double rudder_stuck_time_s{-1.0};
  double throttle_stuck_time_s{-1.0};
};

class Actuators {
public:
  explicit Actuators(const ActuatorConfig& cfg = ActuatorConfig{}, const ActuatorFaultConfig& faults = ActuatorFaultConfig{});

  void reset();
  acs::aero::ControlInputs update(const acs::aero::ControlInputs& cmd, double dt_s, double t_s);

  const ActuatorConfig& cfg() const;
  const ActuatorFaultConfig& faults() const;

  acs::aero::ControlInputs applied() const;

private:
  ActuatorConfig cfg_{};
  ActuatorFaultConfig faults_{};

  acs::aero::ControlInputs applied_{};
  bool aileron_stuck_{false};
  bool elevator_stuck_{false};
  bool rudder_stuck_{false};
  bool throttle_stuck_{false};
};

}  // namespace acs::simulation
