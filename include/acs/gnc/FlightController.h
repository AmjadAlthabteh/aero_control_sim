#pragma once

#include "acs/aero/AeroModel.h"
#include "acs/gnc/PID.h"
#include "acs/math/Vector3.h"

namespace acs::gnc {

struct ControllerTargets {
  double roll_rad{};
  double pitch_rad{};
  double heading_rad{};
  double altitude_m{};
  double airspeed_m_s{15.0};
};

struct ControllerConfig {
  PID::Gains roll_gains{6.0, 0.5, 0.25};
  PID::Gains pitch_gains{7.0, 0.6, 0.30};
  PID::Gains heading_gains{2.0, 0.1, 0.0};
  PID::Gains altitude_gains{0.8, 0.1, 0.0};

  double integrator_limit{0.5};
  double surface_limit_rad{0.35};
  double throttle_limit{1.0};
};

struct EstimatedState {
  // angles are for control and come from sensor fusion in real systems.
  // here we use "perfect" euler angles derived from truth, optionally with noise from sensors.
  double roll_rad{};
  double pitch_rad{};
  double yaw_rad{};

  double altitude_m{};
  double airspeed_m_s{};
  double yaw_rate_rad_s{};
};

class FlightController {
public:
  explicit FlightController(const ControllerConfig& cfg);

  void reset();
  acs::aero::ControlInputs update(const ControllerTargets& targets,
                                 const EstimatedState& est,
                                 double dt_s);

private:
  ControllerConfig cfg_{};
  PID roll_pid_{};
  PID pitch_pid_{};
  PID heading_pid_{};
  PID altitude_pid_{};
};

}  // namespace acs::gnc

