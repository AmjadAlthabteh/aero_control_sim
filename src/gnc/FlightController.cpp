#include "acs/gnc/FlightController.h"

#include <algorithm>
#include <cmath>

#include "acs/math/Constants.h"

namespace acs::gnc {

static double clamp(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }

static double wrap_pi(double a) {
  using acs::math::kPi;
  using acs::math::kTwoPi;
  if (!std::isfinite(a)) return 0.0;
  a = std::remainder(a, kTwoPi);
  if (a <= -kPi) a += kTwoPi;
  if (a > kPi) a -= kTwoPi;
  return a;
}

FlightController::FlightController(const ControllerConfig& cfg)
    : cfg_(cfg),
      roll_pid_(cfg.roll_gains, cfg.integrator_limit, cfg.surface_limit_rad),
      pitch_pid_(cfg.pitch_gains, cfg.integrator_limit, cfg.surface_limit_rad),
      heading_pid_(cfg.heading_gains, cfg.integrator_limit, 0.8),
      altitude_pid_(cfg.altitude_gains, cfg.integrator_limit, 0.5) {}

void FlightController::reset() {
  roll_pid_.reset();
  pitch_pid_.reset();
  heading_pid_.reset();
  altitude_pid_.reset();
}

acs::aero::ControlInputs FlightController::update(const ControllerTargets& targets,
                                                 const EstimatedState& est,
                                                 double dt_s) {
  // a small cascaded control structure:
  // - heading loop generates a roll command (bank-to-turn)
  // - altitude loop generates a pitch command
  // - roll/pitch inner loops drive aileron/elevator

  const double hdg_err = wrap_pi(targets.heading_rad - est.yaw_rad);
  const double roll_cmd = clamp(targets.roll_rad + heading_pid_.update(0.0, -hdg_err, dt_s), -0.6, 0.6);

  const double alt_err = targets.altitude_m - est.altitude_m;
  const double pitch_cmd = clamp(targets.pitch_rad + altitude_pid_.update(0.0, -alt_err, dt_s), -0.35, 0.35);

  const double aileron = roll_pid_.update(roll_cmd, est.roll_rad, dt_s);
  const double elevator = pitch_pid_.update(pitch_cmd, est.pitch_rad, dt_s);

  // throttle for airspeed: simple proportional with a little trim.
  const double v_err = targets.airspeed_m_s - est.airspeed_m_s;
  double throttle = 0.55 + 0.04 * v_err + 0.01 * alt_err;
  throttle = clamp(throttle, 0.0, cfg_.throttle_limit);

  // rudder: lightly damps yaw rate and nudges beta towards zero (we don't estimate beta here, so use yaw rate only).
  const double rudder = clamp(-0.15 * est.yaw_rate_rad_s + 0.25 * hdg_err, -cfg_.surface_limit_rad, cfg_.surface_limit_rad);

  acs::aero::ControlInputs u{};
  u.aileron_rad = clamp(aileron, -cfg_.surface_limit_rad, cfg_.surface_limit_rad);
  u.elevator_rad = clamp(elevator, -cfg_.surface_limit_rad, cfg_.surface_limit_rad);
  u.rudder_rad = rudder;
  u.throttle_01 = throttle;
  return u;
}

}  // namespace acs::gnc
