#include "acs/gnc/FlightController.h"

#include <algorithm>
#include <cmath>

#include "acs/math/Constants.h"

namespace acs::gnc {

static double clamp(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }

FlightController::FlightController(const ControllerConfig& cfg)
    : cfg_(cfg),
      roll_pid_(cfg.roll_gains, cfg.integrator_limit, cfg.roll_rate_limit_rad_s),
      roll_rate_pid_(cfg.roll_rate_gains, cfg.integrator_limit, cfg.surface_limit_rad),
      pitch_pid_(cfg.pitch_gains, cfg.integrator_limit, cfg.pitch_rate_limit_rad_s),
      pitch_rate_pid_(cfg.pitch_rate_gains, cfg.integrator_limit, cfg.surface_limit_rad),
      heading_pid_(cfg.heading_gains, cfg.integrator_limit, 0.8),
      altitude_pid_(cfg.altitude_gains, cfg.integrator_limit, 0.5) {}

void FlightController::reset() {
  roll_pid_.reset();
  roll_rate_pid_.reset();
  pitch_pid_.reset();
  pitch_rate_pid_.reset();
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

  const double hdg_err = acs::math::wrap_pi(targets.heading_rad - est.yaw_rad);
  const double roll_cmd = clamp(targets.roll_rad + heading_pid_.update(0.0, -hdg_err, dt_s), -0.6, 0.6);

  const double alt_err = targets.altitude_m - est.altitude_m;
  const double pitch_cmd = clamp(targets.pitch_rad + altitude_pid_.update(0.0, -alt_err, dt_s), -0.35, 0.35);

  // Gain scheduling: scale kp/kd by (V_ref/V)^2 so control authority stays consistent across
  // airspeeds — aerodynamic effectiveness scales with V^2, so gains must invert that factor.
  static constexpr double kVref_m_s = 15.0;
  const double v_safe = std::max(5.0, est.airspeed_m_s);
  const double gs = clamp((kVref_m_s / v_safe) * (kVref_m_s / v_safe), 0.25, 4.0);

  PID::Gains scaled_roll_rate = cfg_.roll_rate_gains;
  scaled_roll_rate.kp *= gs;
  scaled_roll_rate.kd *= gs;
  roll_rate_pid_.set_gains(scaled_roll_rate);

  PID::Gains scaled_pitch_rate = cfg_.pitch_rate_gains;
  scaled_pitch_rate.kp *= gs;
  scaled_pitch_rate.kd *= gs;
  pitch_rate_pid_.set_gains(scaled_pitch_rate);

  // Keep the attitude loop deliberately slower than the gyro loop: roll angle
  // produces a bounded p command, while the inner loop rejects fast gust rates.
  const double roll_rate_cmd = roll_pid_.update(roll_cmd, est.roll_rad, dt_s);
  const double aileron = roll_rate_pid_.update(roll_rate_cmd, est.roll_rate_rad_s, dt_s);
  // Cascade pitch angle into a body-q command, then close the fast inner loop
  // directly on the gyro. This keeps pitch-angle authority without allowing a
  // large q transient to run ahead of the integrated attitude.
  const double pitch_rate_cmd = pitch_pid_.update(pitch_cmd, est.pitch_rad, dt_s);
  const double elevator = pitch_rate_pid_.update(pitch_rate_cmd, est.pitch_rate_rad_s, dt_s);

  // throttle for airspeed: simple proportional with a little trim.
  const double v_err = targets.airspeed_m_s - est.airspeed_m_s;
  double throttle = 0.55 + 0.04 * v_err + 0.01 * alt_err;
  throttle = clamp(throttle, 0.0, cfg_.throttle_limit);

  // Feed the yaw damper the steady coordinated-turn rate, r = g*tan(phi)/V,
  // rather than always driving body yaw rate to zero. This prevents the rudder
  // from fighting the bank command while the beta feedback rejects gust slip.
  static constexpr double kGravity_m_s2 = 9.80665;
  const double coordinated_yaw_rate =
      clamp(cfg_.coordinated_turn_gain * kGravity_m_s2 * std::tan(roll_cmd) / v_safe, -0.8, 0.8);

  // Positive beta is velocity toward body-right. A positive rudder command
  // creates the positive yaw moment that drives beta back toward zero.
  const double rudder = clamp(cfg_.sideslip_gain * est.sideslip_rad
                                  + cfg_.yaw_rate_damping_gain *
                                        (coordinated_yaw_rate - est.yaw_rate_rad_s)
                                  + cfg_.heading_rudder_gain * hdg_err,
                              -cfg_.surface_limit_rad, cfg_.surface_limit_rad);

  acs::aero::ControlInputs u{};
  u.aileron_rad = clamp(aileron, -cfg_.surface_limit_rad, cfg_.surface_limit_rad);
  u.elevator_rad = clamp(elevator, -cfg_.surface_limit_rad, cfg_.surface_limit_rad);
  u.rudder_rad = rudder;
  u.throttle_01 = throttle;
  return u;
}

}  // namespace acs::gnc
