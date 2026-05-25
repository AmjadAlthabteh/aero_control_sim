#include "acs/simulation/Actuators.h"

#include <algorithm>
#include <cmath>

namespace acs::simulation {

static double clamp(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }

static double first_order_step(double x, double u, double tau_s, double dt_s) {
  if (tau_s <= 1e-9 || dt_s <= 0.0) return u;
  const double a = std::exp(-dt_s / tau_s);
  return a * x + (1.0 - a) * u;
}

static double rate_limit_step(double x, double u, double rate_limit_per_s, double dt_s) {
  if (rate_limit_per_s <= 0.0 || dt_s <= 0.0) return x;
  const double max_delta = rate_limit_per_s * dt_s;
  const double delta = clamp(u - x, -max_delta, max_delta);
  return x + delta;
}

Actuators::Actuators(const ActuatorConfig& cfg, const ActuatorFaultConfig& faults) : cfg_(cfg), faults_(faults) {
  applied_.throttle_01 = 0.0;
}

void Actuators::reset() {
  applied_ = acs::aero::ControlInputs{};
  applied_.throttle_01 = 0.0;
  aileron_stuck_ = false;
  elevator_stuck_ = false;
  rudder_stuck_ = false;
  throttle_stuck_ = false;
}

const ActuatorConfig& Actuators::cfg() const { return cfg_; }
const ActuatorFaultConfig& Actuators::faults() const { return faults_; }
acs::aero::ControlInputs Actuators::applied() const { return applied_; }

acs::aero::ControlInputs Actuators::update(const acs::aero::ControlInputs& cmd_in, double dt_s, double t_s) {
  acs::aero::ControlInputs cmd = cmd_in;
  cmd.aileron_rad = clamp(cmd.aileron_rad, -cfg_.surface_limit_rad, cfg_.surface_limit_rad);
  cmd.elevator_rad = clamp(cmd.elevator_rad, -cfg_.surface_limit_rad, cfg_.surface_limit_rad);
  cmd.rudder_rad = clamp(cmd.rudder_rad, -cfg_.surface_limit_rad, cfg_.surface_limit_rad);
  cmd.throttle_01 = clamp(cmd.throttle_01, 0.0, 1.0);

  // latch faults if time has passed
  if (!aileron_stuck_ && faults_.aileron_stuck_time_s >= 0.0 && t_s >= faults_.aileron_stuck_time_s) aileron_stuck_ = true;
  if (!elevator_stuck_ && faults_.elevator_stuck_time_s >= 0.0 && t_s >= faults_.elevator_stuck_time_s) elevator_stuck_ = true;
  if (!rudder_stuck_ && faults_.rudder_stuck_time_s >= 0.0 && t_s >= faults_.rudder_stuck_time_s) rudder_stuck_ = true;
  if (!throttle_stuck_ && faults_.throttle_stuck_time_s >= 0.0 && t_s >= faults_.throttle_stuck_time_s) throttle_stuck_ = true;

  // first-order dynamics
  const double aileron_dyn = first_order_step(applied_.aileron_rad, cmd.aileron_rad, cfg_.surface_tau_s, dt_s);
  const double elevator_dyn = first_order_step(applied_.elevator_rad, cmd.elevator_rad, cfg_.surface_tau_s, dt_s);
  const double rudder_dyn = first_order_step(applied_.rudder_rad, cmd.rudder_rad, cfg_.surface_tau_s, dt_s);
  const double throttle_dyn = first_order_step(applied_.throttle_01, cmd.throttle_01, cfg_.throttle_tau_s, dt_s);

  // rate limits
  const double aileron_rl = rate_limit_step(applied_.aileron_rad, aileron_dyn, cfg_.surface_rate_limit_rad_s, dt_s);
  const double elevator_rl = rate_limit_step(applied_.elevator_rad, elevator_dyn, cfg_.surface_rate_limit_rad_s, dt_s);
  const double rudder_rl = rate_limit_step(applied_.rudder_rad, rudder_dyn, cfg_.surface_rate_limit_rad_s, dt_s);
  const double throttle_rl = rate_limit_step(applied_.throttle_01, throttle_dyn, cfg_.throttle_rate_limit_per_s, dt_s);

  if (!aileron_stuck_) applied_.aileron_rad = clamp(aileron_rl, -cfg_.surface_limit_rad, cfg_.surface_limit_rad);
  if (!elevator_stuck_) applied_.elevator_rad = clamp(elevator_rl, -cfg_.surface_limit_rad, cfg_.surface_limit_rad);
  if (!rudder_stuck_) applied_.rudder_rad = clamp(rudder_rl, -cfg_.surface_limit_rad, cfg_.surface_limit_rad);
  if (!throttle_stuck_) applied_.throttle_01 = clamp(throttle_rl, 0.0, 1.0);

  return applied_;
}

}  // namespace acs::simulation

