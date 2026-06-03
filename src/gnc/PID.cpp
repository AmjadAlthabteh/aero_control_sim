#include "acs/gnc/PID.h"

#include <algorithm>
#include <cmath>

namespace acs::gnc {

static double clamp(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }

PID::PID(const Gains& gains, double i_limit, double out_limit)
    : gains_(gains), i_limit_(std::abs(i_limit)), out_limit_(std::abs(out_limit)) {}

void PID::set_gains(const Gains& gains) { gains_ = gains; }

void PID::set_limits(double i_limit, double out_limit) {
  i_limit_ = std::abs(i_limit);
  out_limit_ = std::abs(out_limit);
}

void PID::reset() {
  integrator_ = 0.0;
  prev_measurement_ = 0.0;
  d_filtered_ = 0.0;
  has_prev_ = false;
}

double PID::update(double setpoint, double measurement, double dt_s) {
  const double error = setpoint - measurement;
  const bool dt_ok = std::isfinite(dt_s) && dt_s > 0.0;

  // derivative on measurement (prevents derivative kick on setpoint changes)
  double d_meas = 0.0;
  if (has_prev_ && dt_ok) {
    d_meas = (measurement - prev_measurement_) / dt_s;

    // optional first-order low-pass filter on derivative term
    if (gains_.tau_d > 0.0) {
      // matched-Z (impulse-invariant) discretisation: places the digital pole at
      // exp(-dt/tau), exactly matching the analog corner frequency at any sample rate.
      const double alpha = 1.0 - std::exp(-dt_s / gains_.tau_d);
      d_filtered_ = (1.0 - alpha) * d_filtered_ + alpha * d_meas;
      d_meas = d_filtered_;
    }
  }
  prev_measurement_ = measurement;
  has_prev_ = true;

  // compute control output before saturation
  const double u_unsat = gains_.kp * error + gains_.ki * integrator_ - gains_.kd * d_meas;
  const double u = clamp(u_unsat, -out_limit_, out_limit_);

  if (!dt_ok) {
    return u;
  }

  // back-calculation anti-windup: reduce integrator based on saturation error
  const double u_sat_error = u - u_unsat;
  integrator_ += (error + gains_.kaw * u_sat_error) * dt_s;
  integrator_ = clamp(integrator_, -i_limit_, i_limit_);

  return u;
}

}  // namespace acs::gnc
