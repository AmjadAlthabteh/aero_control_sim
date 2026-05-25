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
  has_prev_ = false;
}

double PID::update(double setpoint, double measurement, double dt_s) {
  const double error = setpoint - measurement;

  integrator_ += error * dt_s;
  integrator_ = clamp(integrator_, -i_limit_, i_limit_);

  double d_meas = 0.0;
  if (has_prev_ && dt_s > 0.0) {
    d_meas = (measurement - prev_measurement_) / dt_s;
  }
  prev_measurement_ = measurement;
  has_prev_ = true;

  const double u = gains_.kp * error + gains_.ki * integrator_ - gains_.kd * d_meas;
  return clamp(u, -out_limit_, out_limit_);
}

}  // namespace acs::gnc

