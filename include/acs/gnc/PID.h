#pragma once

namespace acs::gnc {

class PID {
public:
  struct Gains {
    double kp{};
    double ki{};
    double kd{};
  };

  PID() = default;
  PID(const Gains& gains, double i_limit, double out_limit);

  void set_gains(const Gains& gains);
  void set_limits(double i_limit, double out_limit);
  void reset();

  // simple pid with derivative on measurement.
  double update(double setpoint, double measurement, double dt_s);

private:
  Gains gains_{};
  double i_limit_{};
  double out_limit_{};
  double integrator_{};
  double prev_measurement_{};
  bool has_prev_{false};
};

}  // namespace acs::gnc

