#pragma once

#include "acs/math/Vector3.h"
#include "acs/simulation/Sensors.h"
#include <array>

namespace acs::simulation {

struct FusionConfig {
  double accel_process_noise_m_s2{0.35};
  double accel_bias_process_noise_m_s2{0.015};
  double initial_position_std_m{2.0};
  double initial_velocity_std_m_s{0.5};
  double initial_accel_bias_std_m_s2{0.2};
  double attitude_time_constant_s{0.5};
};

struct FilteredState {
  acs::math::Vector3 position_ned_m{}, velocity_ned_m_s{}, accel_bias_body_m_s2{};
  double roll_rad{}, pitch_rad{}, yaw_rad{}, airspeed_m_s{}, altitude_m{};
  double position_std_m{}, velocity_std_m_s{}, gps_innovation_m{}, air_data_innovation_m_s{};
  bool initialized{false}, gps_aiding{false}, air_data_aiding{false};
};

// Nine-state navigation KF: NED position, velocity, and accelerometer bias.
// Attitude is gyro-propagated and gravity-referenced by the accelerometer.
class SensorFusion {
public:
  SensorFusion(const FusionConfig& = {}, const SensorConfig& = {});
  void reset(const acs::math::Vector3&, const acs::math::Vector3&, const acs::math::Vector3& euler321_rad);
  const FilteredState& update(const ImuSample&, const GpsSample&, const AirDataSample&,
                              const acs::math::Vector3& wind_ned_m_s, double dt_s);
  const FilteredState& state() const { return output_; }
private:
  static constexpr int N = 9;
  using Covariance = std::array<std::array<double, N>, N>;
  void propagate_attitude(const ImuSample&, double);
  void predict(const ImuSample&, double);
  double correct(const std::array<double, N>&, double residual, double variance);
  void refresh();
  FusionConfig cfg_{};
  SensorConfig sensor_cfg_{};
  std::array<double, N> x_{};
  Covariance p_{};
  FilteredState output_{};
};
}  // namespace acs::simulation
