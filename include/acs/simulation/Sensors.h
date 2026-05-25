#pragma once

#include "acs/math/Vector3.h"
#include "acs/simulation/Noise.h"

namespace acs::simulation {

struct ImuSample {
  acs::math::Vector3 accel_body_m_s2{};
  acs::math::Vector3 gyro_body_rad_s{};
  bool valid{true};
};

struct GpsSample {
  acs::math::Vector3 position_ned_m{};
  acs::math::Vector3 velocity_ned_m_s{};
  bool valid{true};
};

struct BaroSample {
  double altitude_m{};
  bool valid{true};
};

struct SensorConfig {
  double accel_noise_std_m_s2{0.08};
  double gyro_noise_std_rad_s{0.002};
  double gps_pos_noise_std_m{0.8};
  double gps_vel_noise_std_m_s{0.1};
  double baro_alt_noise_std_m{0.5};

  // constant sensor biases.
  acs::math::Vector3 accel_bias_m_s2{};
  acs::math::Vector3 gyro_bias_rad_s{};

  // bias random walk, 1-sigma per sqrt(second).
  double accel_bias_rw_std_m_s2_sqrt_s{0.0};
  double gyro_bias_rw_std_rad_s_sqrt_s{0.0};

  // simple dropout model (per-sample probability).
  double gps_dropout_prob{0.0};
  double baro_dropout_prob{0.0};
  bool hold_last_on_dropout{true};
};

class Sensors {
public:
  explicit Sensors(const SensorConfig& cfg, unsigned int seed = 42U);
  void reset(unsigned int seed = 42U);

  ImuSample imu(const acs::math::Vector3& specific_force_body_m_s2,
                const acs::math::Vector3& omega_body_rad_s,
                double dt_s);

  GpsSample gps(const acs::math::Vector3& position_ned_m,
                const acs::math::Vector3& velocity_ned_m_s);

  BaroSample baro(double altitude_m);

private:
  SensorConfig cfg_{};
  Noise noise_;
  acs::math::Vector3 accel_bias_m_s2_{};
  acs::math::Vector3 gyro_bias_rad_s_{};
  GpsSample last_gps_{};
  BaroSample last_baro_{};
};

}  // namespace acs::simulation
