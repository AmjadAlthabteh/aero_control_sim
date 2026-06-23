#include "acs/simulation/Sensors.h"

#include <cmath>

namespace acs::simulation {

Sensors::Sensors(const SensorConfig& cfg, unsigned int seed)
    : cfg_(cfg),
      noise_(seed),
      accel_bias_m_s2_(cfg.accel_bias_m_s2),
      gyro_bias_rad_s_(cfg.gyro_bias_rad_s) {}

void Sensors::reset(unsigned int seed) {
  noise_ = Noise(seed);
  accel_bias_m_s2_ = cfg_.accel_bias_m_s2;
  gyro_bias_rad_s_ = cfg_.gyro_bias_rad_s;
  last_gps_ = GpsSample{};
  last_baro_ = BaroSample{};
}

ImuSample Sensors::imu(const acs::math::Vector3& specific_force_body_m_s2,
                       const acs::math::Vector3& omega_body_rad_s,
                       double dt_s) {
  if (dt_s > 0.0) {
    const double sdt = std::sqrt(dt_s);
    if (cfg_.accel_bias_rw_std_m_s2_sqrt_s > 0.0) {
      const double accel_rw = cfg_.accel_bias_rw_std_m_s2_sqrt_s * sdt;
      accel_bias_m_s2_.x += noise_.gaussian(0.0, accel_rw);
      accel_bias_m_s2_.y += noise_.gaussian(0.0, accel_rw);
      accel_bias_m_s2_.z += noise_.gaussian(0.0, accel_rw);
    }
    if (cfg_.gyro_bias_rw_std_rad_s_sqrt_s > 0.0) {
      const double gyro_rw = cfg_.gyro_bias_rw_std_rad_s_sqrt_s * sdt;
      gyro_bias_rad_s_.x += noise_.gaussian(0.0, gyro_rw);
      gyro_bias_rad_s_.y += noise_.gaussian(0.0, gyro_rw);
      gyro_bias_rad_s_.z += noise_.gaussian(0.0, gyro_rw);
    }
  }

  ImuSample s{};
  s.accel_body_m_s2 = acs::math::Vector3(
      specific_force_body_m_s2.x + accel_bias_m_s2_.x + noise_.gaussian(0.0, cfg_.accel_noise_std_m_s2),
      specific_force_body_m_s2.y + accel_bias_m_s2_.y + noise_.gaussian(0.0, cfg_.accel_noise_std_m_s2),
      specific_force_body_m_s2.z + accel_bias_m_s2_.z + noise_.gaussian(0.0, cfg_.accel_noise_std_m_s2));
  s.gyro_body_rad_s = acs::math::Vector3(omega_body_rad_s.x + gyro_bias_rad_s_.x + noise_.gaussian(0.0, cfg_.gyro_noise_std_rad_s),
                                        omega_body_rad_s.y + gyro_bias_rad_s_.y + noise_.gaussian(0.0, cfg_.gyro_noise_std_rad_s),
                                        omega_body_rad_s.z + gyro_bias_rad_s_.z + noise_.gaussian(0.0, cfg_.gyro_noise_std_rad_s));
  return s;
}

GpsSample Sensors::gps(const acs::math::Vector3& position_ned_m, const acs::math::Vector3& velocity_ned_m_s) {
  const bool dropout = noise_.bernoulli(cfg_.gps_dropout_prob);
  if (dropout) {
    if (cfg_.hold_last_on_dropout) {
      GpsSample held = last_gps_;
      held.valid = false;
      return held;
    }
    GpsSample s{};
    s.valid = false;
    return s;
  }

  // Vertical GPS accuracy is ~1.5x worse than horizontal due to satellite geometry (VDOP > HDOP).
  static constexpr double kVdopFactor = 1.5;

  GpsSample s{};
  s.position_ned_m = acs::math::Vector3(position_ned_m.x + noise_.gaussian(0.0, cfg_.gps_pos_noise_std_m),
                                       position_ned_m.y + noise_.gaussian(0.0, cfg_.gps_pos_noise_std_m),
                                       position_ned_m.z + noise_.gaussian(0.0, cfg_.gps_pos_noise_std_m * kVdopFactor));
  s.velocity_ned_m_s = acs::math::Vector3(velocity_ned_m_s.x + noise_.gaussian(0.0, cfg_.gps_vel_noise_std_m_s),
                                         velocity_ned_m_s.y + noise_.gaussian(0.0, cfg_.gps_vel_noise_std_m_s),
                                         velocity_ned_m_s.z + noise_.gaussian(0.0, cfg_.gps_vel_noise_std_m_s));
  s.valid = true;
  last_gps_ = s;
  return s;
}

BaroSample Sensors::baro(double altitude_m) {
  const bool dropout = noise_.bernoulli(cfg_.baro_dropout_prob);
  if (dropout) {
    if (cfg_.hold_last_on_dropout) {
      BaroSample held = last_baro_;
      held.valid = false;
      return held;
    }
    BaroSample s{};
    s.valid = false;
    return s;
  }

  BaroSample s{};
  s.altitude_m = altitude_m + noise_.gaussian(0.0, cfg_.baro_alt_noise_std_m);
  s.valid = true;
  last_baro_ = s;
  return s;
}

}  // namespace acs::simulation
