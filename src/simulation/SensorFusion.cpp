#include "acs/simulation/SensorFusion.h"

#include "acs/math/Quaternion.h"
#include "acs/physics/Gravity.h"

#include <algorithm>
#include <cmath>

namespace acs::simulation {
namespace {
double square(double value) { return value * value; }
}

SensorFusion::SensorFusion(const FusionConfig& config, const SensorConfig& sensors)
    : cfg_(config), sensor_cfg_(sensors) {}

void SensorFusion::reset(const acs::math::Vector3& position,
                         const acs::math::Vector3& velocity,
                         const acs::math::Vector3& euler) {
  x_.fill(0.0);
  x_[0] = position.x; x_[1] = position.y; x_[2] = position.z;
  x_[3] = velocity.x; x_[4] = velocity.y; x_[5] = velocity.z;
  for (auto& row : p_) row.fill(0.0);
  for (int i = 0; i < 3; ++i) p_[i][i] = square(cfg_.initial_position_std_m);
  for (int i = 3; i < 6; ++i) p_[i][i] = square(cfg_.initial_velocity_std_m_s);
  for (int i = 6; i < 9; ++i) p_[i][i] = square(cfg_.initial_accel_bias_std_m_s2);
  output_ = {};
  output_.roll_rad = euler.x; output_.pitch_rad = euler.y; output_.yaw_rad = euler.z;
  output_.initialized = true;
  refresh();
}

void SensorFusion::propagate_attitude(const ImuSample& imu, double dt) {
  const double phi = output_.roll_rad;
  const double theta = output_.pitch_rad;
  const double sin_phi = std::sin(phi);
  const double cos_phi = std::cos(phi);
  double cos_theta = std::cos(theta);
  if (std::abs(cos_theta) < 1e-6) cos_theta = std::copysign(1e-6, cos_theta);
  const double coupled_rate = imu.gyro_body_rad_s.y * sin_phi + imu.gyro_body_rad_s.z * cos_phi;
  const double gyro_roll = phi + (imu.gyro_body_rad_s.x + std::tan(theta) * coupled_rate) * dt;
  const double gyro_pitch = theta + (imu.gyro_body_rad_s.y * cos_phi - imu.gyro_body_rad_s.z * sin_phi) * dt;
  output_.yaw_rad += coupled_rate / cos_theta * dt;
  const auto& acceleration = imu.accel_body_m_s2;
  const double norm = acceleration.norm();
  if (norm > 4.9 && norm < 14.7) {
    const double alpha = cfg_.attitude_time_constant_s / (cfg_.attitude_time_constant_s + dt);
    const double accel_roll = std::atan2(-acceleration.y, -acceleration.z);
    const double accel_pitch = std::atan2(acceleration.x,
        std::sqrt(square(acceleration.y) + square(acceleration.z)));
    output_.roll_rad = alpha * gyro_roll + (1.0 - alpha) * accel_roll;
    output_.pitch_rad = alpha * gyro_pitch + (1.0 - alpha) * accel_pitch;
  } else {
    output_.roll_rad = gyro_roll;
    output_.pitch_rad = gyro_pitch;
  }
}

void SensorFusion::predict(const ImuSample& imu, double dt) {
  const auto attitude = acs::math::Quaternion::from_euler321(
      output_.roll_rad, output_.pitch_rad, output_.yaw_rad);
  const acs::math::Vector3 bias(x_[6], x_[7], x_[8]);
  const auto acceleration_ned = attitude.rotate(imu.accel_body_m_s2 - bias) +
                                acs::physics::Gravity::ned(-x_[2]);
  const double half_dt_squared = 0.5 * dt * dt;
  x_[0] += x_[3] * dt + acceleration_ned.x * half_dt_squared;
  x_[1] += x_[4] * dt + acceleration_ned.y * half_dt_squared;
  x_[2] += x_[5] * dt + acceleration_ned.z * half_dt_squared;
  x_[3] += acceleration_ned.x * dt;
  x_[4] += acceleration_ned.y * dt;
  x_[5] += acceleration_ned.z * dt;

  double transition[N][N]{};
  for (int i = 0; i < N; ++i) transition[i][i] = 1.0;
  const auto c_nb = attitude.to_dcm();
  for (int axis = 0; axis < 3; ++axis) {
    transition[axis][axis + 3] = dt;
    for (int bias_axis = 0; bias_axis < 3; ++bias_axis) {
      transition[axis][bias_axis + 6] = -0.5 * c_nb.m[axis][bias_axis] * dt * dt;
      transition[axis + 3][bias_axis + 6] = -c_nb.m[axis][bias_axis] * dt;
    }
  }
  Covariance transition_p{}, next{};
  for (int i = 0; i < N; ++i)
    for (int j = 0; j < N; ++j)
      for (int k = 0; k < N; ++k) transition_p[i][j] += transition[i][k] * p_[k][j];
  for (int i = 0; i < N; ++i)
    for (int j = 0; j < N; ++j)
      for (int k = 0; k < N; ++k) next[i][j] += transition_p[i][k] * transition[j][k];
  const double accel_variance = square(cfg_.accel_process_noise_m_s2);
  const double bias_variance = square(cfg_.accel_bias_process_noise_m_s2);
  for (int axis = 0; axis < 3; ++axis) {
    next[axis][axis] += 0.25 * accel_variance * dt * dt * dt * dt;
    next[axis + 3][axis + 3] += accel_variance * dt * dt;
    next[axis + 6][axis + 6] += bias_variance * dt;
  }
  p_ = next;
}

bool SensorFusion::correct(const std::array<double, N>& observation,
                           double residual, double measurement_variance) {
  std::array<double, N> covariance_observation{};
  for (int i = 0; i < N; ++i)
    for (int j = 0; j < N; ++j)
      covariance_observation[i] += p_[i][j] * observation[j];
  double innovation_variance = measurement_variance;
  for (int i = 0; i < N; ++i)
    innovation_variance += observation[i] * covariance_observation[i];
  if (!std::isfinite(innovation_variance) || innovation_variance <= 1e-12) return false;

  // Reject statistically implausible measurements before they can corrupt the estimate.
  if (square(residual) > square(cfg_.innovation_gate_sigma) * innovation_variance) return false;

  std::array<double, N> gain{};
  for (int i = 0; i < N; ++i) {
    gain[i] = covariance_observation[i] / innovation_variance;
    x_[i] += gain[i] * residual;
  }
  Covariance next = p_;
  for (int i = 0; i < N; ++i)
    for (int j = 0; j < N; ++j) next[i][j] -= gain[i] * covariance_observation[j];
  for (int i = 0; i < N; ++i) {
    for (int j = i + 1; j < N; ++j)
      next[i][j] = next[j][i] = 0.5 * (next[i][j] + next[j][i]);
    next[i][i] = std::max(next[i][i], 1e-12);
  }
  p_ = next;
  return true;
}

const FilteredState& SensorFusion::update(const ImuSample& imu, const GpsSample& gps,
                                          const AirDataSample& air_data,
                                          const acs::math::Vector3& wind, double dt) {
  if (!output_.initialized || !(dt > 0.0)) return output_;
  output_.gps_aiding = false;
  output_.air_data_aiding = false;
  propagate_attitude(imu, dt);
  predict(imu, dt);

  if (gps.valid) {
    double innovation_squared = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
      std::array<double, N> observation{};
      observation[axis] = 1.0;
      const double residual = gps.position_ned_m[axis] - x_[axis];
      innovation_squared += square(residual);
      const double noise = sensor_cfg_.gps_pos_noise_std_m * (axis == 2 ? 1.5 : 1.0);
      output_.gps_aiding |= correct(observation, residual, square(noise));
    }
    for (int axis = 0; axis < 3; ++axis) {
      std::array<double, N> observation{};
      observation[axis + 3] = 1.0;
      output_.gps_aiding |= correct(observation,
          gps.velocity_ned_m_s[axis] - x_[axis + 3], square(sensor_cfg_.gps_vel_noise_std_m_s));
    }
    output_.gps_innovation_m = std::sqrt(innovation_squared);
  }

  if (air_data.valid) {
    std::array<double, N> altitude_observation{};
    altitude_observation[2] = -1.0;
    output_.air_data_aiding = correct(altitude_observation, air_data.altitude_m + x_[2],
                                      square(sensor_cfg_.baro_alt_noise_std_m));
    const acs::math::Vector3 relative_velocity(
        x_[3] - wind.x, x_[4] - wind.y, x_[5] - wind.z);
    const double predicted_airspeed = relative_velocity.norm();
    output_.air_data_innovation_m_s = air_data.airspeed_m_s - predicted_airspeed;
    if (predicted_airspeed > 0.5) {
      std::array<double, N> observation{};
      observation[3] = relative_velocity.x / predicted_airspeed;
      observation[4] = relative_velocity.y / predicted_airspeed;
      observation[5] = relative_velocity.z / predicted_airspeed;
      output_.air_data_aiding |= correct(observation, output_.air_data_innovation_m_s,
                                         square(sensor_cfg_.airspeed_noise_std_m_s));
    }
  }
  refresh();
  output_.airspeed_m_s = acs::math::Vector3(
      x_[3] - wind.x, x_[4] - wind.y, x_[5] - wind.z).norm();
  return output_;
}

void SensorFusion::refresh() {
  output_.position_ned_m = {x_[0], x_[1], x_[2]};
  output_.velocity_ned_m_s = {x_[3], x_[4], x_[5]};
  output_.accel_bias_body_m_s2 = {x_[6], x_[7], x_[8]};
  output_.altitude_m = -x_[2];
  output_.position_std_m = std::sqrt(std::max(0.0, (p_[0][0] + p_[1][1] + p_[2][2]) / 3.0));
  output_.velocity_std_m_s = std::sqrt(std::max(0.0, (p_[3][3] + p_[4][4] + p_[5][5]) / 3.0));
}

}  // namespace acs::simulation
