#pragma once

#include "acs/math/Matrix3.h"
#include "acs/math/Vector3.h"

namespace acs::math {

class Quaternion {
public:
  // w + xi + yj + zk
  double w{1.0};
  double x{};
  double y{};
  double z{};

  Quaternion() = default;
  Quaternion(double w_in, double x_in, double y_in, double z_in);

  static Quaternion identity();
  static Quaternion from_euler321(double roll_rad, double pitch_rad, double yaw_rad);

  double norm() const;
  Quaternion normalized() const;
  void normalize_in_place();

  Quaternion conjugate() const;
  Quaternion operator*(const Quaternion& other) const;

  // rotate vector from body to nav when this quaternion is q_nb
  Vector3 rotate(const Vector3& v) const;
  Matrix3 to_dcm() const;

  // omega_body is body angular rate [p q r] in rad/s.
  // q_dot = 0.5 * q ⊗ [0, omega_body]
  Quaternion derivative_from_body_rates(const Vector3& omega_body_rad_s) const;

  // 3-2-1 yaw-pitch-roll extraction for logging (roll, pitch, yaw)
  Vector3 to_euler321() const;
};

}  // namespace acs::math

