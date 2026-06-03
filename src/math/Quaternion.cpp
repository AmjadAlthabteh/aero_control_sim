#include "acs/math/Quaternion.h"

#include <algorithm>
#include <cmath>

namespace acs::math {

Quaternion::Quaternion(double w_in, double x_in, double y_in, double z_in) : w(w_in), x(x_in), y(y_in), z(z_in) {}

Quaternion Quaternion::identity() { return Quaternion(1.0, 0.0, 0.0, 0.0); }

Quaternion Quaternion::from_euler321(double roll_rad, double pitch_rad, double yaw_rad) {
  // 3-2-1: yaw about z, pitch about y, roll about x.
  const double cr = std::cos(0.5 * roll_rad);
  const double sr = std::sin(0.5 * roll_rad);
  const double cp = std::cos(0.5 * pitch_rad);
  const double sp = std::sin(0.5 * pitch_rad);
  const double cy = std::cos(0.5 * yaw_rad);
  const double sy = std::sin(0.5 * yaw_rad);

  Quaternion q{};
  q.w = cy * cp * cr + sy * sp * sr;
  q.x = cy * cp * sr - sy * sp * cr;
  q.y = cy * sp * cr + sy * cp * sr;
  q.z = sy * cp * cr - cy * sp * sr;
  return q.normalized();
}

double Quaternion::norm() const { return std::sqrt(w * w + x * x + y * y + z * z); }

Quaternion Quaternion::normalized() const {
  const double n = norm();
  if (!std::isfinite(n) || n <= 0.0) {
    return Quaternion::identity();
  }
  return Quaternion(w / n, x / n, y / n, z / n);
}

void Quaternion::normalize_in_place() {
  // optimization: only normalize when norm deviates significantly from 1.0
  // this avoids expensive sqrt when quaternion is already nearly normalized
  constexpr double kNormThreshold = 1e-6;
  const double norm_sq = w * w + x * x + y * y + z * z;
  if (!std::isfinite(norm_sq)) {
    w = 1.0;
    x = y = z = 0.0;
    return;
  }
  const double norm_error = std::abs(norm_sq - 1.0);

  if (norm_error < kNormThreshold) {
    return;  // already normalized within tolerance
  }

  const double n = std::sqrt(norm_sq);
  if (!std::isfinite(n) || n <= 0.0) {
    w = 1.0;
    x = y = z = 0.0;
    return;
  }
  w /= n;
  x /= n;
  y /= n;
  z /= n;
}

Quaternion Quaternion::conjugate() const { return Quaternion(w, -x, -y, -z); }

Quaternion Quaternion::operator*(const Quaternion& o) const {
  // hamilton product.
  return Quaternion(w * o.w - x * o.x - y * o.y - z * o.z,  //
                    w * o.x + x * o.w + y * o.z - z * o.y,  //
                    w * o.y - x * o.z + y * o.w + z * o.x,  //
                    w * o.z + x * o.y - y * o.x + z * o.w);
}

Vector3 Quaternion::rotate(const Vector3& v) const {
  // v' = q * [0,v] * q_conj, implemented without building temporary quaternions.
  // t = 2 * (q_xyz x v)
  // v' = v + w * t + (q_xyz x t)
  const Vector3 q_xyz(x, y, z);
  const Vector3 t = 2.0 * Vector3::cross(q_xyz, v);
  return v + (w * t) + Vector3::cross(q_xyz, t);
}

Matrix3 Quaternion::to_dcm() const {
  // direction cosine matrix for body->nav rotation.
  const double ww = w * w;
  const double xx = x * x;
  const double yy = y * y;
  const double zz = z * z;

  Matrix3 c{};
  c.m[0][0] = ww + xx - yy - zz;
  c.m[0][1] = 2.0 * (x * y - w * z);
  c.m[0][2] = 2.0 * (x * z + w * y);

  c.m[1][0] = 2.0 * (x * y + w * z);
  c.m[1][1] = ww - xx + yy - zz;
  c.m[1][2] = 2.0 * (y * z - w * x);

  c.m[2][0] = 2.0 * (x * z - w * y);
  c.m[2][1] = 2.0 * (y * z + w * x);
  c.m[2][2] = ww - xx - yy + zz;
  return c;
}

Quaternion Quaternion::derivative_from_body_rates(const Vector3& omega_body_rad_s) const {
  // q_dot = (1/2) q ⊗ Ω   where  Ω = [0, p, q, r]  (pure quaternion of body rates)
  //
  // Expanding the Hamilton product against a pure quaternion (zero scalar part)
  // analytically eliminates the four multiplications by Ω.w = 0 and removes
  // the temporary Quaternion object.  This function is called four times per
  // RK4 step, so keeping it lean matters.
  //
  // Derivation — Hamilton product (w,x,y,z) ⊗ (0,p,q,r):
  //   result.w =  0·w − p·x − q·y − r·z  =  −(x·p + y·q + z·r)
  //   result.x =  p·w + 0·x + r·y − q·z  =    w·p − z·q + y·r
  //   result.y =  q·w − r·x + 0·y + p·z  =    z·p + w·q − x·r
  //   result.z =  r·w + q·x − p·y + 0·z  =  −(y·p − x·q − w·r)
  //
  // (p, q, r = roll, pitch, yaw body rates in standard flight-dynamics notation)
  const double p = omega_body_rad_s.x;
  const double q = omega_body_rad_s.y;
  const double r = omega_body_rad_s.z;
  return Quaternion(
      0.5 * (-x * p - y * q - z * r),   // w
      0.5 * ( w * p - z * q + y * r),   // x
      0.5 * ( z * p + w * q - x * r),   // y
      0.5 * (-y * p + x * q + w * r));  // z
}

Vector3 Quaternion::to_euler321() const {
  // yaw-pitch-roll, returns [roll, pitch, yaw].
  const double sinr_cosp = 2.0 * (w * x + y * z);
  const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  const double roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (w * y - z * x);
  const double pitch = std::asin(std::clamp(sinp, -1.0, 1.0));

  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  const double yaw = std::atan2(siny_cosp, cosy_cosp);

  return Vector3(roll, pitch, yaw);
}

}  // namespace acs::math
