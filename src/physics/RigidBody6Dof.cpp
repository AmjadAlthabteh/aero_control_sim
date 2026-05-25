#include "acs/physics/RigidBody6Dof.h"

namespace acs::physics {

RigidBodyState RigidBodyState::operator+(const RigidBodyState& o) const {
  RigidBodyState r{};
  r.position_ned_m = position_ned_m + o.position_ned_m;
  r.velocity_body_m_s = velocity_body_m_s + o.velocity_body_m_s;
  r.omega_body_rad_s = omega_body_rad_s + o.omega_body_rad_s;
  r.q_nb = acs::math::Quaternion(q_nb.w + o.q_nb.w, q_nb.x + o.q_nb.x, q_nb.y + o.q_nb.y, q_nb.z + o.q_nb.z);
  return r;
}

RigidBodyState RigidBodyState::operator*(double s) const {
  RigidBodyState r{};
  r.position_ned_m = position_ned_m * s;
  r.velocity_body_m_s = velocity_body_m_s * s;
  r.omega_body_rad_s = omega_body_rad_s * s;
  r.q_nb = acs::math::Quaternion(q_nb.w * s, q_nb.x * s, q_nb.y * s, q_nb.z * s);
  return r;
}

RigidBodyState operator*(double s, const RigidBodyState& x) { return x * s; }

RigidBody6Dof::RigidBody6Dof(const RigidBodyParams& params) : params_(params) {}

const RigidBodyParams& RigidBody6Dof::params() const { return params_; }

RigidBodyState RigidBody6Dof::derivative(const RigidBodyState& x,
                                         const BodyForcesMoments& fm,
                                         const acs::math::Vector3& gravity_ned_m_s2) const {
  using acs::math::Matrix3;
  using acs::math::Vector3;

  const Matrix3 c_nb = x.q_nb.to_dcm();        // body -> nav
  const Matrix3 c_bn = c_nb.transposed();      // nav -> body

  const Vector3 v_b = x.velocity_body_m_s;
  const Vector3 w_b = x.omega_body_rad_s;

  // translational kinematics: p_dot = v_nav = c_nb * v_body
  const Vector3 p_dot_n = c_nb * v_b;

  // gravity in body frame
  const Vector3 g_b = c_bn * gravity_ned_m_s2;

  // newton in body frame:
  // m * v_dot + omega x (m*v) = F_b + m*g_b
  // -> v_dot = F_b/m + g_b - omega x v
  const Vector3 v_dot_b = (1.0 / params_.mass_kg) * fm.force_body_n + g_b - Vector3::cross(w_b, v_b);

  // euler's rigid body equation:
  // I*omega_dot + omega x (I*omega) = M
  const Vector3 h_b = params_.inertia_kg_m2 * w_b;
  const Vector3 w_dot_b = params_.inertia_inv * (fm.moment_body_n_m - Vector3::cross(w_b, h_b));

  // quaternion attitude propagation from body rates.
  const acs::math::Quaternion q_dot = x.q_nb.derivative_from_body_rates(w_b);

  RigidBodyState xdot{};
  xdot.position_ned_m = p_dot_n;
  xdot.velocity_body_m_s = v_dot_b;
  xdot.omega_body_rad_s = w_dot_b;
  xdot.q_nb = q_dot;
  return xdot;
}

}  // namespace acs::physics

