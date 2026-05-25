#pragma once

#include "acs/math/Matrix3.h"
#include "acs/math/Quaternion.h"
#include "acs/math/Vector3.h"

namespace acs::physics {

struct RigidBodyState {
  // navigation frame position (ned), meters. down positive.
  acs::math::Vector3 position_ned_m{};
  // body-frame velocity (frd), m/s.
  acs::math::Vector3 velocity_body_m_s{};
  // body angular rates [p q r], rad/s.
  acs::math::Vector3 omega_body_rad_s{};
  // quaternion rotating body -> ned
  acs::math::Quaternion q_nb{};

  RigidBodyState operator+(const RigidBodyState& other) const;
  RigidBodyState operator*(double s) const;
};

RigidBodyState operator*(double s, const RigidBodyState& x);

struct RigidBodyParams {
  double mass_kg{1.0};
  acs::math::Matrix3 inertia_kg_m2{acs::math::Matrix3::identity()};
  acs::math::Matrix3 inertia_inv{acs::math::Matrix3::identity()};
};

struct BodyForcesMoments {
  acs::math::Vector3 force_body_n{};
  acs::math::Vector3 moment_body_n_m{};
};

class RigidBody6Dof {
public:
  explicit RigidBody6Dof(const RigidBodyParams& params);

  const RigidBodyParams& params() const;

  // computes time derivative using newton-euler equations in body frame:
  // m*v_dot = F_b + m*g_b - omega x (m*v)
  // I*omega_dot = M_b - omega x (I*omega)
  RigidBodyState derivative(const RigidBodyState& x,
                            const BodyForcesMoments& fm,
                            const acs::math::Vector3& gravity_ned_m_s2) const;

private:
  RigidBodyParams params_{};
};

}  // namespace acs::physics

