#pragma once

#include "acs/math/Vector3.h"

namespace acs::aero {

struct ControlInputs {
  double aileron_rad{};   // +roll right
  double elevator_rad{};  // +pitch up (negative alpha)
  double rudder_rad{};    // +yaw right
  double throttle_01{};   // 0..1
};

struct AeroCoefficients {
  // force coefficients in stability axes with x aligned with body-x (frd forward).
  double cx{};  // positive forward
  double cy{};  // positive right
  double cz{};  // positive down

  // moment coefficients about body axes
  double cl{};  // roll
  double cm{};  // pitch
  double cn{};  // yaw
};

struct AeroForcesMoments {
  acs::math::Vector3 force_body_n{};
  acs::math::Vector3 moment_body_n_m{};
  double airspeed_m_s{};
  double alpha_rad{};
  double beta_rad{};
  double qbar_pa{};
  AeroCoefficients coeffs{};
};

struct AeroParams {
  double s_ref_m2{0.25};
  double b_ref_m{1.2};
  double c_ref_m{0.25};

  // propulsion model (simple)
  double max_thrust_n{15.0};

  // baseline coefficients
  double cx0{0.05};
  double cz0{0.0};
  double cy0{0.0};

  // lift/drag via cz (down positive in frd; lift is -z)
  double cz_alpha{-5.5};       // dCz/dalpha (negative gives lift up for +alpha)
  double cx_alpha2{0.8};       // induced drag vs alpha^2

  // lateral stability
  double cy_beta{0.98};

  // control derivatives
  double cl_da{0.08};
  double cm_de{-1.0};
  double cn_dr{0.06};

  // damping derivatives (scaled by p,q,r nondimensional rates)
  double cl_p{-0.5};
  double cm_q{-12.0};
  double cn_r{-0.35};

  // static stability
  double cm_alpha{-1.8};
  double cn_beta{0.06};
  double cl_beta{-0.08};

  // limits
  double aileron_limit_rad{0.35};
  double elevator_limit_rad{0.35};
  double rudder_limit_rad{0.35};
};

class AeroModel {
public:
  explicit AeroModel(const AeroParams& params);

  const AeroParams& params() const;

  // evaluates forces/moments in body frame given body velocity and rates.
  AeroForcesMoments evaluate(const acs::math::Vector3& velocity_body_m_s,
                             const acs::math::Vector3& omega_body_rad_s,
                             double rho_kg_m3,
                             const ControlInputs& u) const;

private:
  AeroParams params_{};
};

}  // namespace acs::aero

