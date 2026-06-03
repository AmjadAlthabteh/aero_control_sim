#include "acs/aero/AeroModel.h"

#include <algorithm>
#include <cmath>

namespace acs::aero {

static double clamp(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }

// Critical AoA (~15 deg) and 5-deg transition band over which lift degrades to zero post-stall.
static constexpr double kAlphaStall_rad = 0.2618;
static constexpr double kAlphaStallBand_rad = 0.0873;

AeroModel::AeroModel(const AeroParams& params) : params_(params) {}

const AeroParams& AeroModel::params() const { return params_; }

AeroForcesMoments AeroModel::evaluate(const acs::math::Vector3& velocity_body_m_s,
                                      const acs::math::Vector3& omega_body_rad_s,
                                      double rho_kg_m3,
                                      const ControlInputs& u_in) const {
  using acs::math::Vector3;

  const double u = velocity_body_m_s.x;
  const double v = velocity_body_m_s.y;
  const double w = velocity_body_m_s.z;

  const double v2 = u * u + v * v + w * w;
  const double vmag = std::sqrt(std::max(1e-6, v2));

  // angle of attack and sideslip.
  // body frame is frd (x fwd, y right, z down).
  // alpha is defined positive for nose-up, so we flip the sign on w.
  const double alpha = std::atan2(-w, std::max(1e-6, u));
  const double beta = std::asin(clamp(v / vmag, -1.0, 1.0));

  const double qbar = 0.5 * rho_kg_m3 * v2;
  const double p = omega_body_rad_s.x;
  const double q = omega_body_rad_s.y;
  const double r = omega_body_rad_s.z;

  const double p_hat = (params_.b_ref_m / (2.0 * vmag)) * p;
  const double q_hat = (params_.c_ref_m / (2.0 * vmag)) * q;
  const double r_hat = (params_.b_ref_m / (2.0 * vmag)) * r;

  ControlInputs u_cmd = u_in;
  u_cmd.aileron_rad = clamp(u_cmd.aileron_rad, -params_.aileron_limit_rad, params_.aileron_limit_rad);
  u_cmd.elevator_rad = clamp(u_cmd.elevator_rad, -params_.elevator_limit_rad, params_.elevator_limit_rad);
  u_cmd.rudder_rad = clamp(u_cmd.rudder_rad, -params_.rudder_limit_rad, params_.rudder_limit_rad);
  u_cmd.throttle_01 = clamp(u_cmd.throttle_01, 0.0, 1.0);

  AeroCoefficients c{};

  // Stall: above critical AoA, lift degrades smoothly to zero over the transition band.
  // A pitch-down destabilisation (centre-of-pressure shift) grows proportionally so the
  // aircraft naturally pitches towards recovery — matching real stall behaviour.
  const double stall_excess = alpha - kAlphaStall_rad;
  const double stall_factor =
      (stall_excess > 0.0) ? clamp(1.0 - stall_excess / kAlphaStallBand_rad, 0.0, 1.0) : 1.0;
  const double cm_stall = -(1.0 - stall_factor) * 0.8;

  // simple aero model:
  // - cx is mostly drag, grows with alpha^2 (induced + form drag lumped)
  // - cz provides lift in the body z-axis (down positive), so lift is -cz
  // - cy is sideforce from sideslip
  c.cx = params_.cx0 + params_.cx_alpha2 * alpha * alpha;
  c.cy = params_.cy0 + params_.cy_beta * beta;
  c.cz = params_.cz0 + params_.cz_alpha * alpha * stall_factor;

  // moments (stability + control + rate damping).
  c.cl = params_.cl_beta * beta + params_.cl_da * u_cmd.aileron_rad + params_.cl_p * p_hat;
  c.cm = params_.cm_alpha * alpha + params_.cm_de * u_cmd.elevator_rad + params_.cm_q * q_hat + cm_stall;
  c.cn = params_.cn_beta * beta + params_.cn_dr * u_cmd.rudder_rad + params_.cn_r * r_hat;

  const Vector3 f_aero_b(qbar * params_.s_ref_m2 * c.cx,  //
                         qbar * params_.s_ref_m2 * c.cy,  //
                         qbar * params_.s_ref_m2 * c.cz);

  const Vector3 m_aero_b(qbar * params_.s_ref_m2 * params_.b_ref_m * c.cl,  //
                         qbar * params_.s_ref_m2 * params_.c_ref_m * c.cm,  //
                         qbar * params_.s_ref_m2 * params_.b_ref_m * c.cn);

  // propulsion: thrust along body x (forward). no propwash or torque modeled.
  const Vector3 f_thrust_b(params_.max_thrust_n * u_cmd.throttle_01, 0.0, 0.0);

  AeroForcesMoments out{};
  out.force_body_n = f_aero_b + f_thrust_b;
  out.moment_body_n_m = m_aero_b;
  out.airspeed_m_s = vmag;
  out.alpha_rad = alpha;
  out.beta_rad = beta;
  out.qbar_pa = qbar;
  out.coeffs = c;
  return out;
}

}  // namespace acs::aero

