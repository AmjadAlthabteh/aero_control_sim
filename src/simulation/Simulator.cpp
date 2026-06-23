#include "acs/simulation/Simulator.h"

#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>

#include "acs/math/Constants.h"
#include "acs/physics/Atmosphere.h"
#include "acs/physics/Gravity.h"
#include "acs/physics/RK4.h"
#include "acs/profiling/PerformanceProfiler.h"

namespace acs::simulation {

Simulator::Simulator(const SimulatorConfig& sim_cfg,
                     const Aircraft& aircraft,
                     const acs::gnc::FlightController& controller_template,
                     const SensorConfig& sensor_cfg)
    : sim_cfg_(sim_cfg),
      aircraft_(aircraft),
      controller_(controller_template),
      sensors_(sensor_cfg, sim_cfg.seed),
      wind_(sim_cfg.wind, sim_cfg.seed + 1U),
      actuators_(sim_cfg.actuators, sim_cfg.actuator_faults) {}

// =============================================================================
// Complementary Filter AHRS
//
// A complementary filter splits attitude estimation between two sensors whose
// noise properties are complementary:
//
//   Gyroscope   — low noise, high bandwidth, but integrator drift grows as √t.
//   Accelerometer — high noise, but its time-averaged direction is always gravity,
//                   so it is drift-free in the long run.
//
// The filter is a first-order IIR with a single cross-over frequency 1/τ:
//
//   φ_next = α · (φ_prev + ω_body · dt)  +  (1 − α) · φ_accel
//          │                             │
//          └── gyro high-pass ───────────┴── accel low-pass
//
// where α = τ / (τ + dt)  (Tustin bilinear discretisation).
//
// With τ = 0.5 s the crossover is 0.32 Hz.  The gyro handles all dynamic
// manoeuvres; the accelerometer slowly corrects any accumulating gyro bias.
//
// Yaw is gyro-only because the accelerometer cannot sense rotation about the
// gravity vector (a magnetometer or GPS course would be needed to correct yaw
// drift, but neither is fused here).
//
// Accelerometer roll/pitch derivation (FRD body frame, g_ned = [0,0,g]):
//   Specific force ≈ −g_body during quasi-static flight, and g_body is:
//     g_bx = −g sin θ
//     g_by =  g cos θ sin φ
//     g_bz =  g cos θ cos φ
//   so specific force components are  fx = g sinθ,  fy = −g cosθ sinφ,
//   fz = −g cosθ cosφ.  Inverting:
//     φ = atan2(−fy, −fz)              (roll)
//     θ = atan2(fx,  √(fy² + fz²))    (pitch)
//
// The accelerometer blend is gated off when |a| is more than 50 % away from g,
// i.e. during hard manoeuvres where inertial acceleration dominates over
// gravity and the accel-derived angles would be wrong.
// =============================================================================

struct AhrsState {
  double roll_rad{};
  double pitch_rad{};
  double yaw_rad{};
};

static AhrsState ahrs_update(const AhrsState& prev, const ImuSample& imu, double dt_s) {
  constexpr double kTau_s       = 0.5;   // complementary filter time constant (seconds)
  constexpr double kG_lo_sq     = 4.9 * 4.9;    // squared 0.5 g trust gate
  constexpr double kG_hi_sq     = 14.7 * 14.7;  // squared 1.5 g trust gate

  const double alpha = kTau_s / (kTau_s + dt_s);

  // --- Gyro propagation -------------------------------------------------------
  // Body rates (p, q, r) are not exactly Euler rates except at zero pitch/yaw.
  // For the small-angle / low-pitch regime typical of fixed-wing UAVs the
  // approximation φ̇ ≈ p,  θ̇ ≈ q,  ψ̇ ≈ r is accurate to a few percent.
  const double roll_gyro  = prev.roll_rad  + imu.gyro_body_rad_s.x * dt_s;
  const double pitch_gyro = prev.pitch_rad + imu.gyro_body_rad_s.y * dt_s;
  const double yaw_gyro   = prev.yaw_rad   + imu.gyro_body_rad_s.z * dt_s;

  // --- Accelerometer-derived roll and pitch -----------------------------------
  const double ax = imu.accel_body_m_s2.x;
  const double ay = imu.accel_body_m_s2.y;
  const double az = imu.accel_body_m_s2.z;
  const double accel_norm_sq = ax * ax + ay * ay + az * az;

  AhrsState out{};
  out.yaw_rad = yaw_gyro;  // yaw is always gyro-only

  if (accel_norm_sq > kG_lo_sq && accel_norm_sq < kG_hi_sq) {
    // Accelerometer is plausibly sensing gravity — blend it in.
    const double roll_accel  = std::atan2(-ay, -az);
    const double pitch_accel = std::atan2(ax, std::sqrt(ay * ay + az * az));
    out.roll_rad  = alpha * roll_gyro  + (1.0 - alpha) * roll_accel;
    out.pitch_rad = alpha * pitch_gyro + (1.0 - alpha) * pitch_accel;
  } else {
    // Hard manoeuvre — inertial acceleration swamps gravity; trust gyro only.
    out.roll_rad  = roll_gyro;
    out.pitch_rad = pitch_gyro;
  }
  return out;
}

static acs::physics::RigidBodyState make_initial_state() {
  using acs::math::kDeg2Rad;
  acs::physics::RigidBodyState x{};
  x.position_ned_m = acs::math::Vector3(0.0, 0.0, -60.0);  // 60 m altitude
  x.velocity_body_m_s = acs::math::Vector3(16.0, 0.6, 0.2);
  x.omega_body_rad_s = acs::math::Vector3(0.02, -0.01, 0.03);
  x.q_nb = acs::math::Quaternion::from_euler321(12.0 * kDeg2Rad, -6.0 * kDeg2Rad, 25.0 * kDeg2Rad);
  return x;
}

void Simulator::run(const acs::gnc::ControllerTargets& targets, const std::string& telemetry_path) {
  using acs::math::Matrix3;
  using acs::math::Vector3;
  using acs::physics::BodyForcesMoments;
  using acs::physics::RigidBodyState;

  const auto& aero_model = aircraft_.aero_model();
  const auto& rigid_body = aircraft_.rigid_body();
  const double inverse_mass = 1.0 / rigid_body.params().mass_kg;
  const bool enable_telemetry = !telemetry_path.empty();
  const bool enable_profiling = enable_telemetry;

  if (enable_profiling) {
    auto& profiler = acs::profiling::PerformanceProfiler::instance();
    profiler.reset();
  }
  std::unique_ptr<Telemetry> log;
  if (enable_telemetry) {
    log = std::make_unique<Telemetry>(telemetry_path);
    log->write_header();
  }

  controller_.reset();
  sensors_.reset(sim_cfg_.seed);
  wind_.reset(sim_cfg_.seed + 1U);
  actuators_.reset();
  RigidBodyState x = make_initial_state();
  acs::aero::ControlInputs u_applied{};
  u_applied.throttle_01 = 0.55;

  // Seed the AHRS from the truth Euler angles so the filter starts converged.
  // In a real system this would come from an alignment procedure.
  const Vector3 eul_init = x.q_nb.to_euler321();
  AhrsState ahrs{eul_init.x, eul_init.y, eul_init.z};

  const int steps = static_cast<int>(std::ceil(sim_cfg_.sim_time_s / sim_cfg_.dt_s));
  double t = 0.0;
  std::ostringstream row;
  row << std::fixed << std::setprecision(6);

  for (int i = 0; i < steps; ++i) {
    if (enable_profiling) {
      auto& profiler = acs::profiling::PerformanceProfiler::instance();
      profiler.begin_section("simulation_step");
      profiler.record_frame();
    }
    const Matrix3 c_nb = x.q_nb.to_dcm();
    const Matrix3 c_bn = c_nb.transposed();
    const Vector3 v_n = c_nb * x.velocity_body_m_s;

    const double altitude_m = -x.position_ned_m.z;
    const auto atm = acs::physics::Atmosphere::at_altitude(altitude_m);
    const Vector3 wind_n = wind_.update(sim_cfg_.dt_s);
    const Vector3 wind_b = c_bn * wind_n;
    const Vector3 v_air_b = x.velocity_body_m_s - wind_b;

    // evaluate aero/thrust at the current state so we can generate sensor readings and control.
    // control inputs are held constant through the rk4 step.
    acs::aero::ControlInputs u_cmd{};
    acs::aero::ControlInputs u_hold{};
    GpsSample gps{};
    BaroSample baro{};
    {
      PROFILE_SCOPE("control_update");
      const double airspeed = v_air_b.norm();

      if (enable_profiling) {
        auto& profiler = acs::profiling::PerformanceProfiler::instance();
        profiler.begin_section("sensors");
      }
      baro = sensors_.baro(altitude_m);
      gps = sensors_.gps(x.position_ned_m, v_n);
      if (enable_profiling) {
        auto& profiler = acs::profiling::PerformanceProfiler::instance();
        profiler.end_section("sensors");
      }

      // Feed the IMU with the aero-derived specific force so the accelerometer
      // sees realistic inertial loading (thrust, lift, drag) rather than just gravity.
      const auto aero_for_imu = aero_model.evaluate(v_air_b, x.omega_body_rad_s, atm.rho_kg_m3, u_applied);
      const Vector3 specific_force = inverse_mass * aero_for_imu.force_body_n;
      const auto imu = sensors_.imu(specific_force, x.omega_body_rad_s, sim_cfg_.dt_s);

      // Run the complementary filter AHRS.  The estimated attitude now comes
      // from sensor fusion rather than truth state, so gyro bias and accel
      // noise from the sensor model feed through to the controller — the
      // simulation exercises a realistic closed-loop sensing chain.
      ahrs = ahrs_update(ahrs, imu, sim_cfg_.dt_s);
      acs::gnc::EstimatedState est{};
      est.roll_rad  = ahrs.roll_rad;
      est.pitch_rad = ahrs.pitch_rad;
      est.yaw_rad   = ahrs.yaw_rad;
      (void)gps;

      est.altitude_m     = baro.altitude_m;
      est.airspeed_m_s   = airspeed;
      est.yaw_rate_rad_s = imu.gyro_body_rad_s.z;

      if (enable_profiling) {
        auto& profiler = acs::profiling::PerformanceProfiler::instance();
        profiler.begin_section("controller");
      }
      u_cmd = controller_.update(targets, est, sim_cfg_.dt_s);
      if (enable_profiling) {
        auto& profiler = acs::profiling::PerformanceProfiler::instance();
        profiler.end_section("controller");
      }
    }

    u_hold = actuators_.update(u_cmd, sim_cfg_.dt_s, t);

    auto deriv = [&](const RigidBodyState& xs) -> RigidBodyState {
      const double alt_s = -xs.position_ned_m.z;
      const auto atm_s = acs::physics::Atmosphere::at_altitude(alt_s);
      const Vector3 g_n_s = acs::physics::Gravity::ned(alt_s);

      const Matrix3 c_nb_s = xs.q_nb.to_dcm();
      const Matrix3 c_bn_s = c_nb_s.transposed();
      const Vector3 wind_b_s = c_bn_s * wind_n;
      const Vector3 v_air_b_s = xs.velocity_body_m_s - wind_b_s;
      const auto aero_eval = aero_model.evaluate(v_air_b_s, xs.omega_body_rad_s, atm_s.rho_kg_m3, u_hold);

      BodyForcesMoments fm{};
      fm.force_body_n = aero_eval.force_body_n;
      fm.moment_body_n_m = aero_eval.moment_body_n_m;
      return rigid_body.derivative(xs, fm, g_n_s);
    };

    if (enable_profiling) {
      auto& profiler = acs::profiling::PerformanceProfiler::instance();
      profiler.begin_section("physics_integration");
    }
    const RigidBodyState x_next = acs::physics::RK4<RigidBodyState>::step(x, sim_cfg_.dt_s, deriv);
    x = x_next;
    x.q_nb.normalize_in_place();
    if (enable_profiling) {
      auto& profiler = acs::profiling::PerformanceProfiler::instance();
      profiler.end_section("physics_integration");
      profiler.increment_counter("physics_steps", 1);
      profiler.end_section("simulation_step");
    }

    u_applied = u_hold;

    // log telemetry using forces/moments at the updated state (purely for nicer plots).
    if (enable_telemetry) {
      const Matrix3 c_nb_log = x.q_nb.to_dcm();
      const Matrix3 c_bn_log = c_nb_log.transposed();
      const Vector3 wind_b_log = c_bn_log * wind_n;
      const Vector3 v_air_b_log = x.velocity_body_m_s - wind_b_log;
      const auto atm_log = acs::physics::Atmosphere::at_altitude(-x.position_ned_m.z);
      const auto aero_eval =
          aero_model.evaluate(v_air_b_log, x.omega_body_rad_s, atm_log.rho_kg_m3, u_hold);
      const Vector3 eul = x.q_nb.to_euler321();

      row.str(std::string{});
      row.clear();
      row << t << ",";
      row << x.position_ned_m.x << "," << x.position_ned_m.y << "," << x.position_ned_m.z << ",";
      row << x.velocity_body_m_s.x << "," << x.velocity_body_m_s.y << "," << x.velocity_body_m_s.z << ",";
      row << x.omega_body_rad_s.x << "," << x.omega_body_rad_s.y << "," << x.omega_body_rad_s.z << ",";
      row << eul.x << "," << eul.y << "," << eul.z << ",";
      row << (-x.position_ned_m.z) << "," << aero_eval.airspeed_m_s << "," << aero_eval.alpha_rad << "," << aero_eval.beta_rad
          << ",";
      row << u_hold.aileron_rad << "," << u_hold.elevator_rad << "," << u_hold.rudder_rad << "," << u_hold.throttle_01 << ",";
      row << aero_eval.force_body_n.x << "," << aero_eval.force_body_n.y << "," << aero_eval.force_body_n.z << ",";
      row << aero_eval.moment_body_n_m.x << "," << aero_eval.moment_body_n_m.y << "," << aero_eval.moment_body_n_m.z << ",";
      row << wind_n.x << "," << wind_n.y << "," << wind_n.z << ",";
      row << v_air_b_log.x << "," << v_air_b_log.y << "," << v_air_b_log.z << ",";
      row << u_cmd.aileron_rad << "," << u_cmd.elevator_rad << "," << u_cmd.rudder_rad << "," << u_cmd.throttle_01 << ",";
      row << (baro.valid ? 1 : 0) << "," << (gps.valid ? 1 : 0) << ",";
      row << targets.altitude_m << "," << targets.heading_rad << "," << targets.airspeed_m_s << "," << targets.roll_rad << ","
          << targets.pitch_rad;
      log->write_row(row.str());
    }

    t += sim_cfg_.dt_s;
  }

  if (enable_profiling) {
    auto& profiler = acs::profiling::PerformanceProfiler::instance();
    profiler.print_report();
  }
}

}  // namespace acs::simulation
