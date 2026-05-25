#include "acs/simulation/Simulator.h"

#include <cmath>
#include <iomanip>
#include <sstream>

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

static acs::physics::RigidBodyState make_initial_state() {
  constexpr double kPi = 3.14159265358979323846;
  acs::physics::RigidBodyState x{};
  x.position_ned_m = acs::math::Vector3(0.0, 0.0, -60.0);  // 60 m altitude
  x.velocity_body_m_s = acs::math::Vector3(16.0, 0.6, 0.2);
  x.omega_body_rad_s = acs::math::Vector3(0.02, -0.01, 0.03);
  x.q_nb = acs::math::Quaternion::from_euler321(12.0 * kPi / 180.0, -6.0 * kPi / 180.0, 25.0 * kPi / 180.0);
  return x;
}

void Simulator::run(const acs::gnc::ControllerTargets& targets, const std::string& telemetry_path) {
  using acs::math::Matrix3;
  using acs::math::Vector3;
  using acs::physics::BodyForcesMoments;
  using acs::physics::RigidBodyState;

  const bool enable_telemetry = !telemetry_path.empty();
  const bool enable_profiling = enable_telemetry;

  if (enable_profiling) {
    auto& profiler = acs::profiling::PerformanceProfiler::instance();
    profiler.reset();
  }
  Telemetry log(enable_telemetry ? telemetry_path : "temp.csv");
  if (enable_telemetry) {
    log.write_header();
  }

  controller_.reset();
  sensors_.reset(sim_cfg_.seed);
  wind_.reset(sim_cfg_.seed + 1U);
  actuators_.reset();
  RigidBodyState x = make_initial_state();
  acs::aero::ControlInputs u_applied{};
  u_applied.throttle_01 = 0.55;

  const int steps = static_cast<int>(std::ceil(sim_cfg_.sim_time_s / sim_cfg_.dt_s));
  double t = 0.0;

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
      const Vector3 eul = x.q_nb.to_euler321();
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

      // use "perfect" attitude for now (like a fused ahrs), but still take yaw rate from the gyro.
      // this keeps the demo stable while still exercising the sensor code paths.
      const auto aero_for_imu = aircraft_.aero_model().evaluate(v_air_b, x.omega_body_rad_s, atm.rho_kg_m3, u_applied);
      const Vector3 specific_force = (1.0 / aircraft_.rigid_body().params().mass_kg) * aero_for_imu.force_body_n;
      const auto imu = sensors_.imu(specific_force, x.omega_body_rad_s, sim_cfg_.dt_s);

      acs::gnc::EstimatedState est{};
      est.roll_rad = eul.x;
      est.pitch_rad = eul.y;
      est.yaw_rad = eul.z;
      (void)gps;

      est.altitude_m = baro.altitude_m;
      est.airspeed_m_s = airspeed;
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
      const auto aero_eval = aircraft_.aero_model().evaluate(v_air_b_s, xs.omega_body_rad_s, atm_s.rho_kg_m3, u_hold);

      BodyForcesMoments fm{};
      fm.force_body_n = aero_eval.force_body_n;
      fm.moment_body_n_m = aero_eval.moment_body_n_m;
      return aircraft_.rigid_body().derivative(xs, fm, g_n_s);
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
    const Matrix3 c_nb_log = x.q_nb.to_dcm();
    const Matrix3 c_bn_log = c_nb_log.transposed();
    const Vector3 wind_b_log = c_bn_log * wind_n;
    const Vector3 v_air_b_log = x.velocity_body_m_s - wind_b_log;
    const auto aero_eval = aircraft_.aero_model().evaluate(v_air_b_log, x.omega_body_rad_s, atm.rho_kg_m3, u_hold);
    const Vector3 eul = x.q_nb.to_euler321();

    std::ostringstream row;
    row << std::fixed << std::setprecision(6);
    row << t << ",";
    row << x.position_ned_m.x << "," << x.position_ned_m.y << "," << x.position_ned_m.z << ",";
    row << x.velocity_body_m_s.x << "," << x.velocity_body_m_s.y << "," << x.velocity_body_m_s.z << ",";
    row << x.omega_body_rad_s.x << "," << x.omega_body_rad_s.y << "," << x.omega_body_rad_s.z << ",";
    row << eul.x << "," << eul.y << "," << eul.z << ",";
    row << (-x.position_ned_m.z) << "," << aero_eval.airspeed_m_s << "," << aero_eval.alpha_rad << "," << aero_eval.beta_rad << ",";
    row << u_hold.aileron_rad << "," << u_hold.elevator_rad << "," << u_hold.rudder_rad << "," << u_hold.throttle_01 << ",";
    row << aero_eval.force_body_n.x << "," << aero_eval.force_body_n.y << "," << aero_eval.force_body_n.z << ",";
    row << aero_eval.moment_body_n_m.x << "," << aero_eval.moment_body_n_m.y << "," << aero_eval.moment_body_n_m.z << ",";
    row << wind_n.x << "," << wind_n.y << "," << wind_n.z << ",";
    row << v_air_b_log.x << "," << v_air_b_log.y << "," << v_air_b_log.z << ",";
    row << u_cmd.aileron_rad << "," << u_cmd.elevator_rad << "," << u_cmd.rudder_rad << "," << u_cmd.throttle_01 << ",";
    row << (baro.valid ? 1 : 0) << "," << (gps.valid ? 1 : 0);
    if (enable_telemetry) {
      log.write_row(row.str());
    }

    t += sim_cfg_.dt_s;
  }

  if (enable_profiling) {
    auto& profiler = acs::profiling::PerformanceProfiler::instance();
    profiler.print_report();
  }
}

}  // namespace acs::simulation
