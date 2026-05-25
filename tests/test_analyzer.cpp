#include <cassert>
#include <cmath>
#include <fstream>
#include <string>

#include "acs/analysis/Statistics.h"

namespace {
bool near(double a, double b, double eps = 1e-9) {
  return std::fabs(a - b) <= eps;
}
}  // namespace

int main() {
  const std::string path = "test_telemetry_analyzer_tmp.csv";

  {
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    assert(out.is_open());

    // Intentionally shuffled columns to verify header-based mapping.
    out << "alt_m,t_s,roll_rad,pitch_rad,yaw_rad,airspeed_mps,aileron_rad,elevator_rad,rudder_rad,p_rps,q_rps,r_rps,target_alt_m\n";
    out << "0,0,0,0,0,10,0,0,0,0,0,0,100\n";
    out << "80,1,0,0,0,10,0,0,0,0,0,0,100\n";
    out << "105,2,0,0,0,10,0,0,0,0,0,0,100\n";
    out << "100,3,0,0,0,10,0,0,0,0,0,0,100\n";
    out << "100,4,0,0,0,10,0,0,0,0,0,0,100\n";
  }

  const auto report = acs::analysis::TelemetryAnalyzer::analyze_csv(path);

  assert(report.target_altitude_from_telemetry);
  assert(near(report.target_altitude_m, 100.0));
  assert(near(report.steady_state_error, 0.0));
  assert(near(report.max_overshoot_pct, 5.0, 1e-6));
  assert(near(report.settling_time_s, 3.0, 1e-9));

  return 0;
}

