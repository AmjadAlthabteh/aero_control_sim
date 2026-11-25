#include <cassert>
#include <cmath>
#include "acs/simulation/SensorFusion.h"

int main() {
  using namespace acs::simulation;
  SensorConfig sensors{};
  SensorFusion filter({}, sensors);
  filter.reset({0.0, 0.0, -100.0}, {20.0, 0.0, 0.0}, {0.0, 0.0, 0.0});

  ImuSample imu{};
  imu.accel_body_m_s2 = {0.0, 0.0, -9.80665};
  GpsSample gps{};
  AirDataSample air{};
  air.altitude_m = 100.0;
  air.airspeed_m_s = 20.0;
  for (int i = 0; i < 200; ++i) {
    gps.position_ned_m = {20.0 * (i + 1) * 0.01, 0.0, -100.0};
    gps.velocity_ned_m_s = {20.0, 0.0, 0.0};
    const auto& state = filter.update(imu, gps, air, {}, 0.01);
    assert(std::isfinite(state.position_std_m));
    assert(std::isfinite(state.velocity_std_m_s));
  }
  const auto before_outlier = filter.state();
  gps.position_ned_m = {10000.0, -10000.0, 5000.0};
  gps.velocity_ned_m_s = {500.0, -500.0, 300.0};
  air.altitude_m = 9000.0;
  air.airspeed_m_s = 700.0;
  const auto& rejected = filter.update(imu, gps, air, {}, 0.01);
  assert(!rejected.gps_aiding);
  assert(!rejected.air_data_aiding);
  assert(std::abs(rejected.position_ned_m.x - before_outlier.position_ned_m.x) < 1.0);
  assert(std::abs(rejected.altitude_m - before_outlier.altitude_m) < 1.0);

  const auto before_dropout = filter.state();
  gps.valid = false;
  air.valid = false;
  for (int i = 0; i < 50; ++i) filter.update(imu, gps, air, {}, 0.01);
  const auto& state = filter.state();
  assert(!state.gps_aiding && !state.air_data_aiding);
  assert(std::abs(state.altitude_m - 100.0) < 2.0);
  assert(std::abs(state.velocity_ned_m_s.x - 20.0) < 1.0);
  assert(state.position_std_m >= before_dropout.position_std_m);
}
