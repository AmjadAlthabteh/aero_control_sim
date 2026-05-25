#include <chrono>
#include <cmath>
#include <iostream>

#include "acs/math/Constants.h"
#include "acs/math/Matrix3.h"
#include "acs/math/Quaternion.h"
#include "acs/math/Vector3.h"
#include "acs/physics/Atmosphere.h"
#include "acs/physics/Gravity.h"
#include "acs/simulation/Actuators.h"
#include "acs/simulation/Sensors.h"
#include "acs/simulation/Wind.h"
#include "acs/testing/TestFramework.h"

using namespace acs::testing;
using namespace acs::math;
using namespace acs::physics;

void test_vector3_operations() {
  TestSuite suite("Vector3 Operations");

  suite.add_test("Vector3 addition", [](TestResult& r) {
    Vector3 a(1.0, 2.0, 3.0);
    Vector3 b(4.0, 5.0, 6.0);
    Vector3 c = a + b;
    assert_near(r, 5.0, c.x, 1e-9);
    assert_near(r, 7.0, c.y, 1e-9);
    assert_near(r, 9.0, c.z, 1e-9);
  });

  suite.add_test("Vector3 dot product", [](TestResult& r) {
    Vector3 a(1.0, 0.0, 0.0);
    Vector3 b(0.0, 1.0, 0.0);
    double dot = Vector3::dot(a, b);
    assert_near(r, 0.0, dot, 1e-9, "Orthogonal vectors should have dot product 0");
  });

  suite.add_test("Vector3 cross product", [](TestResult& r) {
    Vector3 a(1.0, 0.0, 0.0);
    Vector3 b(0.0, 1.0, 0.0);
    Vector3 c = Vector3::cross(a, b);
    assert_near(r, 0.0, c.x, 1e-9);
    assert_near(r, 0.0, c.y, 1e-9);
    assert_near(r, 1.0, c.z, 1e-9, "X cross Y should equal Z");
  });

  suite.add_test("Vector3 normalization", [](TestResult& r) {
    Vector3 a(3.0, 4.0, 0.0);
    Vector3 normalized = a.normalized();
    assert_near(r, 1.0, normalized.norm(), 1e-9, "Normalized vector should have length 1");
  });

  suite.add_test("Vector3 magnitude", [](TestResult& r) {
    Vector3 a(3.0, 4.0, 0.0);
    assert_near(r, 5.0, a.norm(), 1e-9, "3-4-5 triangle");
  });

  suite.run();
}

void test_quaternion_operations() {
  TestSuite suite("Quaternion Operations");

  suite.add_test("Quaternion identity", [](TestResult& r) {
    Quaternion q = Quaternion::identity();
    assert_near(r, 1.0, q.w, 1e-9);
    assert_near(r, 0.0, q.x, 1e-9);
    assert_near(r, 0.0, q.y, 1e-9);
    assert_near(r, 0.0, q.z, 1e-9);
  });

  suite.add_test("Quaternion normalization", [](TestResult& r) {
    Quaternion q(1.0, 1.0, 1.0, 1.0);
    q.normalize_in_place();
    double norm_sq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    assert_near(r, 1.0, norm_sq, 1e-9, "Normalized quaternion should have unit magnitude");
  });

  suite.add_test("Quaternion from euler angles", [](TestResult& r) {
    using acs::math::kHalfPi;
    Quaternion q = Quaternion::from_euler321(0.0, 0.0, kHalfPi);  // 90 deg yaw
    Vector3 x_body(1.0, 0.0, 0.0);
    Vector3 x_ned = q.rotate(x_body);
    assert_near(r, 0.0, x_ned.x, 1e-6, "90 deg yaw should rotate X to Y");
    assert_near(r, 1.0, x_ned.y, 1e-6);
    assert_near(r, 0.0, x_ned.z, 1e-6);
  });

  suite.add_test("Quaternion conjugate", [](TestResult& r) {
    Quaternion q(1.0, 2.0, 3.0, 4.0);
    Quaternion q_conj = q.conjugate();
    assert_near(r, q.w, q_conj.w, 1e-9);
    assert_near(r, -q.x, q_conj.x, 1e-9);
    assert_near(r, -q.y, q_conj.y, 1e-9);
    assert_near(r, -q.z, q_conj.z, 1e-9);
  });

  suite.run();
}

void test_matrix3_operations() {
  TestSuite suite("Matrix3 Operations");

  suite.add_test("Matrix3 identity", [](TestResult& r) {
    Matrix3 m = Matrix3::identity();
    Vector3 v(1.0, 2.0, 3.0);
    Vector3 result = m * v;
    assert_near(r, v.x, result.x, 1e-9);
    assert_near(r, v.y, result.y, 1e-9);
    assert_near(r, v.z, result.z, 1e-9, "Identity matrix should not change vector");
  });

  suite.add_test("Matrix3 transpose", [](TestResult& r) {
    Matrix3 m = Matrix3::from_rows(
        Vector3(1.0, 2.0, 3.0),
        Vector3(4.0, 5.0, 6.0),
        Vector3(7.0, 8.0, 9.0));
    Matrix3 mt = m.transposed();
    assert_near(r, 1.0, mt.m[0][0], 1e-9);
    assert_near(r, 4.0, mt.m[0][1], 1e-9);
    assert_near(r, 7.0, mt.m[0][2], 1e-9);
  });

  suite.add_test("Matrix3 multiplication", [](TestResult& r) {
    Matrix3 m = Matrix3::identity();
    Vector3 v(5.0, 10.0, 15.0);
    Vector3 result = m * v;
    assert_near(r, 5.0, result.x, 1e-9);
    assert_near(r, 10.0, result.y, 1e-9);
    assert_near(r, 15.0, result.z, 1e-9);
  });

  suite.run();
}

void test_atmosphere_model() {
  TestSuite suite("Atmosphere Model");

  suite.add_test("Sea level atmosphere", [](TestResult& r) {
    auto atm = Atmosphere::at_altitude(0.0);
    assert_near(r, 1.225, atm.rho_kg_m3, 0.01, "Sea level density");
    assert_near(r, 101325.0, atm.pressure_pa, 100.0, "Sea level pressure");
    assert_near(r, 288.15, atm.temperature_k, 0.5, "Sea level temperature");
  });

  suite.add_test("Atmosphere decreases with altitude", [](TestResult& r) {
    auto atm0 = Atmosphere::at_altitude(0.0);
    auto atm1000 = Atmosphere::at_altitude(1000.0);
    assert_true(r, atm1000.rho_kg_m3 < atm0.rho_kg_m3, "Density decreases with altitude");
    assert_true(r, atm1000.pressure_pa < atm0.pressure_pa, "Pressure decreases with altitude");
  });

  suite.run();
}

void test_gravity_model() {
  TestSuite suite("Gravity Model");

  suite.add_test("Gravity at sea level", [](TestResult& r) {
    Vector3 g = Gravity::ned(0.0);
    assert_near(r, 0.0, g.x, 1e-9, "No north gravity component");
    assert_near(r, 0.0, g.y, 1e-9, "No east gravity component");
    assert_near(r, 9.81, g.z, 0.01, "Downward gravity ~9.81 m/s^2");
  });

  suite.add_test("Gravity decreases with altitude", [](TestResult& r) {
    Vector3 g0 = Gravity::ned(0.0);
    Vector3 g10k = Gravity::ned(10000.0);
    assert_true(r, g10k.z < g0.z, "Gravity decreases with altitude");
  });

  suite.run();
}

void test_performance_characteristics() {
  TestSuite suite("Performance Characteristics");

  suite.add_test("Vector operations are fast", [](TestResult& r) {
    auto start = std::chrono::high_resolution_clock::now();

    Vector3 sum(0.0, 0.0, 0.0);
    for (int i = 0; i < 1000000; ++i) {
      Vector3 a(i * 0.1, i * 0.2, i * 0.3);
      Vector3 b(i * 0.4, i * 0.5, i * 0.6);
      sum = sum + Vector3::cross(a, b);
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    assert_true(r, duration < 100, "1M vector ops should complete in < 100ms");
  });

  suite.add_test("Quaternion operations are fast", [](TestResult& r) {
    auto start = std::chrono::high_resolution_clock::now();

    Quaternion q = Quaternion::identity();
    for (int i = 0; i < 100000; ++i) {
      q = Quaternion::from_euler321(i * 0.001, i * 0.002, i * 0.003);
      q.normalize_in_place();
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    assert_true(r, duration < 50, "100K quaternion ops should complete in < 50ms");
  });

  suite.run();
}

void test_wind_and_actuators() {
  TestSuite suite("Wind & Actuators");

  suite.add_test("WindModel is deterministic for a seed", [](TestResult& r) {
    acs::simulation::WindConfig cfg{};
    cfg.mean_ned_m_s = Vector3(1.0, -2.0, 0.5);
    cfg.gust_std_dev_m_s = 0.7;
    cfg.gust_tau_s = 1.5;

    acs::simulation::WindModel w1(cfg, 123U);
    acs::simulation::WindModel w2(cfg, 123U);

    for (int i = 0; i < 50; ++i) {
      Vector3 a = w1.update(0.02);
      Vector3 b = w2.update(0.02);
      assert_near(r, a.x, b.x, 1e-12);
      assert_near(r, a.y, b.y, 1e-12);
      assert_near(r, a.z, b.z, 1e-12);
    }
  });

  suite.add_test("Actuator rate limiting bounds deflection change", [](TestResult& r) {
    acs::simulation::ActuatorConfig cfg{};
    cfg.surface_limit_rad = 10.0;
    cfg.surface_tau_s = 0.0;
    cfg.surface_rate_limit_rad_s = 1.0;  // 1 rad/s

    acs::simulation::Actuators act(cfg);
    act.reset();

    acs::aero::ControlInputs cmd{};
    cmd.aileron_rad = 5.0;
    cmd.elevator_rad = 0.0;
    cmd.rudder_rad = 0.0;
    cmd.throttle_01 = 0.0;

    const auto u0 = act.applied();
    const auto u1 = act.update(cmd, 0.1, 0.0);  // dt=0.1s => max 0.1 rad step

    assert_near(r, u0.aileron_rad + 0.1, u1.aileron_rad, 1e-12, "Rate limit should bound step size");
  });

  suite.add_test("GPS dropout marks invalid", [](TestResult& r) {
    acs::simulation::SensorConfig cfg{};
    cfg.gps_dropout_prob = 1.0;
    cfg.hold_last_on_dropout = false;
    acs::simulation::Sensors s(cfg, 1U);

    const auto gps = s.gps(Vector3(1.0, 2.0, 3.0), Vector3(0.1, 0.2, 0.3));
    assert_true(r, gps.valid == false, "GPS sample should be invalid under forced dropout");
  });

  suite.run();
}

int main() {
  std::cout << "\n";
  std::cout << "========================================\n";
  std::cout << "   ACS Unit Test Suite\n";
  std::cout << "========================================\n";

  test_vector3_operations();
  test_quaternion_operations();
  test_matrix3_operations();
  test_atmosphere_model();
  test_gravity_model();
  test_performance_characteristics();
  test_wind_and_actuators();

  std::cout << "========================================\n";
  std::cout << "   All Test Suites Complete\n";
  std::cout << "========================================\n\n";

  return 0;
}
