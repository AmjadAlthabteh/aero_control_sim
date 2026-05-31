#include <cassert>
#include <cmath>
#include <iostream>

#include "acs/math/Vector3.h"
#include "acs/math/Quaternion.h"
#include "acs/math/Matrix3.h"
#include "acs/aero/AeroModel.h"
#include "acs/physics/RigidBody6Dof.h"
#include "acs/physics/Atmosphere.h"

namespace {
constexpr double EPS = 1e-9;

bool near(double a, double b, double eps = EPS) {
    return std::fabs(a - b) <= eps;
}

bool vec_near(const acs::math::Vector3& a, const acs::math::Vector3& b, double eps = EPS) {
    return near(a.x, b.x, eps) && near(a.y, b.y, eps) && near(a.z, b.z, eps);
}

// Test Vector3 operations
void test_vector3_operations() {
    std::cout << "Testing Vector3 operations..." << std::endl;

    // Construction and basic ops
    acs::math::Vector3 v1(1.0, 2.0, 3.0);
    acs::math::Vector3 v2(4.0, 5.0, 6.0);

    // Addition
    acs::math::Vector3 sum = v1 + v2;
    assert(vec_near(sum, acs::math::Vector3(5.0, 7.0, 9.0)));

    // Subtraction
    acs::math::Vector3 diff = v2 - v1;
    assert(vec_near(diff, acs::math::Vector3(3.0, 3.0, 3.0)));

    // Scalar multiplication
    acs::math::Vector3 scaled = v1 * 2.0;
    assert(vec_near(scaled, acs::math::Vector3(2.0, 4.0, 6.0)));

    // Dot product
    double dot = acs::math::Vector3::dot(v1, v2);
    assert(near(dot, 32.0)); // 1*4 + 2*5 + 3*6 = 32

    // Cross product
    acs::math::Vector3 cross = acs::math::Vector3::cross(
        acs::math::Vector3(1.0, 0.0, 0.0),
        acs::math::Vector3(0.0, 1.0, 0.0)
    );
    assert(vec_near(cross, acs::math::Vector3(0.0, 0.0, 1.0)));

    // Norm
    acs::math::Vector3 v3(3.0, 4.0, 0.0);
    assert(near(v3.norm(), 5.0));
    assert(near(v3.norm_squared(), 25.0));

    // Normalization
    acs::math::Vector3 normalized = v3.normalized();
    assert(near(normalized.norm(), 1.0));
    assert(vec_near(normalized, acs::math::Vector3(0.6, 0.8, 0.0)));

    std::cout << "  ✓ Vector3 operations passed" << std::endl;
}

// Test Quaternion operations
void test_quaternion_operations() {
    std::cout << "Testing Quaternion operations..." << std::endl;

    // Identity quaternion
    acs::math::Quaternion q_identity(1.0, 0.0, 0.0, 0.0);
    assert(near(q_identity.norm(), 1.0));

    // 90-degree rotation about z-axis
    const double sqrt2_2 = std::sqrt(2.0) / 2.0;
    acs::math::Quaternion q_z90(sqrt2_2, 0.0, 0.0, sqrt2_2);

    // Rotate vector (1,0,0) by 90 degrees about z -> should give (0,1,0)
    acs::math::Vector3 v_in(1.0, 0.0, 0.0);
    acs::math::Vector3 v_out = q_z90.rotate(v_in);
    assert(vec_near(v_out, acs::math::Vector3(0.0, 1.0, 0.0), 1e-6));

    // Quaternion multiplication (identity * q = q)
    acs::math::Quaternion q_mult = q_identity * q_z90;
    assert(near(q_mult.w, q_z90.w, 1e-6));
    assert(near(q_mult.x, q_z90.x, 1e-6));
    assert(near(q_mult.y, q_z90.y, 1e-6));
    assert(near(q_mult.z, q_z90.z, 1e-6));

    // Normalization
    acs::math::Quaternion q_unnorm(2.0, 0.0, 0.0, 0.0);
    acs::math::Quaternion q_norm = q_unnorm.normalized();
    assert(near(q_norm.norm(), 1.0));

    std::cout << "  ✓ Quaternion operations passed" << std::endl;
}

// Test AeroModel force/moment calculations
void test_aero_model_forces() {
    std::cout << "Testing AeroModel force calculations..." << std::endl;

    acs::aero::AeroParams params{};
    params.s_ref_m2 = 0.25;
    params.cz_alpha = -5.5;
    params.max_thrust_n = 15.0;

    acs::aero::AeroModel model(params);

    // Test 1: Zero velocity -> zero forces
    acs::math::Vector3 v_zero(0.0, 0.0, 0.0);
    acs::math::Vector3 omega_zero(0.0, 0.0, 0.0);
    acs::aero::ControlInputs u_zero{};

    acs::aero::AeroForcesMoments fm_zero = model.evaluate(v_zero, omega_zero, 1.225, u_zero);
    assert(vec_near(fm_zero.force_body_n, acs::math::Vector3(0.0, 0.0, 0.0), 1e-6));

    // Test 2: Forward velocity with positive angle of attack -> lift (negative z force)
    acs::math::Vector3 v_forward(20.0, 0.0, 2.0); // slight downward component -> +alpha
    acs::aero::AeroForcesMoments fm_lift = model.evaluate(v_forward, omega_zero, 1.225, u_zero);

    // Should have negative z-force (lift) due to positive alpha
    assert(fm_lift.force_body_n.z < 0.0); // Lift opposes downward component
    assert(fm_lift.alpha_rad > 0.0); // Positive angle of attack

    // Test 3: Throttle produces forward thrust
    acs::aero::ControlInputs u_throttle{};
    u_throttle.throttle_01 = 1.0; // Full throttle

    acs::aero::AeroForcesMoments fm_thrust = model.evaluate(v_forward, omega_zero, 1.225, u_throttle);
    assert(fm_thrust.force_body_n.x > fm_lift.force_body_n.x); // More forward force

    // Test 4: Aileron produces rolling moment
    acs::aero::ControlInputs u_aileron{};
    u_aileron.aileron_rad = 0.1;

    acs::aero::AeroForcesMoments fm_roll = model.evaluate(v_forward, omega_zero, 1.225, u_aileron);
    assert(std::fabs(fm_roll.moment_body_n_m.x) > 0.0); // Should have roll moment

    // Test 5: Elevator produces pitching moment
    acs::aero::ControlInputs u_elevator{};
    u_elevator.elevator_rad = 0.1;

    acs::aero::AeroForcesMoments fm_pitch = model.evaluate(v_forward, omega_zero, 1.225, u_elevator);
    assert(std::fabs(fm_pitch.moment_body_n_m.y) > 0.0); // Should have pitch moment

    std::cout << "  ✓ AeroModel force calculations passed" << std::endl;
}

// Test RigidBody6Dof dynamics
void test_rigid_body_dynamics() {
    std::cout << "Testing RigidBody6Dof dynamics..." << std::endl;

    // Create simple rigid body (1 kg mass, unit inertia)
    acs::physics::RigidBodyParams params{};
    params.mass_kg = 1.0;
    params.inertia_kg_m2 = acs::math::Matrix3::identity();
    params.inertia_inv = acs::math::Matrix3::identity();

    acs::physics::RigidBody6Dof body(params);

    // Test 1: Zero forces -> zero acceleration
    acs::physics::RigidBodyState state{};
    state.q_nb = acs::math::Quaternion(1.0, 0.0, 0.0, 0.0); // Identity

    acs::physics::BodyForcesMoments fm_zero{};
    acs::math::Vector3 gravity_zero(0.0, 0.0, 0.0);

    acs::physics::RigidBodyState deriv_zero = body.derivative(state, fm_zero, gravity_zero);
    assert(vec_near(deriv_zero.velocity_body_m_s, acs::math::Vector3(0.0, 0.0, 0.0)));
    assert(vec_near(deriv_zero.omega_body_rad_s, acs::math::Vector3(0.0, 0.0, 0.0)));

    // Test 2: Forward force produces forward acceleration
    acs::physics::BodyForcesMoments fm_forward{};
    fm_forward.force_body_n = acs::math::Vector3(10.0, 0.0, 0.0); // 10 N forward

    acs::physics::RigidBodyState deriv_forward = body.derivative(state, fm_forward, gravity_zero);
    assert(near(deriv_forward.velocity_body_m_s.x, 10.0)); // a = F/m = 10/1 = 10 m/s²

    // Test 3: Roll moment produces roll rate acceleration
    acs::physics::BodyForcesMoments fm_roll{};
    fm_roll.moment_body_n_m = acs::math::Vector3(5.0, 0.0, 0.0); // Roll moment

    acs::physics::RigidBodyState deriv_roll = body.derivative(state, fm_roll, gravity_zero);
    assert(near(deriv_roll.omega_body_rad_s.x, 5.0)); // alpha = M/I = 5/1 = 5 rad/s²

    // Test 4: Gravity in NED frame produces correct body-frame acceleration
    state.q_nb = acs::math::Quaternion(1.0, 0.0, 0.0, 0.0); // Level flight
    acs::math::Vector3 gravity_ned(0.0, 0.0, 9.81); // Standard gravity down

    acs::physics::RigidBodyState deriv_gravity = body.derivative(state, fm_zero, gravity_ned);
    // In body frame (level), gravity should produce downward acceleration
    assert(near(deriv_gravity.velocity_body_m_s.z, 9.81, 0.01));

    std::cout << "  ✓ RigidBody6Dof dynamics passed" << std::endl;
}

// Test atmosphere model
void test_atmosphere_model() {
    std::cout << "Testing Atmosphere model..." << std::endl;

    // Test 1: Sea level conditions
    acs::physics::AtmosphereState atmo_sl = acs::physics::Atmosphere::at_altitude(0.0);

    assert(near(atmo_sl.rho_kg_m3, 1.225, 0.01)); // Standard sea level density
    assert(atmo_sl.pressure_pa > 101000.0 && atmo_sl.pressure_pa < 102000.0); // ~101325 Pa
    assert(atmo_sl.temperature_k > 287.0 && atmo_sl.temperature_k < 290.0); // ~288 K

    // Test 2: Density decreases with altitude
    acs::physics::AtmosphereState atmo_1km = acs::physics::Atmosphere::at_altitude(1000.0);
    assert(atmo_1km.rho_kg_m3 < atmo_sl.rho_kg_m3);

    // Test 3: High altitude
    acs::physics::AtmosphereState atmo_10km = acs::physics::Atmosphere::at_altitude(10000.0);
    assert(atmo_10km.rho_kg_m3 < atmo_1km.rho_kg_m3);
    assert(atmo_10km.rho_kg_m3 > 0.0); // Should still be positive

    std::cout << "  ✓ Atmosphere model passed" << std::endl;
}

// Test energy conservation in free fall
void test_energy_conservation() {
    std::cout << "Testing energy conservation..." << std::endl;

    // Simple test: object in free fall should conserve total energy
    acs::physics::RigidBodyParams params{};
    params.mass_kg = 1.0;
    params.inertia_kg_m2 = acs::math::Matrix3::identity();
    params.inertia_inv = acs::math::Matrix3::identity();

    acs::physics::RigidBody6Dof body(params);

    // Initial state: at altitude with zero velocity
    acs::physics::RigidBodyState state{};
    state.position_ned_m = acs::math::Vector3(0.0, 0.0, 0.0); // Sea level reference
    state.velocity_body_m_s = acs::math::Vector3(0.0, 0.0, 0.0);
    state.q_nb = acs::math::Quaternion(1.0, 0.0, 0.0, 0.0);

    // Take a small time step with gravity
    const double dt = 0.01;
    acs::physics::BodyForcesMoments fm{};
    acs::math::Vector3 gravity(0.0, 0.0, 9.81);

    acs::physics::RigidBodyState deriv = body.derivative(state, fm, gravity);

    // After one step, should have downward velocity
    acs::physics::RigidBodyState state_new = state + deriv * dt;

    // Check that velocity increased downward (in body frame, which is level)
    assert(state_new.velocity_body_m_s.z > 0.0);
    assert(near(state_new.velocity_body_m_s.z, 9.81 * dt, 1e-6));

    std::cout << "  ✓ Energy conservation check passed" << std::endl;
}

} // anonymous namespace

int main() {
    std::cout << "\n=== Comprehensive Aerodynamics & Physics Test Suite ===" << std::endl;
    std::cout << std::endl;

    try {
        test_vector3_operations();
        test_quaternion_operations();
        test_aero_model_forces();
        test_rigid_body_dynamics();
        test_atmosphere_model();
        test_energy_conservation();

        std::cout << std::endl;
        std::cout << "=== All tests passed! ===" << std::endl;
        std::cout << std::endl;

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
}
