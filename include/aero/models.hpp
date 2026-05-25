#pragma once

#include "aero/state_space.hpp"

namespace aero
{
// State: x = [h_m, w_mps, theta_rad, q_radps]
// Input: u = [elevator_rad]
inline StateSpaceModel<4, 1> makeToyLongitudinalModel()
{
    StateSpaceModel<4, 1> m{};

    // h_dot = w
    m.A(0, 1) = 1.0;

    // w_dot = -0.5*w + g*theta + 5*elev
    m.A(1, 1) = -0.5;
    m.A(1, 2) = 9.81;
    m.B(1, 0) = 5.0;

    // theta_dot = q
    m.A(2, 3) = 1.0;

    // q_dot = -2.0*theta -0.8*q + 15*elev
    m.A(3, 2) = -2.0;
    m.A(3, 3) = -0.8;
    m.B(3, 0) = 15.0;

    return m;
}

} // namespace aero

