#pragma once

#include "aero/linalg.hpp"

namespace aero
{
template <std::size_t N, typename DerivFunc>
Vec<N> rk4Step(const Vec<N>& x, double t, double dt, DerivFunc&& deriv)
{
    const Vec<N> k1 = deriv(t, x);
    const Vec<N> k2 = deriv(t + 0.5 * dt, x + (0.5 * dt) * k1);
    const Vec<N> k3 = deriv(t + 0.5 * dt, x + (0.5 * dt) * k2);
    const Vec<N> k4 = deriv(t + dt, x + dt * k3);
    return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

} // namespace aero

