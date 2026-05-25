#pragma once

#include <functional>

namespace acs::physics {

template <typename State>
struct RK4 {
  using DerivFunc = std::function<State(const State&)>;

  static State step(const State& x, double dt, const DerivFunc& f) {
    const State k1 = f(x);
    const State k2 = f(x + (dt * 0.5) * k1);
    const State k3 = f(x + (dt * 0.5) * k2);
    const State k4 = f(x + dt * k3);
    return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
  }
};

}  // namespace acs::physics

