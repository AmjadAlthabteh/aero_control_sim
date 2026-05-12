#pragma once

#include <algorithm>
#include <cstddef>

namespace aero
{
template <typename T>
constexpr T clamp(T value, T low, T high)
{
    return std::min(std::max(value, low), high);
}

struct SimConfig
{
    double dt_s{0.01};
    double duration_s{20.0};
};

} // namespace aero

