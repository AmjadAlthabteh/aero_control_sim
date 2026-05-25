#pragma once

#include "aero/linalg.hpp"

namespace aero
{
template <std::size_t Nx, std::size_t Nu>
struct StateSpaceModel
{
    Mat<Nx, Nx> A{};
    Mat<Nx, Nu> B{};

    Vec<Nx> xdot(const Vec<Nx>& x, const Vec<Nu>& u) const
    {
        return mul(A, x) + mul(B, u);
    }
};

} // namespace aero

