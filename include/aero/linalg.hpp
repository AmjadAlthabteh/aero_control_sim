#pragma once

#include <array>
#include <cstddef>

namespace aero
{
template <std::size_t N>
struct Vec
{
    std::array<double, N> v{};

    double& operator[](std::size_t i) { return v[i]; }
    const double& operator[](std::size_t i) const { return v[i]; }
};

template <std::size_t N>
inline Vec<N> operator+(const Vec<N>& a, const Vec<N>& b)
{
    Vec<N> out{};
    for (std::size_t i = 0; i < N; ++i)
        out[i] = a[i] + b[i];
    return out;
}

template <std::size_t N>
inline Vec<N> operator-(const Vec<N>& a, const Vec<N>& b)
{
    Vec<N> out{};
    for (std::size_t i = 0; i < N; ++i)
        out[i] = a[i] - b[i];
    return out;
}

template <std::size_t N>
inline Vec<N> operator*(double s, const Vec<N>& x)
{
    Vec<N> out{};
    for (std::size_t i = 0; i < N; ++i)
        out[i] = s * x[i];
    return out;
}

template <std::size_t N>
inline Vec<N> operator*(const Vec<N>& x, double s)
{
    return s * x;
}

template <std::size_t R, std::size_t C>
struct Mat
{
    std::array<double, R * C> a{};

    double& operator()(std::size_t r, std::size_t c) { return a[r * C + c]; }
    const double& operator()(std::size_t r, std::size_t c) const { return a[r * C + c]; }
};

template <std::size_t R, std::size_t C>
inline Vec<R> mul(const Mat<R, C>& m, const Vec<C>& x)
{
    Vec<R> out{};
    for (std::size_t r = 0; r < R; ++r)
    {
        double sum = 0.0;
        for (std::size_t c = 0; c < C; ++c)
            sum += m(r, c) * x[c];
        out[r] = sum;
    }
    return out;
}

} // namespace aero

