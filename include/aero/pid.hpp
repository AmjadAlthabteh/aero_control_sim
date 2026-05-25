#pragma once

#include <limits>

#include "aero/core.hpp"

namespace aero
{
struct PidConfig
{
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};

    double out_min{-std::numeric_limits<double>::infinity()};
    double out_max{std::numeric_limits<double>::infinity()};

    double i_min{-std::numeric_limits<double>::infinity()};
    double i_max{std::numeric_limits<double>::infinity()};

    double d_filter_tau_s{0.0};
    bool anti_windup{true};
};

struct PidState
{
    double i{0.0};
    double prev_err{0.0};
    double d_filt{0.0};
    bool has_prev{false};
};

class Pid
{
public:
    explicit Pid(PidConfig config) : config_(config) {}

    void reset()
    {
        state_ = {};
    }

    double update(double err, double dt_s)
    {
        if (dt_s <= 0.0)
            return 0.0;

        const double p = config_.kp * err;

        const double i_before = state_.i;
        state_.i = clamp(state_.i + config_.ki * err * dt_s, config_.i_min, config_.i_max);

        double d = 0.0;
        if (state_.has_prev)
        {
            const double d_raw = (err - state_.prev_err) / dt_s;
            if (config_.d_filter_tau_s > 0.0)
            {
                const double alpha = config_.d_filter_tau_s / (config_.d_filter_tau_s + dt_s);
                state_.d_filt = alpha * state_.d_filt + (1.0 - alpha) * d_raw;
                d = state_.d_filt;
            }
            else
            {
                d = d_raw;
            }
        }

        const double u_unsat = p + state_.i + config_.kd * d;
        const double u = clamp(u_unsat, config_.out_min, config_.out_max);

        if (config_.anti_windup && u != u_unsat)
        {
            const bool drives_further_high = (u_unsat > u) && (err > 0.0);
            const bool drives_further_low = (u_unsat < u) && (err < 0.0);
            if (drives_further_high || drives_further_low)
                state_.i = i_before;
        }

        state_.prev_err = err;
        state_.has_prev = true;
        return u;
    }

private:
    PidConfig config_{};
    PidState state_{};
};

} // namespace aero

