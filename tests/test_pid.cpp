#include <cassert>
#include <cmath>

#include "aero/pid.hpp"

namespace
{
bool near(double a, double b, double eps = 1e-12)
{
    return std::fabs(a - b) <= eps;
}
} // namespace

int main()
{
    {
        aero::PidConfig cfg{};
        cfg.kp = 10.0;
        cfg.out_min = -1.0;
        cfg.out_max = 1.0;
        aero::Pid pid(cfg);

        const double u = pid.update(1.0, 0.1);
        assert(near(u, 1.0));
    }

    {
        aero::PidConfig cfg{};
        cfg.ki = 1.0;
        cfg.out_min = -0.5;
        cfg.out_max = 0.5;
        cfg.anti_windup = true;
        aero::Pid pid(cfg);

        for (int i = 0; i < 100; ++i)
        {
            const double u = pid.update(10.0, 0.1);
            assert(near(u, 0.5));
        }

        const double u0 = pid.update(0.0, 0.1);
        assert(near(u0, 0.0));
    }

    return 0;
}

