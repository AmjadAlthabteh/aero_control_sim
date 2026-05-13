#include <cmath>
#include <iostream>

#include "aero/cli.hpp"
#include "aero/core.hpp"
#include "aero/csv.hpp"
#include "aero/models.hpp"
#include "aero/pid.hpp"
#include "aero/rk4.hpp"

namespace
{
constexpr double deg2rad(double deg) { return deg * (3.14159265358979323846 / 180.0); }
} // namespace

int main(int argc, char** argv)
{
    bool ok = true;
    const auto args = aero::parseArgs(argc, argv, ok);
    if (!ok || args.help)
    {
        std::cout << "aero_control_sim\n\n"
                     "Flags:\n"
                     "  --dt <seconds>         Timestep (default 0.01)\n"
                     "  --duration <seconds>   Sim duration (default 20)\n"
                     "  --step-alt <meters>    Altitude step command (default 100)\n"
                     "  --step-time <seconds>  Step time (default 1)\n"
                     "  --out <path>           Output CSV (default out.csv)\n";
        return ok ? 0 : 2;
    }

    aero::SimConfig cfg{};
    cfg.dt_s = args.dt_s;
    cfg.duration_s = args.duration_s;

    const auto plant = aero::makeToyLongitudinalModel();

    aero::Vec<4> x{};

    aero::PidConfig altCfg{};
    altCfg.kp = 0.002;
    altCfg.ki = 0.00015;
    altCfg.out_min = -deg2rad(20.0);
    altCfg.out_max = deg2rad(20.0);
    altCfg.i_min = -deg2rad(5.0);
    altCfg.i_max = deg2rad(5.0);
    altCfg.anti_windup = true;
    aero::Pid altPid(altCfg);

    aero::PidConfig pitchCfg{};
    pitchCfg.kp = 3.0;
    pitchCfg.ki = 0.8;
    pitchCfg.kd = 0.12;
    pitchCfg.out_min = -deg2rad(25.0);
    pitchCfg.out_max = deg2rad(25.0);
    pitchCfg.i_min = -deg2rad(10.0);
    pitchCfg.i_max = deg2rad(10.0);
    pitchCfg.d_filter_tau_s = 0.05;
    pitchCfg.anti_windup = true;
    aero::Pid pitchPid(pitchCfg);

    aero::CsvWriter csv(args.out_csv);
    if (!csv.ok())
    {
        std::cerr << "Failed to open " << args.out_csv << " for writing\n";
        return 1;
    }

    csv.writeHeader({"t_s", "h_m", "w_mps", "theta_rad", "q_radps", "h_cmd_m", "theta_cmd_rad", "elev_cmd_rad"});

    const double step_altitude_m = args.step_altitude_m;
    const double step_time_s = args.step_time_s;

    for (double t = 0.0; t <= cfg.duration_s; t += cfg.dt_s)
    {
        const double h_cmd = (t >= step_time_s) ? step_altitude_m : 0.0;
        const double h = x[0];
        const double theta = x[2];

        const double theta_cmd = altPid.update(h_cmd - h, cfg.dt_s);
        const double elev_cmd = pitchPid.update(theta_cmd - theta, cfg.dt_s);

        aero::Vec<1> u{};
        u[0] = elev_cmd;

        x = aero::rk4Step<4>(
            x,
            t,
            cfg.dt_s,
            [&](double /*time*/, const aero::Vec<4>& xs) { return plant.xdot(xs, u); });

        csv.writeRow({t, x[0], x[1], x[2], x[3], h_cmd, theta_cmd, elev_cmd});
    }

    std::cout << "Wrote " << args.out_csv << "\n";
    return 0;
}
