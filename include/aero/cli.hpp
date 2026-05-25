#pragma once

#include <cstddef>
#include <string>

namespace aero
{
struct CliArgs
{
    double dt_s{0.01};
    double duration_s{20.0};
    double step_altitude_m{100.0};
    double step_time_s{1.0};
    std::string out_csv{"out.csv"};
    bool help{false};
};

inline bool startsWith(const std::string& s, const std::string& prefix)
{
    return s.size() >= prefix.size() && s.compare(0, prefix.size(), prefix) == 0;
}

inline bool parseDouble(const std::string& s, double& out)
{
    std::size_t pos = 0;
    try
    {
        out = std::stod(s, &pos);
        return pos == s.size();
    }
    catch (...)
    {
        return false;
    }
}

inline CliArgs parseArgs(int argc, char** argv, bool& ok)
{
    ok = true;
    CliArgs a{};

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help")
        {
            a.help = true;
            continue;
        }

        auto requireValue = [&](double& target) {
            if (i + 1 >= argc)
            {
                ok = false;
                return;
            }
            double v = 0.0;
            if (!parseDouble(argv[++i], v))
            {
                ok = false;
                return;
            }
            target = v;
        };

        if (arg == "--dt")
        {
            requireValue(a.dt_s);
        }
        else if (arg == "--duration")
        {
            requireValue(a.duration_s);
        }
        else if (arg == "--step-alt")
        {
            requireValue(a.step_altitude_m);
        }
        else if (arg == "--step-time")
        {
            requireValue(a.step_time_s);
        }
        else if (arg == "--out")
        {
            if (i + 1 >= argc)
            {
                ok = false;
                continue;
            }
            a.out_csv = argv[++i];
        }
        else
        {
            ok = false;
        }
    }

    return a;
}

} // namespace aero
