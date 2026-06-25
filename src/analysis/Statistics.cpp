#include "acs/analysis/Statistics.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string_view>
#include <unordered_map>

namespace acs::analysis {

void TimeSeriesStats::compute(const std::vector<double>& data, double dt) {
  value_stats.compute(data);

  rate_of_change_max = 0.0;
  rate_of_change_avg = 0.0;
  num_zero_crossings = 0;
  num_peaks = 0;
  num_valleys = 0;

  if (data.size() < 2) return;

  // Compute rate of change
  const bool dt_ok = std::isfinite(dt) && dt > 0.0;
  if (dt_ok) {
    std::vector<double> rates;
    rates.reserve(data.size() - 1);
    double rate_sum = 0.0;
    for (size_t i = 1; i < data.size(); ++i) {
      const double rate = std::abs((data[i] - data[i - 1]) / dt);
      rates.push_back(rate);
      rate_sum += rate;
    }
    rate_of_change_max = *std::max_element(rates.begin(), rates.end());
    rate_of_change_avg = rate_sum / rates.size();
  } else {
    rate_of_change_max = std::numeric_limits<double>::quiet_NaN();
    rate_of_change_avg = std::numeric_limits<double>::quiet_NaN();
  }

  // Count zero crossings
  for (size_t i = 1; i < data.size(); ++i) {
    if ((data[i - 1] < 0.0 && data[i] >= 0.0) ||
        (data[i - 1] > 0.0 && data[i] <= 0.0)) {
      num_zero_crossings++;
    }
  }

  // Count peaks and valleys
  for (size_t i = 1; i < data.size() - 1; ++i) {
    if (data[i] > data[i - 1] && data[i] > data[i + 1]) {
      num_peaks++;
    } else if (data[i] < data[i - 1] && data[i] < data[i + 1]) {
      num_valleys++;
    }
  }
}

static std::string trim_copy(std::string_view s) {
  size_t b = 0;
  while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
  size_t e = s.size();
  while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
  return std::string(s.substr(b, e - b));
}

static std::vector<std::string> split_csv_header(const std::string& line) {
  std::vector<std::string> cols;
  std::stringstream ss(line);
  std::string token;
  while (std::getline(ss, token, ',')) {
    cols.push_back(trim_copy(token));
  }
  return cols;
}

bool TelemetryAnalyzer::read_csv(const std::string& path,
                                std::vector<std::string>& header_out,
                                std::vector<std::vector<double>>& data_out) {
  header_out.clear();
  data_out.clear();

  std::ifstream file(path);
  if (!file.is_open()) {
    return false;
  }

  std::string line;
  if (!std::getline(file, line)) {
    return false;
  }
  header_out = split_csv_header(line);
  if (header_out.empty()) {
    return false;
  }

  while (std::getline(file, line)) {
    if (line.empty()) continue;

    std::vector<double> row;
    row.reserve(header_out.size());
    std::stringstream ss(line);
    std::string value;

    while (std::getline(ss, value, ',')) {
      try {
        row.push_back(std::stod(value));
      } catch (...) {
        row.push_back(std::numeric_limits<double>::quiet_NaN());
      }
    }

    if (row.empty()) continue;
    if (row.size() < header_out.size()) {
      row.resize(header_out.size(), std::numeric_limits<double>::quiet_NaN());
    }
    data_out.push_back(std::move(row));
  }

  return !data_out.empty();
}

TelemetryAnalyzer::AnalysisReport TelemetryAnalyzer::analyze_csv(const std::string& csv_path) {
  AnalysisReport report;

  std::vector<std::string> header;
  std::vector<std::vector<double>> data;
  if (!read_csv(csv_path, header, data)) {
    std::cerr << "Failed to read telemetry file: " << csv_path << "\n";
    return report;
  }

  std::unordered_map<std::string, size_t> idx;
  idx.reserve(header.size());
  for (size_t i = 0; i < header.size(); ++i) {
    if (!header[i].empty()) idx[header[i]] = i;
  }

  auto find_idx = [&](const char* name) -> size_t {
    auto it = idx.find(name);
    if (it == idx.end()) return static_cast<size_t>(-1);
    return it->second;
  };

  const size_t i_t = find_idx("t_s");
  const size_t i_roll = find_idx("roll_rad");
  const size_t i_pitch = find_idx("pitch_rad");
  const size_t i_yaw = find_idx("yaw_rad");
  const size_t i_alt = find_idx("alt_m");
  const size_t i_airspeed = find_idx("airspeed_mps");
  const size_t i_aileron = find_idx("aileron_rad");
  const size_t i_elevator = find_idx("elevator_rad");
  const size_t i_rudder = find_idx("rudder_rad");
  const size_t i_p = find_idx("p_rps");
  const size_t i_q = find_idx("q_rps");
  const size_t i_r = find_idx("r_rps");
  const size_t i_target_alt = find_idx("target_alt_m");

  std::vector<double> t_s, roll, pitch, yaw, altitude, airspeed;
  std::vector<double> aileron, elevator, rudder;
  std::vector<double> p, q, r;

  auto get = [](const std::vector<double>& row, size_t i) -> double {
    if (i == static_cast<size_t>(-1) || i >= row.size()) return std::numeric_limits<double>::quiet_NaN();
    return row[i];
  };

  double target_altitude_m = report.target_altitude_m;
  bool target_alt_from_telemetry = false;

  for (const auto& row : data) {
    const double alt = get(row, i_alt);
    const double ts = get(row, i_t);

    if (std::isfinite(get(row, i_roll))) roll.push_back(get(row, i_roll));
    if (std::isfinite(get(row, i_pitch))) pitch.push_back(get(row, i_pitch));
    if (std::isfinite(get(row, i_yaw))) yaw.push_back(get(row, i_yaw));
    if (std::isfinite(get(row, i_airspeed))) airspeed.push_back(get(row, i_airspeed));
    if (std::isfinite(get(row, i_aileron))) aileron.push_back(get(row, i_aileron));
    if (std::isfinite(get(row, i_elevator))) elevator.push_back(get(row, i_elevator));
    if (std::isfinite(get(row, i_rudder))) rudder.push_back(get(row, i_rudder));
    if (std::isfinite(get(row, i_p))) p.push_back(get(row, i_p));
    if (std::isfinite(get(row, i_q))) q.push_back(get(row, i_q));
    if (std::isfinite(get(row, i_r))) r.push_back(get(row, i_r));

    if (std::isfinite(ts) && std::isfinite(alt)) {
      t_s.push_back(ts);
      altitude.push_back(alt);
    }

    const double tgt = get(row, i_target_alt);
    if (!target_alt_from_telemetry && std::isfinite(tgt)) {
      target_altitude_m = tgt;
      target_alt_from_telemetry = true;
    }
  }

  report.target_altitude_m = target_altitude_m;
  report.target_altitude_from_telemetry = target_alt_from_telemetry;

  // Compute statistics
  report.roll_rad.compute(roll);
  report.pitch_rad.compute(pitch);
  report.yaw_rad.compute(yaw);
  report.altitude_m.compute(altitude);
  report.airspeed_m_s.compute(airspeed);
  report.aileron_rad.compute(aileron);
  report.elevator_rad.compute(elevator);
  report.rudder_rad.compute(rudder);
  report.roll_rate_rad_s.compute(p);
  report.pitch_rate_rad_s.compute(q);
  report.yaw_rate_rad_s.compute(r);
  report.altitude_p95_m = compute_percentile(altitude, 95.0);

  // Stability check - check if any values are diverging
  const double max_altitude = std::max(200.0, std::abs(target_altitude_m) * 5.0);  // meters
  const double max_attitude = 1.57;  // ~90 degrees
  const double max_abs_roll = std::max(std::abs(report.roll_rad.max), std::abs(report.roll_rad.min));
  const double max_abs_pitch = std::max(std::abs(report.pitch_rad.max), std::abs(report.pitch_rad.min));
  report.is_stable = (report.altitude_m.max < max_altitude) &&
                     (max_abs_roll < max_attitude) &&
                     (max_abs_pitch < max_attitude);

  // Control performance metrics for altitude (based on last 10% for steady-state)
  if (!altitude.empty()) {
    size_t steady_start = altitude.size() * 9 / 10;
    double sum = 0.0;
    for (size_t i = steady_start; i < altitude.size(); ++i) {
      sum += altitude[i];
    }
    double steady_altitude = sum / (altitude.size() - steady_start);
    report.steady_state_error = std::abs(steady_altitude - target_altitude_m);

    // Overshoot percent (relative to step size when available)
    const double y0 = altitude.front();
    const double step = target_altitude_m - y0;
    const double denom = std::max(1e-6, std::abs(step));
    double overshoot = 0.0;
    if (step >= 0.0) {
      overshoot = std::max(0.0, report.altitude_m.max - target_altitude_m);
    } else {
      overshoot = std::max(0.0, target_altitude_m - report.altitude_m.min);
    }
    report.max_overshoot_pct = 100.0 * (overshoot / denom);

    // Settling time: time after which altitude stays within a tolerance band
    if (t_s.size() == altitude.size() && t_s.size() >= 2) {
      const double tol_m = std::max(0.5, 0.02 * std::abs(step));
      size_t last_out_of_band = 0;
      bool any_out = false;
      for (size_t i = 0; i < altitude.size(); ++i) {
        if (std::abs(altitude[i] - target_altitude_m) > tol_m) {
          last_out_of_band = i;
          any_out = true;
        }
      }
      if (!any_out) {
        report.settling_time_s = 0.0;
      } else if (last_out_of_band + 1 < t_s.size()) {
        report.settling_time_s = t_s[last_out_of_band + 1];
      } else {
        report.settling_time_s = std::numeric_limits<double>::quiet_NaN();
      }
    }
  }

  return report;
}

void TelemetryAnalyzer::print_report(const AnalysisReport& report) {
  std::cout << "\n=== Telemetry Analysis Report ===\n\n";

  auto print_stats = [](const std::string& name, const StatsSummary<double>& stats, const std::string& unit) {
    std::cout << name << ":\n";
    std::cout << "  Mean: " << std::fixed << std::setprecision(4) << stats.mean << " " << unit << "\n";
    std::cout << "  Range: [" << stats.min << ", " << stats.max << "] " << unit << "\n";
    std::cout << "  Std Dev: " << stats.std_dev << " " << unit << "\n\n";
  };

  std::cout << "Attitude:\n";
  std::cout << "--------\n";
  print_stats("Roll", report.roll_rad, "rad");
  print_stats("Pitch", report.pitch_rad, "rad");
  print_stats("Yaw", report.yaw_rad, "rad");

  std::cout << "Flight State:\n";
  std::cout << "------------\n";
  print_stats("Altitude", report.altitude_m, "m");
  print_stats("Airspeed", report.airspeed_m_s, "m/s");

  std::cout << "Control Surfaces:\n";
  std::cout << "----------------\n";
  print_stats("Aileron", report.aileron_rad, "rad");
  print_stats("Elevator", report.elevator_rad, "rad");
  print_stats("Rudder", report.rudder_rad, "rad");

  std::cout << "Performance Metrics:\n";
  std::cout << "-------------------\n";
  std::cout << "Stability: " << (report.is_stable ? "STABLE" : "UNSTABLE") << "\n";
  std::cout << "Altitude Target: " << std::fixed << std::setprecision(3)
            << report.target_altitude_m << " m"
            << (report.target_altitude_from_telemetry ? " (from telemetry)" : "") << "\n";
  std::cout << "Steady State Error: " << std::fixed << std::setprecision(3)
            << report.steady_state_error << " m\n";
  std::cout << "Altitude P95: " << std::fixed << std::setprecision(3)
            << report.altitude_p95_m << " m\n";
  std::cout << "Max Overshoot: " << std::fixed << std::setprecision(2)
            << report.max_overshoot_pct << " %\n";
  if (std::isfinite(report.settling_time_s)) {
    std::cout << "Settling Time: " << std::fixed << std::setprecision(3)
              << report.settling_time_s << " s\n";
  } else {
    std::cout << "Settling Time: N/A\n";
  }

  std::cout << "\n=================================\n\n";
}

double compute_rms(const std::vector<double>& data) {
  if (data.empty()) return 0.0;
  double sum_sq = 0.0;
  for (double val : data) {
    sum_sq += val * val;
  }
  return std::sqrt(sum_sq / data.size());
}

double compute_peak_to_peak(const std::vector<double>& data) {
  if (data.empty()) return 0.0;
  auto [min_it, max_it] = std::minmax_element(data.begin(), data.end());
  return *max_it - *min_it;
}

double compute_percentile(std::vector<double> data, double percentile) {
  if (data.empty()) return 0.0;
  if (!std::isfinite(percentile)) return std::numeric_limits<double>::quiet_NaN();

  percentile = std::clamp(percentile, 0.0, 100.0);
  std::sort(data.begin(), data.end());

  const double rank = (percentile / 100.0) * static_cast<double>(data.size() - 1);
  const auto lo = static_cast<size_t>(std::floor(rank));
  const auto hi = static_cast<size_t>(std::ceil(rank));
  const double frac = rank - static_cast<double>(lo);
  return data[lo] * (1.0 - frac) + data[hi] * frac;
}

std::vector<double> moving_average(const std::vector<double>& data, size_t window_size) {
  std::vector<double> result;
  if (window_size == 0) return result;
  if (data.size() < window_size) return result;

  result.reserve(data.size() - window_size + 1);
  double sum = 0.0;

  for (size_t i = 0; i < window_size; ++i) {
    sum += data[i];
  }
  result.push_back(sum / window_size);

  for (size_t i = window_size; i < data.size(); ++i) {
    sum = sum - data[i - window_size] + data[i];
    result.push_back(sum / window_size);
  }

  return result;
}

std::vector<double> differentiate(const std::vector<double>& data, double dt) {
  std::vector<double> result;
  if (data.size() < 2) return result;
  if (!std::isfinite(dt) || dt <= 0.0) return result;

  result.reserve(data.size() - 1);
  for (size_t i = 1; i < data.size(); ++i) {
    result.push_back((data[i] - data[i - 1]) / dt);
  }

  return result;
}

}  // namespace acs::analysis
