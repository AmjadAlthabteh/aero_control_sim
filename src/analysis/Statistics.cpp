#include "acs/analysis/Statistics.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace acs::analysis {

void TimeSeriesStats::compute(const std::vector<double>& data, double dt) {
  value_stats.compute(data);

  if (data.size() < 2) return;

  // Compute rate of change
  std::vector<double> rates;
  rates.reserve(data.size() - 1);
  double rate_sum = 0.0;
  for (size_t i = 1; i < data.size(); ++i) {
    double rate = std::abs((data[i] - data[i - 1]) / dt);
    rates.push_back(rate);
    rate_sum += rate;
  }
  rate_of_change_max = *std::max_element(rates.begin(), rates.end());
  rate_of_change_avg = rate_sum / rates.size();

  // Count zero crossings
  num_zero_crossings = 0;
  for (size_t i = 1; i < data.size(); ++i) {
    if ((data[i - 1] < 0.0 && data[i] >= 0.0) ||
        (data[i - 1] > 0.0 && data[i] <= 0.0)) {
      num_zero_crossings++;
    }
  }

  // Count peaks and valleys
  num_peaks = 0;
  num_valleys = 0;
  for (size_t i = 1; i < data.size() - 1; ++i) {
    if (data[i] > data[i - 1] && data[i] > data[i + 1]) {
      num_peaks++;
    } else if (data[i] < data[i - 1] && data[i] < data[i + 1]) {
      num_valleys++;
    }
  }
}

std::vector<std::vector<double>> TelemetryAnalyzer::read_csv(const std::string& path) {
  std::vector<std::vector<double>> data;
  std::ifstream file(path);
  if (!file.is_open()) {
    return data;
  }

  std::string line;
  // Skip header
  std::getline(file, line);

  while (std::getline(file, line)) {
    std::vector<double> row;
    std::stringstream ss(line);
    std::string value;

    while (std::getline(ss, value, ',')) {
      try {
        row.push_back(std::stod(value));
      } catch (...) {
        row.push_back(0.0);
      }
    }
    if (!row.empty()) {
      data.push_back(row);
    }
  }

  return data;
}

TelemetryAnalyzer::AnalysisReport TelemetryAnalyzer::analyze_csv(const std::string& csv_path) {
  AnalysisReport report;

  auto data = read_csv(csv_path);
  if (data.empty()) {
    std::cerr << "Failed to read telemetry file: " << csv_path << "\n";
    return report;
  }

  // Extract columns (based on telemetry format)
  std::vector<double> roll, pitch, yaw, altitude, airspeed;
  std::vector<double> aileron, elevator, rudder;
  std::vector<double> p, q, r;

  for (const auto& row : data) {
    if (row.size() < 20) continue;

    roll.push_back(row[10]);
    pitch.push_back(row[11]);
    yaw.push_back(row[12]);
    altitude.push_back(row[13]);
    airspeed.push_back(row[14]);
    aileron.push_back(row[17]);
    elevator.push_back(row[18]);
    rudder.push_back(row[19]);
    p.push_back(row[7]);
    q.push_back(row[8]);
    r.push_back(row[9]);
  }

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

  // Stability check - check if any values are diverging
  const double max_altitude = 200.0;  // meters
  const double max_attitude = 1.57;    // ~90 degrees
  report.is_stable = (report.altitude_m.max < max_altitude) &&
                    (std::abs(report.roll_rad.max) < max_attitude) &&
                    (std::abs(report.pitch_rad.max) < max_attitude);

  // Compute steady state error for altitude (last 10% of data)
  if (!altitude.empty()) {
    size_t steady_start = altitude.size() * 9 / 10;
    double sum = 0.0;
    for (size_t i = steady_start; i < altitude.size(); ++i) {
      sum += altitude[i];
    }
    double steady_altitude = sum / (altitude.size() - steady_start);
    report.steady_state_error = std::abs(steady_altitude - 60.0);  // Target is 60m
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
  std::cout << "Steady State Error: " << std::fixed << std::setprecision(3)
            << report.steady_state_error << " m\n";

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

std::vector<double> moving_average(const std::vector<double>& data, size_t window_size) {
  std::vector<double> result;
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

  result.reserve(data.size() - 1);
  for (size_t i = 1; i < data.size(); ++i) {
    result.push_back((data[i] - data[i - 1]) / dt);
  }

  return result;
}

}  // namespace acs::analysis
