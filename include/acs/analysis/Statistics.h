#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace acs::analysis {

template <typename T>
struct StatsSummary {
  T min{};
  T max{};
  T mean{};
  T median{};
  T std_dev{};
  T variance{};
  size_t count{0};

  void compute(const std::vector<T>& data) {
    if (data.empty()) return;

    count = data.size();
    min = *std::min_element(data.begin(), data.end());
    max = *std::max_element(data.begin(), data.end());

    // Compute mean
    T sum = T{};
    for (const auto& val : data) {
      sum += val;
    }
    mean = sum / static_cast<T>(count);

    // Compute variance and std dev
    T sq_sum = T{};
    for (const auto& val : data) {
      T diff = val - mean;
      sq_sum += diff * diff;
    }
    variance = sq_sum / static_cast<T>(count);
    std_dev = std::sqrt(variance);

    // Compute median
    std::vector<T> sorted_data = data;
    std::sort(sorted_data.begin(), sorted_data.end());
    if (count % 2 == 0) {
      median = (sorted_data[count / 2 - 1] + sorted_data[count / 2]) / T{2};
    } else {
      median = sorted_data[count / 2];
    }
  }
};

struct TimeSeriesStats {
  StatsSummary<double> value_stats;
  double rate_of_change_max{0.0};
  double rate_of_change_avg{0.0};
  size_t num_zero_crossings{0};
  size_t num_peaks{0};
  size_t num_valleys{0};

  void compute(const std::vector<double>& data, double dt);
};

class TelemetryAnalyzer {
public:
  struct AnalysisReport {
    // Attitude stats
    StatsSummary<double> roll_rad;
    StatsSummary<double> pitch_rad;
    StatsSummary<double> yaw_rad;

    // Position/velocity stats
    StatsSummary<double> altitude_m;
    StatsSummary<double> airspeed_m_s;

    // Control surface stats
    StatsSummary<double> aileron_rad;
    StatsSummary<double> elevator_rad;
    StatsSummary<double> rudder_rad;

    // Angular rate stats
    StatsSummary<double> roll_rate_rad_s;
    StatsSummary<double> pitch_rate_rad_s;
    StatsSummary<double> yaw_rate_rad_s;

    // Performance metrics
    double settling_time_s{0.0};
    double max_overshoot_pct{0.0};
    double steady_state_error{0.0};
    bool is_stable{true};
  };

  static AnalysisReport analyze_csv(const std::string& csv_path);
  static void print_report(const AnalysisReport& report);

private:
  static std::vector<std::vector<double>> read_csv(const std::string& path);
};

// Utility functions for signal processing
double compute_rms(const std::vector<double>& data);
double compute_peak_to_peak(const std::vector<double>& data);
std::vector<double> moving_average(const std::vector<double>& data, size_t window_size);
std::vector<double> differentiate(const std::vector<double>& data, double dt);

}  // namespace acs::analysis
