#pragma once

#include <chrono>
#include <string>
#include <vector>

namespace acs::monitoring {

struct SystemMetrics {
  double cpu_usage_pct{0.0};
  double memory_usage_mb{0.0};
  size_t thread_count{0};
  double uptime_s{0.0};
};

struct SimulationMetrics {
  double current_fps{0.0};
  double avg_fps{0.0};
  double min_fps{1e9};
  double max_fps{0.0};
  double frame_time_ms{0.0};
  double physics_time_ms{0.0};
  double control_time_ms{0.0};
  size_t total_frames{0};
  double sim_time_s{0.0};
  double realtime_factor{0.0};
};

class PerformanceMonitor {
public:
  PerformanceMonitor();

  void start();
  void update_frame_metrics(double frame_ms, double physics_ms, double control_ms);
  void update_simulation_time(double sim_time_s);

  void print_dashboard() const;
  void print_compact() const;

  const SimulationMetrics& get_sim_metrics() const { return sim_metrics_; }
  const SystemMetrics& get_sys_metrics() const { return sys_metrics_; }

  void save_metrics_to_csv(const std::string& path) const;

private:
  void update_system_metrics();
  std::string format_bar(double value, double max_val, size_t width = 20) const;

  SimulationMetrics sim_metrics_;
  SystemMetrics sys_metrics_;
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;

  struct FrameRecord {
    double timestamp_s;
    double frame_ms;
    double fps;
  };
  std::vector<FrameRecord> frame_history_;
  static constexpr size_t kMaxHistorySize = 1000;
};

}  // namespace acs::monitoring
