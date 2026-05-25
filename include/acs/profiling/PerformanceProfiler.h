#pragma once

#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>

namespace acs::profiling {

struct TimingStats {
  double min_ms{1e9};
  double max_ms{0.0};
  double avg_ms{0.0};
  double total_ms{0.0};
  size_t count{0};

  void update(double time_ms) {
    min_ms = std::min(min_ms, time_ms);
    max_ms = std::max(max_ms, time_ms);
    total_ms += time_ms;
    count++;
    avg_ms = total_ms / count;
  }
};

class PerformanceProfiler {
public:
  using Clock = std::chrono::high_resolution_clock;
  using TimePoint = std::chrono::time_point<Clock>;

  static PerformanceProfiler& instance() {
    static PerformanceProfiler profiler;
    return profiler;
  }

  void begin_section(const std::string& name);
  void end_section(const std::string& name);

  void record_frame();
  void increment_counter(const std::string& name, size_t value = 1);

  const TimingStats& get_stats(const std::string& name) const;
  const std::unordered_map<std::string, TimingStats>& get_all_stats() const;
  const std::unordered_map<std::string, size_t>& get_counters() const;

  double get_fps() const;
  double get_frame_time_ms() const;

  void print_report() const;
  void reset();

private:
  PerformanceProfiler() = default;

  std::unordered_map<std::string, TimePoint> active_sections_;
  std::unordered_map<std::string, TimingStats> timing_stats_;
  std::unordered_map<std::string, size_t> counters_;

  TimePoint last_frame_time_{Clock::now()};
  TimingStats frame_stats_;
};

class ScopedTimer {
public:
  explicit ScopedTimer(const std::string& name) : name_(name) {
    PerformanceProfiler::instance().begin_section(name_);
  }

  ~ScopedTimer() {
    PerformanceProfiler::instance().end_section(name_);
  }

private:
  std::string name_;
};

#define PROFILE_SCOPE(name) acs::profiling::ScopedTimer _profiler_##__LINE__(name)

}  // namespace acs::profiling
