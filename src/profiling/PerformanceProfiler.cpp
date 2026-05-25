#include "acs/profiling/PerformanceProfiler.h"
#include <iostream>
#include <iomanip>
#include <algorithm>

namespace acs::profiling {

void PerformanceProfiler::begin_section(const std::string& name) {
  active_sections_[name] = Clock::now();
}

void PerformanceProfiler::end_section(const std::string& name) {
  auto it = active_sections_.find(name);
  if (it == active_sections_.end()) {
    return;
  }

  auto end_time = Clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
      end_time - it->second).count();
  double time_ms = duration / 1e6;

  timing_stats_[name].update(time_ms);
  active_sections_.erase(it);
}

void PerformanceProfiler::record_frame() {
  auto now = Clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
      now - last_frame_time_).count();
  double frame_ms = duration / 1e6;

  frame_stats_.update(frame_ms);
  last_frame_time_ = now;
}

void PerformanceProfiler::increment_counter(const std::string& name, size_t value) {
  counters_[name] += value;
}

const TimingStats& PerformanceProfiler::get_stats(const std::string& name) const {
  static TimingStats empty;
  auto it = timing_stats_.find(name);
  return it != timing_stats_.end() ? it->second : empty;
}

const std::unordered_map<std::string, TimingStats>&
PerformanceProfiler::get_all_stats() const {
  return timing_stats_;
}

const std::unordered_map<std::string, size_t>&
PerformanceProfiler::get_counters() const {
  return counters_;
}

double PerformanceProfiler::get_fps() const {
  return frame_stats_.avg_ms > 0.0 ? 1000.0 / frame_stats_.avg_ms : 0.0;
}

double PerformanceProfiler::get_frame_time_ms() const {
  return frame_stats_.avg_ms;
}

void PerformanceProfiler::print_report() const {
  std::cout << "\n=== Performance Profiling Report ===\n\n";

  std::cout << "Frame Statistics:\n";
  std::cout << "  Average FPS: " << std::fixed << std::setprecision(2)
            << get_fps() << "\n";
  std::cout << "  Frame Time: " << std::fixed << std::setprecision(3)
            << frame_stats_.avg_ms << " ms (avg), "
            << frame_stats_.min_ms << " ms (min), "
            << frame_stats_.max_ms << " ms (max)\n\n";

  if (!timing_stats_.empty()) {
    std::cout << "Section Timings:\n";
    std::cout << std::left << std::setw(30) << "  Section"
              << std::right << std::setw(12) << "Avg (ms)"
              << std::setw(12) << "Min (ms)"
              << std::setw(12) << "Max (ms)"
              << std::setw(12) << "Calls\n";
    std::cout << std::string(78, '-') << "\n";

    std::vector<std::pair<std::string, TimingStats>> sorted_stats(
        timing_stats_.begin(), timing_stats_.end());
    std::sort(sorted_stats.begin(), sorted_stats.end(),
              [](const auto& a, const auto& b) {
                return a.second.total_ms > b.second.total_ms;
              });

    for (const auto& [name, stats] : sorted_stats) {
      std::cout << "  " << std::left << std::setw(28) << name
                << std::right << std::fixed << std::setprecision(3)
                << std::setw(12) << stats.avg_ms
                << std::setw(12) << stats.min_ms
                << std::setw(12) << stats.max_ms
                << std::setw(12) << stats.count << "\n";
    }
    std::cout << "\n";
  }

  if (!counters_.empty()) {
    std::cout << "Performance Counters:\n";
    for (const auto& [name, value] : counters_) {
      std::cout << "  " << name << ": " << value << "\n";
    }
    std::cout << "\n";
  }

  std::cout << "====================================\n\n";
}

void PerformanceProfiler::reset() {
  active_sections_.clear();
  timing_stats_.clear();
  counters_.clear();
  frame_stats_ = TimingStats{};
  last_frame_time_ = Clock::now();
}

}  // namespace acs::profiling
