#include "acs/monitoring/PerformanceMonitor.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace acs::monitoring {

PerformanceMonitor::PerformanceMonitor() {
  start_time_ = std::chrono::high_resolution_clock::now();
}

void PerformanceMonitor::start() {
  start_time_ = std::chrono::high_resolution_clock::now();
  sim_metrics_ = SimulationMetrics{};
  sys_metrics_ = SystemMetrics{};
  frame_history_.clear();
}

void PerformanceMonitor::update_frame_metrics(double frame_ms, double physics_ms,
                                             double control_ms) {
  sim_metrics_.total_frames++;
  sim_metrics_.frame_time_ms = frame_ms;
  sim_metrics_.physics_time_ms = physics_ms;
  sim_metrics_.control_time_ms = control_ms;

  if (frame_ms > 0.0) {
    sim_metrics_.current_fps = 1000.0 / frame_ms;
    sim_metrics_.min_fps = std::min(sim_metrics_.min_fps, sim_metrics_.current_fps);
    sim_metrics_.max_fps = std::max(sim_metrics_.max_fps, sim_metrics_.current_fps);

    // Update average FPS
    double total_time = sim_metrics_.total_frames * frame_ms;
    sim_metrics_.avg_fps = (sim_metrics_.total_frames * 1000.0) / total_time;
  }

  // Record frame history
  auto now = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - start_time_).count();
  double timestamp_s = duration / 1000.0;

  frame_history_.push_back({timestamp_s, frame_ms, sim_metrics_.current_fps});

  if (frame_history_.size() > kMaxHistorySize) {
    frame_history_.erase(frame_history_.begin());
  }

  update_system_metrics();
}

void PerformanceMonitor::update_simulation_time(double sim_time_s) {
  sim_metrics_.sim_time_s = sim_time_s;

  auto now = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - start_time_).count();
  double wall_time_s = duration / 1000.0;

  if (wall_time_s > 0.0) {
    sim_metrics_.realtime_factor = sim_time_s / wall_time_s;
  }
}

void PerformanceMonitor::update_system_metrics() {
  auto now = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::seconds>(
      now - start_time_).count();
  sys_metrics_.uptime_s = static_cast<double>(duration);

  // Note: CPU/memory metrics would require platform-specific code
  // This is a placeholder implementation
  sys_metrics_.thread_count = 1;
}

std::string PerformanceMonitor::format_bar(double value, double max_val, size_t width) const {
  if (max_val <= 0.0) return std::string(width, ' ');

  double ratio = std::clamp(value / max_val, 0.0, 1.0);
  size_t filled = static_cast<size_t>(ratio * width);

  std::string bar = "[";
  for (size_t i = 0; i < width; ++i) {
    bar += (i < filled) ? "=" : " ";
  }
  bar += "]";
  return bar;
}

void PerformanceMonitor::print_dashboard() const {
  std::cout << "\n";
  std::cout << "╔════════════════════════════════════════════════════════════╗\n";
  std::cout << "║         PERFORMANCE MONITORING DASHBOARD                  ║\n";
  std::cout << "╠════════════════════════════════════════════════════════════╣\n";

  // Simulation Metrics
  std::cout << "║ SIMULATION METRICS                                         ║\n";
  std::cout << "╟────────────────────────────────────────────────────────────╢\n";
  std::cout << "║ FPS:          " << std::setw(8) << std::fixed << std::setprecision(1)
            << sim_metrics_.current_fps << " fps (avg: " << sim_metrics_.avg_fps << ")     ║\n";
  std::cout << "║ FPS Range:    " << std::setw(8) << std::fixed << std::setprecision(1)
            << sim_metrics_.min_fps << " - " << sim_metrics_.max_fps << " fps         ║\n";
  std::cout << "║ Frame Time:   " << std::setw(8) << std::fixed << std::setprecision(3)
            << sim_metrics_.frame_time_ms << " ms                         ║\n";
  std::cout << "║   Physics:    " << std::setw(8) << std::fixed << std::setprecision(3)
            << sim_metrics_.physics_time_ms << " ms                         ║\n";
  std::cout << "║   Control:    " << std::setw(8) << std::fixed << std::setprecision(3)
            << sim_metrics_.control_time_ms << " ms                         ║\n";
  std::cout << "║ Total Frames: " << std::setw(10) << sim_metrics_.total_frames
            << "                                     ║\n";
  std::cout << "║ Sim Time:     " << std::setw(8) << std::fixed << std::setprecision(2)
            << sim_metrics_.sim_time_s << " s                          ║\n";
  std::cout << "║ RT Factor:    " << std::setw(8) << std::fixed << std::setprecision(2)
            << sim_metrics_.realtime_factor << "x                           ║\n";

  // System Metrics
  std::cout << "╟────────────────────────────────────────────────────────────╢\n";
  std::cout << "║ SYSTEM METRICS                                             ║\n";
  std::cout << "╟────────────────────────────────────────────────────────────╢\n";
  std::cout << "║ Uptime:       " << std::setw(8) << std::fixed << std::setprecision(1)
            << sys_metrics_.uptime_s << " s                          ║\n";
  std::cout << "║ Threads:      " << std::setw(8) << sys_metrics_.thread_count
            << "                                      ║\n";

  // Performance bars
  std::cout << "╟────────────────────────────────────────────────────────────╢\n";
  std::cout << "║ PERFORMANCE VISUALIZATION                                  ║\n";
  std::cout << "╟────────────────────────────────────────────────────────────╢\n";

  double target_fps = 100.0;  // Assuming 0.01s timestep = 100 fps ideal
  std::cout << "║ FPS:     " << format_bar(sim_metrics_.current_fps, target_fps, 30)
            << "     ║\n";

  double max_frame_ms = 20.0;
  std::cout << "║ Frame:   " << format_bar(sim_metrics_.frame_time_ms, max_frame_ms, 30)
            << "     ║\n";

  std::cout << "╚════════════════════════════════════════════════════════════╝\n";
  std::cout << std::endl;
}

void PerformanceMonitor::print_compact() const {
  std::cout << "[PERF] FPS: " << std::fixed << std::setprecision(1)
            << sim_metrics_.current_fps << " | Frame: " << std::setprecision(3)
            << sim_metrics_.frame_time_ms << "ms | RT: " << std::setprecision(2)
            << sim_metrics_.realtime_factor << "x\n";
}

void PerformanceMonitor::save_metrics_to_csv(const std::string& path) const {
  std::ofstream file(path);
  if (!file.is_open()) {
    std::cerr << "Failed to open metrics file: " << path << "\n";
    return;
  }

  file << "timestamp_s,frame_time_ms,fps\n";
  for (const auto& record : frame_history_) {
    file << std::fixed << std::setprecision(6)
         << record.timestamp_s << ","
         << record.frame_ms << ","
         << record.fps << "\n";
  }

  std::cout << "Saved performance metrics to: " << path << "\n";
}

}  // namespace acs::monitoring
