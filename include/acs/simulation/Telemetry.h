#pragma once

#include <fstream>
#include <cstddef>
#include <string>

namespace acs::simulation {

class Telemetry {
public:
  explicit Telemetry(const std::string& path);
  ~Telemetry();

  Telemetry(const Telemetry&) = delete;
  Telemetry& operator=(const Telemetry&) = delete;

  void write_header();
  void write_row(const std::string& csv_row);
  bool ok() const;
  std::size_t rows_written() const;

private:
  std::ofstream out_;
  std::size_t rows_written_{0};
};

}  // namespace acs::simulation
