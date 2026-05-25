#pragma once

#include <fstream>
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

private:
  std::ofstream out_;
};

}  // namespace acs::simulation

