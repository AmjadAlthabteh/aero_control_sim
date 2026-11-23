#pragma once

#include <fstream>
#include <cstddef>
#include <string>

namespace acs::simulation {

class Telemetry {
public:
  explicit Telemetry(const std::string& path, double sample_period_s);
  ~Telemetry();

  Telemetry(const Telemetry&) = delete;
  Telemetry& operator=(const Telemetry&) = delete;

  void write_header();
  // Returns false without writing when the row does not match the schema.
  bool write_row(const std::string& csv_row);
  bool ok() const;
  bool metadata_ok() const;
  std::size_t rows_written() const;

private:
  void write_metadata(const std::string& metadata_path, double sample_period_s);

  std::ofstream out_;
  bool metadata_ok_{false};
  std::size_t rows_written_{0};
};

}  // namespace acs::simulation
