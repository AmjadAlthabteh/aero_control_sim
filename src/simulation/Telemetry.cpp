#include "acs/simulation/Telemetry.h"

#include <array>
#include <cmath>
#include <iomanip>

namespace acs::simulation {

namespace {
struct ColumnDefinition {
  const char* name;
  const char* unit;
};

constexpr std::array<ColumnDefinition, 45> kColumns{{
    {"t_s", "s"}, {"pn_m", "m"}, {"pe_m", "m"}, {"pd_m", "m"},
    {"u_mps", "m/s"}, {"v_mps", "m/s"}, {"w_mps", "m/s"},
    {"p_rps", "rad/s"}, {"q_rps", "rad/s"}, {"r_rps", "rad/s"},
    {"roll_rad", "rad"}, {"pitch_rad", "rad"}, {"yaw_rad", "rad"},
    {"alt_m", "m"}, {"airspeed_mps", "m/s"}, {"alpha_rad", "rad"}, {"beta_rad", "rad"},
    {"aileron_rad", "rad"}, {"elevator_rad", "rad"}, {"rudder_rad", "rad"}, {"throttle_01", "1"},
    {"fx_n", "N"}, {"fy_n", "N"}, {"fz_n", "N"},
    {"mx_nm", "N*m"}, {"my_nm", "N*m"}, {"mz_nm", "N*m"},
    {"wind_n_mps", "m/s"}, {"wind_e_mps", "m/s"}, {"wind_d_mps", "m/s"},
    {"u_air_mps", "m/s"}, {"v_air_mps", "m/s"}, {"w_air_mps", "m/s"},
    {"aileron_cmd_rad", "rad"}, {"elevator_cmd_rad", "rad"}, {"rudder_cmd_rad", "rad"},
    {"throttle_cmd_01", "1"}, {"baro_valid", "bool"}, {"gps_valid", "bool"},
    {"target_alt_m", "m"}, {"target_heading_rad", "rad"}, {"target_airspeed_mps", "m/s"},
    {"target_roll_rad", "rad"}, {"target_pitch_rad", "rad"}, {"mission_segment", "index"}}};
}  // namespace

Telemetry::Telemetry(const std::string& path, double sample_period_s)
    : out_(path, std::ios::out | std::ios::trunc) {
  write_metadata(path + ".metadata.json", sample_period_s);
}

Telemetry::~Telemetry() {
  if (out_.is_open()) {
    out_.flush();
    out_.close();
  }
}

bool Telemetry::ok() const { return out_.good(); }

bool Telemetry::metadata_ok() const { return metadata_ok_; }

std::size_t Telemetry::rows_written() const { return rows_written_; }

void Telemetry::write_header() {
  if (!ok()) return;
  for (std::size_t i = 0; i < kColumns.size(); ++i) {
    if (i != 0) out_ << ',';
    out_ << kColumns[i].name;
  }
  out_ << '\n';
}

void Telemetry::write_metadata(const std::string& metadata_path, double sample_period_s) {
  std::ofstream metadata(metadata_path, std::ios::out | std::ios::trunc);
  if (!metadata.good()) return;
  const bool valid_period = std::isfinite(sample_period_s) && sample_period_s > 0.0;
  metadata << std::setprecision(12)
                << "{\n"
                << "  \"schema\": \"aero-control-system-sim.telemetry\",\n"
                << "  \"schema_version\": 1,\n"
                << "  \"coordinate_frames\": {\"navigation\": \"NED\", \"body\": \"FRD\"},\n"
                << "  \"sample_period_s\": " << (valid_period ? sample_period_s : 0.0) << ",\n"
                << "  \"sample_rate_hz\": " << (valid_period ? 1.0 / sample_period_s : 0.0) << ",\n"
                << "  \"channel_policy\": \"native_columns_only\",\n"
                << "  \"reference_channels_synthesized\": false,\n"
                << "  \"columns\": [\n";
  for (std::size_t i = 0; i < kColumns.size(); ++i) {
    metadata << "    {\"name\": \"" << kColumns[i].name << "\", \"unit\": \""
             << kColumns[i].unit << "\"}" << (i + 1 == kColumns.size() ? "\n" : ",\n");
  }
  metadata << "  ]\n}\n";
  metadata.flush();
  metadata_ok_ = metadata.good();
}

void Telemetry::write_row(const std::string& csv_row) {
  if (!ok()) return;
  out_ << csv_row << "\n";
  if (ok()) {
    ++rows_written_;
  }
}

}  // namespace acs::simulation
