#include "acs/simulation/Telemetry.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>

namespace acs::simulation {

namespace {
struct ColumnDefinition {
  const char* name;
  const char* unit;
};

constexpr std::array<ColumnDefinition, 62> kColumns{{
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
    {"target_roll_rad", "rad"}, {"target_pitch_rad", "rad"}, {"mission_segment", "index"},
    {"fused_pn_m", "m"}, {"fused_pe_m", "m"}, {"fused_pd_m", "m"},
    {"fused_vn_mps", "m/s"}, {"fused_ve_mps", "m/s"}, {"fused_vd_mps", "m/s"},
    {"fused_roll_rad", "rad"}, {"fused_pitch_rad", "rad"}, {"fused_yaw_rad", "rad"},
    {"fused_alt_m", "m"}, {"fused_airspeed_mps", "m/s"},
    {"fused_pos_std_m", "m"}, {"fused_vel_std_mps", "m/s"},
    {"gps_innovation_m", "m"}, {"air_data_innovation_mps", "m/s"},
    {"gps_aiding", "bool"}, {"air_data_aiding", "bool"}}};
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
                << "  \"schema_version\": 3,\n"
                << "  \"coordinate_frames\": {\"navigation\": \"NED\", \"body\": \"FRD\"},\n"
                << "  \"sample_period_s\": " << (valid_period ? sample_period_s : 0.0) << ",\n"
                << "  \"sample_rate_hz\": " << (valid_period ? 1.0 / sample_period_s : 0.0) << ",\n"
                << "  \"channel_policy\": \"native_columns_only\",\n"
                << "  \"reference_channels_synthesized\": false,\n"
                << "  \"channel_groups\": {\n"
                << "    \"state\": [\"pn_m\", \"pe_m\", \"pd_m\", \"u_mps\", \"v_mps\", \"w_mps\", \"p_rps\", \"q_rps\", \"r_rps\", \"roll_rad\", \"pitch_rad\", \"yaw_rad\", \"alt_m\"],\n"
                << "    \"air_data\": [\"airspeed_mps\", \"alpha_rad\", \"beta_rad\", \"u_air_mps\", \"v_air_mps\", \"w_air_mps\"],\n"
                << "    \"applied_controls\": [\"aileron_rad\", \"elevator_rad\", \"rudder_rad\", \"throttle_01\"],\n"
                << "    \"forces_moments\": [\"fx_n\", \"fy_n\", \"fz_n\", \"mx_nm\", \"my_nm\", \"mz_nm\"],\n"
                << "    \"environment\": [\"wind_n_mps\", \"wind_e_mps\", \"wind_d_mps\"],\n"
                << "    \"commanded_controls\": [\"aileron_cmd_rad\", \"elevator_cmd_rad\", \"rudder_cmd_rad\", \"throttle_cmd_01\"],\n"
                << "    \"sensor_validity\": [\"baro_valid\", \"gps_valid\"],\n"
                << "    \"fused_state\": [\"fused_pn_m\", \"fused_pe_m\", \"fused_pd_m\", \"fused_vn_mps\", \"fused_ve_mps\", \"fused_vd_mps\", \"fused_roll_rad\", \"fused_pitch_rad\", \"fused_yaw_rad\", \"fused_alt_m\", \"fused_airspeed_mps\"],\n"
                << "    \"fusion_health\": [\"fused_pos_std_m\", \"fused_vel_std_mps\", \"gps_innovation_m\", \"air_data_innovation_mps\", \"gps_aiding\", \"air_data_aiding\"],\n"
                << "    \"reference_targets\": [\"target_alt_m\", \"target_heading_rad\", \"target_airspeed_mps\", \"target_roll_rad\", \"target_pitch_rad\"],\n"
                << "    \"context\": [\"t_s\", \"mission_segment\"]\n"
                << "  },\n"
                << "  \"columns\": [\n";
  for (std::size_t i = 0; i < kColumns.size(); ++i) {
    metadata << "    {\"name\": \"" << kColumns[i].name << "\", \"unit\": \""
             << kColumns[i].unit << "\"}" << (i + 1 == kColumns.size() ? "\n" : ",\n");
  }
  metadata << "  ]\n}\n";
  metadata.flush();
  metadata_ok_ = metadata.good();
}

bool Telemetry::write_row(const std::string& csv_row) {
  if (!ok()) return false;
  const std::size_t field_count =
      1U + static_cast<std::size_t>(std::count(csv_row.begin(), csv_row.end(), ','));
  if (field_count != kColumns.size()) return false;
  out_ << csv_row << "\n";
  if (ok()) {
    ++rows_written_;
    return true;
  }
  return false;
}

}  // namespace acs::simulation
