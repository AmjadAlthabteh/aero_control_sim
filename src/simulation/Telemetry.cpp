#include "acs/simulation/Telemetry.h"

namespace acs::simulation {

Telemetry::Telemetry(const std::string& path) : out_(path, std::ios::out | std::ios::trunc) {}

Telemetry::~Telemetry() {
  if (out_.is_open()) {
    out_.flush();
    out_.close();
  }
}

bool Telemetry::ok() const { return out_.good(); }

std::size_t Telemetry::rows_written() const { return rows_written_; }

void Telemetry::write_header() {
  if (!ok()) return;
  out_ << "t_s,"
          "pn_m,pe_m,pd_m,"
          "u_mps,v_mps,w_mps,"
          "p_rps,q_rps,r_rps,"
          "roll_rad,pitch_rad,yaw_rad,"
          "alt_m,airspeed_mps,alpha_rad,beta_rad,"
          "aileron_rad,elevator_rad,rudder_rad,throttle_01,"
          "fx_n,fy_n,fz_n,"
          "mx_nm,my_nm,mz_nm,"
          "wind_n_mps,wind_e_mps,wind_d_mps,"
          "u_air_mps,v_air_mps,w_air_mps,"
          "aileron_cmd_rad,elevator_cmd_rad,rudder_cmd_rad,throttle_cmd_01,"
          "baro_valid,gps_valid,"
          "target_alt_m,target_heading_rad,target_airspeed_mps,target_roll_rad,target_pitch_rad\n";
}

void Telemetry::write_row(const std::string& csv_row) {
  if (!ok()) return;
  out_ << csv_row << "\n";
  if (ok()) {
    ++rows_written_;
  }
}

}  // namespace acs::simulation
