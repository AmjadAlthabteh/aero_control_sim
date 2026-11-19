#include <cassert>
#include <fstream>
#include <iterator>
#include <string>

#include "acs/simulation/Telemetry.h"

int main() {
  const std::string path = "test_telemetry_tmp.csv";

  {
    acs::simulation::Telemetry telemetry(path, 0.01);
    assert(telemetry.ok());
    assert(telemetry.metadata_ok());
    telemetry.write_header();
    assert(telemetry.rows_written() == 0);
    telemetry.write_row("0,1,2");
    telemetry.write_row("1,2,3");
    assert(telemetry.rows_written() == 2);
  }

  std::ifstream in(path);
  std::string line;
  int line_count = 0;
  while (std::getline(in, line)) {
    ++line_count;
  }
  assert(line_count == 3);

  std::ifstream metadata(path + ".metadata.json");
  assert(metadata.is_open());
  const std::string metadata_text((std::istreambuf_iterator<char>(metadata)),
                                  std::istreambuf_iterator<char>());
  assert(metadata_text.find("\"sample_rate_hz\": 100") != std::string::npos);
  assert(metadata_text.find("\"channel_policy\": \"native_columns_only\"") != std::string::npos);
  assert(metadata_text.find("\"reference_channels_synthesized\": false") != std::string::npos);

  return 0;
}
