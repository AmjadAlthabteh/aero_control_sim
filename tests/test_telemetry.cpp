#include <cassert>
#include <fstream>
#include <string>

#include "acs/simulation/Telemetry.h"

int main() {
  const std::string path = "test_telemetry_tmp.csv";

  {
    acs::simulation::Telemetry telemetry(path);
    assert(telemetry.ok());
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

  return 0;
}
