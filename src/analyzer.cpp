#include <iostream>
#include <string>

#include "acs/analysis/Statistics.h"

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " <telemetry.csv>\n";
    std::cout << "\nAnalyzes flight simulation telemetry and provides detailed statistics.\n";
    return 1;
  }

  std::string telemetry_path = argv[1];
  std::cout << "Analyzing telemetry file: " << telemetry_path << "\n";

  auto report = acs::analysis::TelemetryAnalyzer::analyze_csv(telemetry_path);
  acs::analysis::TelemetryAnalyzer::print_report(report);

  return 0;
}
