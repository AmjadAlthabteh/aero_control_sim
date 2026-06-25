#include <cassert>
#include <cmath>
#include <vector>

#include "acs/analysis/Statistics.h"

namespace {
bool near(double a, double b, double eps = 1e-12) {
  return std::fabs(a - b) <= eps;
}
}  // namespace

int main() {
  const std::vector<double> values{10.0, 0.0, 30.0, 20.0};

  assert(near(acs::analysis::compute_percentile(values, 0.0), 0.0));
  assert(near(acs::analysis::compute_percentile(values, 50.0), 15.0));
  assert(near(acs::analysis::compute_percentile(values, 75.0), 22.5));
  assert(near(acs::analysis::compute_percentile(values, 100.0), 30.0));
  assert(near(acs::analysis::compute_percentile(values, -5.0), 0.0));
  assert(near(acs::analysis::compute_percentile(values, 105.0), 30.0));

  return 0;
}
