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

  const std::vector<double> sparse_values{10.0, std::nan(""), 30.0, 20.0};
  assert(near(acs::analysis::compute_percentile(sparse_values, 50.0), 20.0));

  const std::vector<double> series{1.0, 3.0, 5.0, 7.0};
  const auto averaged = acs::analysis::moving_average(series, 2);
  assert(averaged.size() == 3);
  assert(near(averaged[0], 2.0));
  assert(near(averaged[1], 4.0));
  assert(near(averaged[2], 6.0));

  const auto derivative = acs::analysis::differentiate(series, 0.5);
  assert(derivative.size() == 3);
  assert(near(derivative[0], 4.0));
  assert(near(derivative[1], 4.0));
  assert(near(derivative[2], 4.0));

  assert(acs::analysis::differentiate(series, 0.0).empty());

  return 0;
}
