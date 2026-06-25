#pragma once

#include <random>

namespace acs::simulation {

class Noise {
public:
  explicit Noise(unsigned int seed = 1U);

  double gaussian(double mean, double stddev);
  double uniform01();
  bool bernoulli(double p_true);

private:
  std::mt19937 rng_;
  std::uniform_real_distribution<double> uniform01_{0.0, 1.0};
};

}  // namespace acs::simulation
