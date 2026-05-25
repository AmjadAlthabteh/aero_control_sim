#include "acs/simulation/Noise.h"

namespace acs::simulation {

Noise::Noise(unsigned int seed) : rng_(seed) {}

double Noise::gaussian(double mean, double stddev) {
  std::normal_distribution<double> dist(mean, stddev);
  return dist(rng_);
}

double Noise::uniform01() {
  std::uniform_real_distribution<double> dist(0.0, 1.0);
  return dist(rng_);
}

bool Noise::bernoulli(double p_true) {
  if (p_true <= 0.0) return false;
  if (p_true >= 1.0) return true;
  return uniform01() < p_true;
}

}  // namespace acs::simulation
