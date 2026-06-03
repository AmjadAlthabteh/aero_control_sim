#include "acs/simulation/Noise.h"

#include <cmath>

namespace acs::simulation {

Noise::Noise(unsigned int seed) : rng_(seed) {}

double Noise::gaussian(double mean, double stddev) {
  if (stddev <= 0.0) return mean;

  // Box-Muller transform: maps two independent U(0,1) draws to a standard normal.
  //
  //   z = sqrt(−2 ln u1) · cos(2π u2)
  //
  // Why not std::normal_distribution?
  //   The STL implementation internally caches the second of the two normals it
  //   generates per transform pair.  Creating a new distribution object on every
  //   call (as the previous code did) discards that cached value, effectively
  //   throwing away half the work.  It also has undefined behaviour when stddev=0
  //   (the C++ standard requires stddev > 0).
  //
  // Why not a thread_local static distribution?
  //   Multiple Noise instances in the same thread would share the cache, meaning
  //   the sample sequence from instance A can "steal" a pre-generated value
  //   produced using instance B's RNG — breaking per-seed reproducibility.
  //
  // Generating both uniforms fresh per call is slightly more work per sample but
  // keeps the sequence from each Noise instance purely determined by its own seed.
  // u1 is clamped away from zero to avoid log(0) → −∞ on rare full-zero draws.
  static constexpr double kTwoPi = 6.283185307179586;
  std::uniform_real_distribution<double> u_dist{0.0, 1.0};
  const double u1 = std::max(u_dist(rng_), 1e-300);
  const double u2 = u_dist(rng_);
  const double z  = std::sqrt(-2.0 * std::log(u1)) * std::cos(kTwoPi * u2);
  return mean + stddev * z;
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
