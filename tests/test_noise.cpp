#include <cassert>

#include "acs/simulation/Noise.h"

int main() {
  acs::simulation::Noise a(123U);
  acs::simulation::Noise b(123U);

  for (int i = 0; i < 32; ++i) {
    assert(a.uniform01() == b.uniform01());
  }

  acs::simulation::Noise c(456U);
  assert(c.gaussian(5.0, 0.0) == 5.0);
  assert(!c.bernoulli(0.0));
  assert(c.bernoulli(1.0));

  return 0;
}
