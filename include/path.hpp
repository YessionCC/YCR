#pragma once

#include <glm/glm.hpp>
#include <vector>

#include "integrator.hpp"

class PathIntegrator: public Integrator {
private:
  int max_bounce;

public:
  PathIntegrator(int max_bounce = 12): max_bounce(max_bounce) {}
  void render(const Scene& scene, RayGenerator& rayGen, Film& film) const;
};