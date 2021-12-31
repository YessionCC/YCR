#pragma once

#include <glm/glm.hpp>
#include <vector>

#include "integrator.hpp"

class PathIntegrator: public Integrator {
private:
  int max_bounce;
  bool useMIS;

  void render_no_medium(const Scene& scene, RayGenerator& rayGen, Film& film) const;
  void render_with_medium(const Scene& scene, RayGenerator& rayGen, Film& film) const;

public:
  PathIntegrator(int max_bounce = 12, bool useMIS = true): 
    max_bounce(max_bounce), useMIS(useMIS) {}
  void render(const Scene& scene, RayGenerator& rayGen, Film& film) const;
};