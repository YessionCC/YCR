#pragma once

#include "scene.hpp"
#include "camera.hpp"

class Integrator{
public:
  enum TransportMode {
    FromLight,
    FromCamera
  };
private:
  thread_local static TransportMode transportMode;

public:
  virtual ~Integrator() {}
  virtual void render(
    const Scene& scene, RayGenerator* rayGen, Film& film) const = 0;

  static void setTransportMode(TransportMode tmode);
  static TransportMode getTransportMode();
};

using TMode = Integrator::TransportMode;
