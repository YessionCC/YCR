#pragma once

#include <glm/glm.hpp>
#include <vector>

#include "film.hpp"
#include "scene.hpp"
#include "bxdf.hpp"
#include "camera.hpp"

struct PathVertex {
  Intersection itsc;
  glm::vec3 beta;
  glm::vec3 dir_o;
  int bxdfType;
};

class SubPathGenerator {
public:
  enum TerminateState {
    UpToMaxBounce,
    ItscLight,
    NoItsc,
    RRFailed,
    None
  };

private:
  std::vector<PathVertex> pathVertices;
  TerminateState tstate = TerminateState::None;
  glm::vec3 diracRadiance; // first light bounce or specular light

public: 
  // return real bounce
  // TODO: Russia Roulette for BDPT ?
  int createSubPath(const Ray& start_ray, Scene& scene, int max_bounce);

  void clear() {
    diracRadiance = glm::vec3(0.0f);
    pathVertices.clear(); 
    tstate = TerminateState::None;
  }

  std::vector<PathVertex>& getPathVtxs() {return pathVertices;}

  TerminateState getTerminateState() const {return tstate;}

  glm::vec3 getDiracLight() const {return diracRadiance;}
};

class PathIntegrator {
private:
  using PathTerminateState = SubPathGenerator::TerminateState;
  const int max_bounce = 12;

  SubPathGenerator subPathGenerator;

public:
  void render(RayGenerator& rayGen, Scene& scene, Film& film);
  void visualizeRender(PCShower& pc, RayGenerator& rayGen, 
    Scene& scene, Film& film);
};