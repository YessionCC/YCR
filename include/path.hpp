#pragma once

#include <glm/glm.hpp>
#include <vector>

#include "film.hpp"
#include "scene.hpp"
#include "bxdf.hpp"
#include "camera.hpp"
#include "medium.hpp"

struct PathVertex {
  Intersection itsc;
  glm::vec3 dir_o;
  glm::vec3 beta;
  const Medium* inMedium;
  const BXDF* bxdf;
};

class SubPathGenerator {
public:
  enum TerminateState {
    UpToMaxBounce,
    ItscLight,
    NoItsc,
    NoBXDF,
    RRFailed,
    TotalBlack,
    CalcERROR,
    None
  };

private:
  std::vector<PathVertex> pathVertices;
  TerminateState tstate = TerminateState::None;
  glm::vec3 diracRadiance; // first light bounce or specular light

public: 
  // return real bounce
  // TODO: Russia Roulette for BDPT ?
  int createSubPath(const Ray& start_ray, const Scene& scene, int max_bounce);

  void clear() {
    diracRadiance = glm::vec3(0.0f);
    pathVertices.clear(); 
    tstate = TerminateState::None;
  }

  inline std::vector<PathVertex>& getPathVtxs() {return pathVertices;}

  inline TerminateState getTerminateState() const {return tstate;}

  inline glm::vec3 getDiracLight() const {return diracRadiance;}
};

class PathIntegrator {
private:
  using PathTerminateState = SubPathGenerator::TerminateState;
  const int max_bounce = 12;

  // return pdf (respect to solid angle)
  void estimateDirectLightByLi(const Scene& scene, const DiscreteDistribution1D& ldd1d,
    PathVertex& pvtx, glm::vec3& L, bool needMIS) const ;

  void estimateDirectLightByBSDF(const Scene& scene, const DiscreteDistribution1D& ldd1d,
    PathVertex& pvtx, glm::vec3& L, bool needMIS) const;

public:
  void render(const Scene& scene, SubPathGenerator& subpathGen,
    RayGenerator& rayGen, Film& film) const;
  void visualizeRender(PCShower& pc, RayGenerator& rayGen, 
    Scene& scene, Film& film);
};