#pragma once

#include <vector>

#include "film.hpp"
#include "bxdf.hpp"
#include "medium.hpp"
#include "integrator.hpp"

struct PathVertex {
  Intersection itsc;
  glm::vec3 dir_o;
  glm::vec3 beta;
  const BXDF* bxdf;
  const Light* light;
  const Medium* inMedium;
  float fwdPdf = -1.0f, revPdf = -1.0f;
  PathVertex() {}
  PathVertex(const Intersection& itsc, glm::vec3 dir_o, glm::vec3 beta, 
    const BXDF* bxdf = nullptr, const Light* light = nullptr, 
    const Medium* inm = nullptr):
    itsc(itsc), dir_o(dir_o), beta(beta), bxdf(bxdf), 
    light(light), inMedium(inm) {}
};

class SubPathGenerator{
public:
  enum TerminateState {
    NoItsc,
    NoBXDF,
    TotalBlack,
    CalcERROR,
    UpToMaxBounce,
    None
  };

  std::vector<PathVertex> pathVerticesLt;
  std::vector<PathVertex> pathVerticesCam;
  TerminateState tstateLt = TerminateState::None;
  TerminateState tstateCam = TerminateState::None;

private:
  void createSubPath(
    const Ray& start_ray, const Scene& scene, 
    std::vector<PathVertex>& pathVertices, 
    TerminateState& tstate, int max_bounce);

public: 
  void createSubPath(const Ray& start_ray, const Scene& scene, 
    TMode tmode, PathVertex& iniVtx, int max_bounce);

  void clear() {
    pathVerticesLt.clear(); 
    pathVerticesCam.clear(); 
    tstateLt = TerminateState::None;
    tstateCam = TerminateState::None;
  }
};



class BDPTIntegrator: public Integrator {
private:
  const int max_sub_path_bounce = 6;

public:
  void render(const Scene& scene, RayGenerator& rayGen, Film& film) const;
};