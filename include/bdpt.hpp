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
  float fwdPdf = -1.0f;
  float revPdf = -1.0f;
  PathVertex(): bxdf(nullptr), light(nullptr), inMedium(nullptr) {}
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
  int max_sub_path_bounce;

public:
  BDPTIntegrator(int max_sub_bounce = 6): max_sub_path_bounce(max_sub_bounce) {}
  void render(const Scene& scene, RayGenerator& rayGen, Film& film) const;
};