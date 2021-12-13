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
// record EnvLight or NoBXDF Light, always the last vertex
struct LastVertex {
  Intersection itsc;
  glm::vec3 dir_o;
  glm::vec3 beta;
  const Light* lt;
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
  enum TransportMode {
    FromLight,
    FromCamera
  };
  static TransportMode transportMode;

private:
  std::vector<PathVertex> pathVertices;
  TerminateState tstate = TerminateState::None;
  glm::vec3 diracRadiance; // first light bounce or specular light
  LastVertex lastVertex;

public: 
  // return real bounce
  // TODO: Russia Roulette for BDPT ?
  int createSubPath(const Ray& start_ray, const Scene& scene, int max_bounce);

  void clear() {
    diracRadiance = glm::vec3(0.0f);
    pathVertices.clear(); 
    tstate = TerminateState::None;
    lastVertex.lt = nullptr;
  }

  inline std::vector<PathVertex>& getPathVtxs() {return pathVertices;}

  inline TerminateState getTerminateState() const {return tstate;}

  inline glm::vec3 getDiracLight() const {return diracRadiance;}

  inline const LastVertex& getLastVertex() const {return lastVertex;}
};

class Integrator{
public:
  virtual ~Integrator() {}
  virtual void render(const Scene& scene, SubPathGenerator& subpathGen,
    RayGenerator& rayGen, Film& film) const = 0;
};

class PathIntegrator: public Integrator {
private:
  using PathTerminateState = SubPathGenerator::TerminateState;
  const int max_bounce = 12;

  // return pdf (respect to solid angle)
  void estimateDirectLightByLi(const Scene& scene, const DiscreteDistribution1D& ldd1d,
    PathVertex& pvtx, glm::vec3& L, bool needMIS) const ;

  void estimateDirectLightByBSDF(
    const Scene& scene, const DiscreteDistribution1D& ldd1d,
    PathVertex& pvtx, glm::vec3& L, bool needMIS) const;

  void estimateDirectLightByBSDF(
    const Scene& scene, const DiscreteDistribution1D& ldd1d,
    PathVertex* pvtx1, PathVertex* pvtx2, const LastVertex* lstvtx,
    glm::vec3& L, bool needMIS) const;

public:
  void render(const Scene& scene, SubPathGenerator& subpathGen,
    RayGenerator& rayGen, Film& film) const;
};

class BDPTIntegrator: public Integrator {
private:
  using PathTerminateState = SubPathGenerator::TerminateState;
  using PathTransportMode = SubPathGenerator::TransportMode;
  const int max_bounce = 8;

  float calcMISWeight(
    const std::vector<PathVertex>& pcam,
    const std::vector<PathVertex>& plt,
    int s, int t) const; // s for light, t for camera

  void calcFwdPdfs(
    const std::vector<PathVertex>& pvtx, std::vector<float>& res) const;
  void calcRevPdfs(
    const std::vector<PathVertex>& pvtx, std::vector<float>& res) const;

public:
  void render(const Scene& scene, SubPathGenerator& subpathGen,
    RayGenerator& rayGen, Film& film) const;
};