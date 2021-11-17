#pragma once

#include "model.hpp"
#include "bxdf.hpp"
#include "mesh.hpp"
#include "primitive.hpp"

class Scene;

// Multiple media are allowed, but they are not allowed to overlap, 
// otherwise the results are unpredictable
class Medium {
private:
  float sigmaT; // determine thickness
  glm::vec3 sigmaS; // determine color
  // medium bound are not allowed overlapping!
  Model& bound;
  BXDF* phaseFunc;
  Mesh* particleMesh; // for assistance
  Primitive* particle;

public:
  ~Medium() {delete particleMesh;}
  Medium(float sigmaT, glm::vec3 sigmaS, Model& model, BXDF* phaseFunc);

  const Model& getBound() const {return bound;}

  void addToScene(Scene& scene);

  glm::vec3 sampleNextItsc(const Ray& ray, Intersection& itsc);

  glm::vec3 tr(float t) const {
    return glm::vec3(glm::exp(-sigmaT*t));
  }

};