
#include "medium.hpp"
#include "scene.hpp"

Medium::Medium(float sigmaT, glm::vec3 sigmaS, 
  Model& model, BXDF* phaseFunc):
  sigmaT(sigmaT), sigmaS(sigmaS),
  bound(model), phaseFunc(phaseFunc) {

  CustomMesh* mesh = CustomMesh::CreatePoint(glm::vec3(0));
  particle = mesh->prims[0];
  particleMesh = mesh;
  particleMesh->bxdf = phaseFunc;
  particleMesh->medium = this;
}

void Medium::addToScene(Scene& scene) {
  bound.setMediumForAllMeshes(this);
  bound.setBxdfForAllMeshes(nullptr);
  scene.addModel(bound);
}

glm::vec3 Medium::sampleNextItsc(const Ray& ray, Intersection& itsc) {
  float t = SampleShape::sampler().expSampleMedium(sigmaT);
  if(t < itsc.t) {
    itsc.itscVtx.position = ray.pass(t);
    itsc.prim = particle;
    itsc.t = t;
  }
  return tr(itsc.t);
}