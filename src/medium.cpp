
#include "medium.hpp"
#include "scene.hpp"
#include "sampler.hpp"

Medium::Medium(float sigmaT, glm::vec3 sigmaS, 
  Model* model, const BXDF* phaseFunc):
  sigmaT(sigmaT), sigmaS(sigmaS),
  bound(model), phaseFunc(phaseFunc) {

  CustomMesh* mesh = CustomMesh::CreatePoint(glm::vec3(0));
  particle = mesh->prims[0];
  particleMesh = mesh;
  particleMesh->material.bxdfNode = phaseFunc;
  particleMesh->material.mediumInside = this;
}

void Medium::addToScene(Scene& scene, bool noBXDF) {
  if(!bound) return;
  bound->setMediumForAllMeshes(this);
  bound->setMeshPurposes(Mesh::MeshPurpose::MediumBound);
  if(noBXDF) bound->setBxdfForAllMeshes(new PureTransmission()); // memory manager notice 
  scene.addModel(*bound);
}

glm::vec3 Medium::sampleNextItsc(const Ray& ray, Intersection& itsc) const{
  float t = _ThreadSampler.expSampleMedium(sigmaT);
  if(t < itsc.t) {
    itsc.itscVtx.position = ray.pass(t);
    itsc.prim = particle;
    itsc.t = t;
  }
  return tr(itsc.t);
}