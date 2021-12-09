#include "material.hpp"

#include "itsc.hpp"
#include "medium.hpp"
#include "light.hpp"
#include "texture.hpp"
#include "bxdf.hpp"

float Material::getBXDF(
  const Intersection& itsc, const Ray& ray_o, const BXDF*& bxdfNode) const {
  const BXDFNode* tnode;
  float weight = this->bxdfNode->getBXDF(itsc, ray_o, tnode);
  // we always promiss a bxdf is the leaf node
  bxdfNode = dynamic_cast<const BXDF*>(tnode);
  return weight;
}

void Material::bumpMapping(Intersection& itsc) const {
  if(!normalMap) return;
  glm::vec3 normal = normalMap->tex2D(itsc.itscVtx.uv);
  normal = glm::normalize(normal)*2.0f - 1.0f;
  normal = glm::normalize(itsc.toWorldSpace(normal));
  // reconstruct tbn coord.
  itsc.itscVtx.btangent = glm::normalize(glm::cross(itsc.itscVtx.tangent, normal));
  itsc.itscVtx.tangent = glm::cross(normal, itsc.itscVtx.btangent);
  itsc.itscVtx.normal = normal;
}