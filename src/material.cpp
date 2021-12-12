#include "material.hpp"

#include "itsc.hpp"
#include "medium.hpp"
#include "light.hpp"
#include "texture.hpp"
#include "bxdf.hpp"
#include "bxdfc.hpp"

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

void Material::handleMaterialInfo(const MaterialInfo& minfo) {
  if(minfo.normal.size()) this->normalMap = minfo.normal[0];
  // if(minfo.lightMap.size()) this->light = minfo.lightMap[0];
  if(!minfo.specular.size()) this->bxdfNode = new LambertianReflection(minfo.diffuse[0]);
  else if(!minfo.glossy.size()) 
    this->bxdfNode = new PrincipleBSDF1(minfo.diffuse[0], minfo.specular[0]);
  else if(!minfo.opacity.size()) 
    this->bxdfNode = new PrincipleBSDF1(
      minfo.diffuse[0], minfo.specular[0], minfo.glossy[0]);
  else this->bxdfNode = new PrincipleBSDF1(
    minfo.diffuse[0], minfo.specular[0], minfo.glossy[0], minfo.opacity[0], minfo.IOR);
}