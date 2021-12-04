#pragma once

#include <glm/glm.hpp>

inline float Luminance(glm::vec3 le) {
  return 0.299f*le.x+0.587f*le.y+0.114f*le.z;
}

// dir_o and normal in same side
inline glm::vec3 Reflect(glm::vec3 dir_o, glm::vec3 normal) {
  return -dir_o + 2.0f*glm::dot(dir_o, normal)*normal;
}

// dir_o and normal in same side
// if return true: total reflect, and dir_refr = dir_refl
inline bool Refract(glm::vec3 dir_o, glm::vec3 normal, 
  float eataI, float eataT, glm::vec3& dir_refr, float& cosThetaT) {
  
  float idt = eataI / eataT;
  float cosThetaI = glm::dot(dir_o, normal);
  float sinThetaI = glm::sqrt(1 - glm::min(1.0f, cosThetaI*cosThetaI));
  float sinThetaT = idt*sinThetaI;
  if(sinThetaT>1.0f) {
    dir_refr = -dir_o + 2.0f*cosThetaI*normal;
    return true;
  }
  else {
    cosThetaT = glm::sqrt(1-glm::min(1.0f, sinThetaT*sinThetaT));
    dir_refr = -idt*dir_o+(idt*glm::dot(dir_o, normal) - cosThetaT)*normal;
    return false;
  }
}

// Fr is symmetric(I<->T, swap and value unchanged)
inline float FrDielectric(
  float cosThetaI, float cosThetaT, float eataI, float eataT) {
  float r1 = (eataT*cosThetaI - eataI*cosThetaT)/
    (eataT*cosThetaI + eataI*cosThetaT);
  float r2 = (eataI*cosThetaI - eataT*cosThetaT)/
    (eataI*cosThetaI + eataT*cosThetaT);
  return (r1*r1+r2*r2) / 2.0f;
}

// only take reflection into account
inline float FrDielectricReflect(
  glm::vec3 dir_o, glm::vec3 normal, float eataI, float eataT) {
  glm::vec3 t1; float cosThetaT = 0.5f*PI;
  Refract(dir_o, normal, eataI, eataT, t1, cosThetaT); // never return true
  float cosThetaI = glm::dot(dir_o, normal);
  float t = FrDielectric(cosThetaI, cosThetaT, eataI, eataT);
  return t;
}

inline float PaToPw(float pdf, float dist2, float cosTheta) {
  return pdf*dist2/cosTheta;
}

inline float PwToPa(float pdf, float dist2, float cosTheta) {
  return pdf*cosTheta/dist2;
}

// pdf1**2 / (pdf1**2+pdf2**2)
inline float PowerHeuristicWeight(float pdf1, float pdf2) {
  return 1.0f/(1.0f+(pdf2/pdf1)*(pdf2/pdf1));
}

inline void CheckRadiance(glm::vec3 L, glm::vec2 rasPos) {
  if(L.x<0 || L.y<0 || L.z<0) {
    std::cout<<"find negative radiance: "
      <<rasPos.x<<" "<<rasPos.y<<std::endl;
  }
  if(L.x>1e5 || L.y>1e5 || L.z>1e5) {
    std::cout<<"find huge radiance: "
      <<rasPos.x<<" "<<rasPos.y<<std::endl;
  }
  if(std::isnan(L.x) || std::isnan(L.y) || std::isnan(L.z)) {
    std::cout<<"find NaN radiance: "
      <<rasPos.x<<" "<<rasPos.y<<std::endl;
  }
}

inline bool IsBlack(glm::vec3 L) {
  return L.x == 0.0f && L.y == 0.0f && L.z ==0.0f;
}