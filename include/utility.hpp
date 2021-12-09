#pragma once

#include <glm/glm.hpp>

inline float Luminance(glm::vec3 le) {
  return 0.299f*le.x+0.587f*le.y+0.114f*le.z;
}

// dir_o and normal in same side
inline glm::vec3 Reflect(glm::vec3 dir_o, glm::vec3 normal) {
  return -dir_o + 2.0f*glm::dot(dir_o, normal)*normal;
}

// do not auto reverse normal
// if return false: total reflect, no refract dir will return
inline bool Refract(glm::vec3 dir_o, glm::vec3 normal, 
  float IOR, glm::vec3& dir_refr) {
  
  float idt = 1.0f / IOR;
  float cosThetaI = glm::dot(dir_o, normal);
  float sinThetaI = glm::sqrt(1 - glm::min(1.0f, cosThetaI*cosThetaI));
  float sinThetaT = idt*sinThetaI;
  if(sinThetaT>1.0f) return false;
  else {
    float cosThetaT = glm::sqrt(1-glm::min(1.0f, sinThetaT*sinThetaT));
    dir_refr = -idt*dir_o+(idt*glm::dot(dir_o, normal) - cosThetaT)*normal;
    return true;
  }
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