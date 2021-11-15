#pragma once

#include <glm/glm.hpp>
#include <algorithm>
#include "const.hpp"
#include "ray.hpp"

class BB3 {
private:
  glm::vec3 pMin, pMax;

public:
  BB3():pMin(FLOAT_MAX), pMax(FLOAT_MIN) {}
  BB3(glm::vec3 p) {pMin = pMax = p;}
  BB3(glm::vec3 pMin, glm::vec3 pMax): pMin(pMin), pMax(pMax){}

  inline void update(const glm::vec3 p) {
    pMin = glm::min(pMin, p);
    pMax = glm::max(pMax, p);
  }

  inline BB3 Union(const BB3& bb3) const{
    BB3 res;
    res.pMax = glm::max(pMax, bb3.pMax);
    res.pMin = glm::min(pMin, bb3.pMin);
    return res;
  }

  inline BB3 Union_(const BB3& bb3){ // in place
    pMax = glm::max(pMax, bb3.pMax);
    pMin = glm::min(pMin, bb3.pMin);
    return *this;
  }

  inline glm::vec3 getRelativePos(const glm::vec3 pos) { // world pos -> pos relative in bb3(0~1)
    return (pos - pMin)/(pMax - pMin);
  }

  inline float getSurfaceArea() {
    glm::vec3 t = pMax-pMin;
    return 2*(t.x*t.y+t.x*t.z+t.y*t.z);
  }

  inline float getVolume() {
    glm::vec3 t = pMax-pMin;
    return t.x*t.y*t.z;
  }

  inline int getMaxAxis() { 
    glm::vec3 t = pMax-pMin;
    if(t.x > t.y && t.x>t.z) return 0;
    if(t.y > t.z) return 1;
    return 2;
  }

  inline void getAxisOrder(int *ord) {
    glm::vec3 t = pMax-pMin;
    if(t.x>t.y) {
      if(t.y>t.z) ord[0]=0, ord[1]=1, ord[2] = 2;
      else if(t.x>t.z) ord[0]=0, ord[1]=2, ord[2] = 1;
      else ord[0]=1, ord[1]=2, ord[2] = 0;
    }
    else {
      if(t.x>t.z) ord[0]=1, ord[1]=0, ord[2] = 2;
      else if(t.y>t.z) ord[0]=2, ord[1]=0, ord[2] = 1;
      else ord[0]=2, ord[1]=1, ord[2] = 0;
    }
  }

  inline glm::vec3 getCenter() {
    return 0.5f*(pMin+pMax);
  }

  inline float getDiagonalLength() {
    return glm::length(pMax - pMin);
  }

  inline void get8Cornor(glm::vec3* cs) {
    cs[0] = pMin;
    cs[1] = glm::vec3(pMin.x, pMax.y, pMin.z);
    cs[2] = glm::vec3(pMax.x, pMax.y, pMin.z);
    cs[3] = glm::vec3(pMax.x, pMin.y, pMin.z);
    cs[4] = glm::vec3(pMin.x, pMin.y, pMax.z);
    cs[5] = glm::vec3(pMin.x, pMax.y, pMax.z);
    cs[6] = pMax;
    cs[7] = glm::vec3(pMax.x, pMin.y, pMax.z);
  }

  inline bool intersect(const Ray& ray, float& tMin, float& tMax){
    tMin = FLOAT_MIN, tMax = FLOAT_MAX;
    return itsc_help(ray, tMin, tMax, 0) &&
      itsc_help(ray, tMin, tMax, 1) &&
      itsc_help(ray, tMin, tMax, 2) &&
      tMin <= tMax && tMax > 0;
  }
  
private:
 inline bool itsc_help(const Ray& ray, float& tMin, float& tMax, int axis) {
    if(std::abs(ray.d[axis])<CUSTOM_EPSILON) {
      if(ray.o[axis]>pMax[axis] || ray.o[axis]<pMin[axis])
        return false;
    }
    else {
      float t1 = (pMin[axis] - ray.o[axis])/ray.d[axis];
      float t2 = (pMax[axis] - ray.o[axis])/ray.d[axis];
      if(t1>t2) std::swap(t1, t2);
      if(t2<0) return false;
      tMin = std::max(tMin, t1);
      tMax = std::min(tMax, t2);
    }
    return true;
  }

};