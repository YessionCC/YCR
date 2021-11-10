#pragma once

#include <glm/glm.hpp>

#include "debug/pcshow.hpp"

class Ray {
public:
  // NOTICE: must keep direction normalized!
  glm::vec3 o, d;

  inline void normalizeD() {d = glm::normalize(d);}
  inline glm::vec3 pass(float t) const {return o+t*d;}

  // transform dir to x axis, make sure dir has normalized
  inline glm::mat3x3 getTranform() { 
    glm::vec3 ya;
    if(d.x<d.y && d.x<d.z) ya = glm::vec3(0, -d.z, d.y);
    else if(d.y<d.z) ya = glm::vec3(-d.z, 0, d.x);
    else ya = glm::vec3(-d.x, d.y, 0);
    glm::vec3 za = glm::cross(d, ya);
    return {d.x,d.y,d.z,ya.x,ya.y,ya.z,za.x,za.y,za.z};
  }
  // transform to world, make sure dir has normalized
  inline glm::mat3x3 getInvTransform() {
    return glm::transpose(getTranform());
  }

  void saveSegLineAsPointCloud(PCShower& pc, float t, 
    glm::vec3 col, int pNum = 20) {
    for(int i=0; i<pNum; i++) {
      pc.addItem(pass(t*i/pNum), col);
    }
  }
};