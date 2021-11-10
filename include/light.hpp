#pragma once

#include <glm/glm.hpp>

class Light { // Area(Tri-Mesh) light

private:  
  float luminance;
  glm::vec3 le;

public:
  Light(glm::vec3 le): le(le) {
    luminance = 0.299f*le.x+0.587f*le.y+0.114f*le.z;
  }

  inline float getLuminance() const {return luminance;}

  // calc Le*cosTheta, dir in world space, dir point to outside surface
  glm::vec3 evaluate(const Intersection& itsc, glm::vec3 dir) const {
    return le*glm::max(0.0f, itsc.cosTheta(dir));
  }

};