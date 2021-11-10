#pragma once

#include <glm/glm.hpp>

#include "const.hpp"

class Vertex {
public:
  glm::vec3 position;
  glm::vec2 uv;

  // normal, tangent, btangent must normalized 
  // and perpendicular to each other
  glm::vec3 normal;
  // NOTICE Cross multiply two vectors that are not perpendicular. 
  // Even if they are normalized, the direction of the result is correct, 
  // but it is not normalized
  glm::vec3 tangent;
  glm::vec3 btangent;

  Vertex() {}
  Vertex(glm::vec3 pos):position(pos) {}

  void transform(const glm::mat4x4& trans) {
    glm::mat3x3 nTrans = glm::transpose(glm::inverse(glm::mat3x3(trans)));
    position = trans*glm::vec4(position,1.0f);
    normal = glm::normalize(nTrans*normal);
    tangent = glm::normalize(nTrans*tangent);
    btangent = glm::normalize(nTrans*btangent);
  }

  void transform(const glm::mat4x4& trans, const glm::mat3x3 transT_inv) {
    position = trans*glm::vec4(position,1.0f);
    normal = glm::normalize(transT_inv*normal);
    tangent = glm::normalize(transT_inv*tangent);
    btangent = glm::normalize(transT_inv*btangent);
  }

  // dot(w, n) should >0
  inline float cosTheta(glm::vec3 w) const{
    return glm::dot(w, normal);
  }

  inline float theta(glm::vec3 w) const{
    return glm::acos(glm::dot(w, normal));
  }

  // tan(phi), do not promiss valid return !
  inline float tanPhi(glm::vec3 w) const{
    return glm::dot(btangent, w)/glm::dot(tangent, w);
  }

  // return 0~2pi
  inline float phi(glm::vec3 w) const{
    float res = std::atan2(
      glm::dot(btangent, w), 
      glm::dot(tangent, w));
    if(res < 0) res+=PI2;
    return res;
  }

  // from tangent space to world space
  // tangent: x, btan: y, normal: z
  inline glm::vec3 toWorldSpace(glm::vec3 p) const{
    return p.x*tangent+p.y*btangent+p.z*normal;
  }

  inline glm::vec3 toTangentSpace(glm::vec3 p) const {
    return {glm::dot(tangent, p), glm::dot(btangent, p), glm::dot(normal, p)};
  }
};