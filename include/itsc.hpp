#pragma once 

#include <glm/glm.hpp>

#include <cmath>

#include "vertex.hpp"
#include "const.hpp"
#include "ray.hpp"

class Primitive;

class Intersection {
public:
  float t;
  // all interpolated when triangle
  Vertex itscVtx;
  // NOTICE when use uv map, do not use it !! use itscVtx.uv
  // localUV used in triangle intersect, 
  // some custom primitive intersect will not use it
  glm::vec2 localUV;
  glm::vec3 itscError; // the error bound of the itsc point
  glm::vec3 geoNormal; // the real normal for the surface
  Primitive* prim;

  Intersection(): t(FLOAT_MAX), localUV(0), 
    itscError(0), prim(nullptr) {}

  // NOTICE: make sure t > 0 !
  inline void updateItscInfo(float t, Primitive* prim, const Ray& ray) {
    if(t < this->t) {
      this->t = t;
      this->prim = prim;
      this->itscVtx.position = ray.pass(t);
    }
  }

  // NOTICE: make sure t > 0 !
  inline void updateItscInfo(float t, Primitive* prim, glm::vec2 luv) {
    if(t < this->t) {
      this->t = t;
      this->prim = prim;
      this->localUV = luv;
    }
  }

  // offset pos by error and the relation of normal and dir
  inline void maxErrorOffset(glm::vec3 dir, glm::vec3& pos) {
    float errord = glm::dot(itscError, glm::abs(geoNormal));
    errord = std::nextafter(errord, FLOAT_MAX); //+1 ulp
    if(glm::dot(dir, geoNormal) > 0) pos += errord*geoNormal;
    else pos -= errord*geoNormal;
  }

  // from world space direction to calc tangent space angles
  
  // dot(w, n) should >0, with real geoNormal
  inline float cosTheta(glm::vec3 w) const{
    return glm::dot(geoNormal, w);
  }

  inline float theta(glm::vec3 w) const{
    return glm::acos(glm::dot(geoNormal, w));
  }

  // tan(phi), do not promiss valid return !
  inline float tanPhi(glm::vec3 w) const{
    return itscVtx.tanPhi(w);
  }

  // return 0~2pi
  inline float phi(glm::vec3 w) const{
    return itscVtx.phi(w);
  }

  // from tangent space to world space
  // tangent: x, btan: y, normal: z
  inline glm::vec3 toWorldSpace(glm::vec3 p) const{
    return itscVtx.toWorldSpace(p);
  }
};