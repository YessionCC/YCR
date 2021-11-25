#pragma once

#include <glm/glm.hpp>

#include "itsc.hpp"
#include "distribution.hpp"
#include "texture.hpp"
#include "utility.hpp"

class Scene;
class Model;

class Light {
public:
  virtual ~Light() {}
  virtual float selectProbality(const Scene& scene) = 0;
  // return pdf
  virtual float getItscOnLight(Intersection& itsc, glm::vec3 evaP) const = 0;
  virtual glm::vec3 evaluate(const Intersection& itsc, glm::vec3 dir) const = 0;
  virtual void addToScene(Scene& scene) = 0;
};

class ShapeLight: public Light { 
private:  
  glm::vec3 le;
  float selectP;
  Model& model;
  DiscreteDistribution1D dist;
  std::vector<const Primitive*> vp;

public:
  ShapeLight(glm::vec3 le, Model& shape);

  void addToScene(Scene& scene);

  float selectProbality(const Scene& scene);

  // calc Le*cosTheta, dir in world space, dir point to outside surface
  glm::vec3 evaluate(const Intersection& itsc, glm::vec3 dir) const;

  float getItscOnLight(Intersection& itsc, glm::vec3 evaP) const;

};

class PointLight: public Light { 
private:  
  glm::vec3 le;
  glm::vec3 position;

public:
  // le falloff by default square relationship
  PointLight(glm::vec3 le, glm::vec3 pos): le(le), position(pos){}

  void addToScene(Scene& scene) {}

  float selectProbality(const Scene& scene) {return Luminance(le);}

  // calc Le*cosTheta, dir in world space, dir point to outside surface
  glm::vec3 evaluate(const Intersection& itsc, glm::vec3 dir) const {return le;}

  float getItscOnLight(Intersection& itsc, glm::vec3 evaP) const {
    itsc.itscVtx.position = position;
    itsc.prim = nullptr; // important
    return 1.0f;
  }

};

class DirectionalLight: public Light { 
private:  
  glm::vec3 le;
  glm::vec3 direction;
  float sceneDiameter;

public:
  // le do not falloff because we assume the light in the infinite distance
  // so you should set a small value to le
  DirectionalLight(glm::vec3 le, glm::vec3 dir): 
    le(le), direction(glm::normalize(dir)){}

  void addToScene(Scene& scene) {}

  float selectProbality(const Scene& scene);

  // calc Le*cosTheta, dir in world space, dir point to outside surface
  glm::vec3 evaluate(const Intersection& itsc, glm::vec3 dir) const {
    return sceneDiameter*sceneDiameter*le;
  }

  float getItscOnLight(Intersection& itsc, glm::vec3 evaP) const {
    itsc.itscVtx.position = evaP - sceneDiameter*direction;
    itsc.prim = nullptr; // important
    return 1.0f;
  }

};

class EnvironmentLight: public Light { 
private:  
  const Texture* environment;
  float sceneDiameter;
  float avgLuminance;
  float lumiScale;
  bool isSolid = false;
  DiscreteDistribution2D dd2d;

public:
  // le falloff by default square relationship
  EnvironmentLight(const Texture* tex, float scale = 1.0f);

  void addToScene(Scene& scene);

  float selectProbality(const Scene& scene);

  // calc Le*cosTheta, dir in world space, dir point to outside surface
  glm::vec3 evaluate(const Intersection& itsc, glm::vec3 dir) const;

  float getItscOnLight(Intersection& itsc, glm::vec3 evaP) const;

};