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
  // TO BE Improved
  virtual float selectProbality(const Scene& scene) = 0;
  // return pdf
  virtual float getItscOnLight(Intersection& itsc, glm::vec3 evaP) const = 0;
  // if the itsc on the light, return the pdf sampling this itsc
  virtual float getItscPdf(const Intersection& itsc, const Ray& rayToLight) const = 0;
  // return le
  virtual glm::vec3 evaluate(const Intersection& itsc, glm::vec3 dir) const = 0;

  virtual void addToScene(Scene& scene) = 0;
};

class ShapeLight: public Light { 
private:  
  const Texture* lightMap; // do not important sample it
  Model& model;
  float selectP, totArea;
  DiscreteDistribution1D dist;
  std::vector<const Primitive*> vp;

public:
  ShapeLight(const Texture* ltMp, Model& shape);

  void addToScene(Scene& scene);

  inline float selectProbality(const Scene& scene) {
    return selectP;
  }

  // return le, dir in world space, dir point to outside surface
  inline glm::vec3 evaluate(const Intersection& itsc, glm::vec3 dir) const{
    return lightMap->tex2D(itsc.itscVtx.uv);
  }

  inline float getItscPdf(const Intersection& itsc, const Ray& rayToLight) const{
    return 1.0f/totArea;
  }

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

  inline float selectProbality(const Scene& scene) {return Luminance(le);}

  inline glm::vec3 evaluate(const Intersection& itsc, glm::vec3 dir) const {return le;}

  inline float getItscOnLight(Intersection& itsc, glm::vec3 evaP) const {
    itsc.itscVtx.position = position;
    itsc.itscVtx.normal = glm::normalize(evaP - position);
    itsc.prim = nullptr; // important
    return 1.0f;
  }

  inline float getItscPdf(const Intersection& itsc, const Ray& rayToLight) const{
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

  inline glm::vec3 evaluate(const Intersection& itsc, glm::vec3 dir) const {
    return le;
  }

  inline float getItscOnLight(Intersection& itsc, glm::vec3 evaP) const {
    itsc.itscVtx.position = evaP - sceneDiameter*direction;
    itsc.itscVtx.normal = direction;
    itsc.prim = nullptr; // important
    return 0.25f*PI*sceneDiameter*sceneDiameter;
  }

  inline float getItscPdf(const Intersection& itsc, const Ray& rayToLight) const{
    return 0.25f*PI*sceneDiameter*sceneDiameter;
  }

};

class EnvironmentLight: public Light { 
private:  
  const Texture* environment;
  float sceneDiameter;
  float avgLuminance;
  bool isSolid = false;
  DiscreteDistribution2D dd2d;

public:
  // le falloff by default square relationship
  EnvironmentLight(const Texture* tex);

  void addToScene(Scene& scene);

  float selectProbality(const Scene& scene);

  glm::vec3 evaluate(const Intersection& itsc, glm::vec3 dir) const;

  float getItscOnLight(Intersection& itsc, glm::vec3 evaP) const;

  float getItscPdf(const Intersection& itsc, const Ray& rayToLight) const;

};