#pragma once

#include <glm/glm.hpp>

#include "itsc.hpp"
#include "distribution.hpp"
#include "texture.hpp"
#include "utility.hpp"
#include "bb3.hpp"

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
  // return ray pdf_A*pdf_dir
  virtual void genRay(Intersection& itsc, Ray& ray, float& pdf_A, float& pdf_D) const = 0;
  // itsc: litsc
  virtual float getRayPdf(const Intersection& itsc, glm::vec3 dir) const = 0;

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

  void genRay(Intersection& itsc, Ray& ray, float& pdf_A, float& pdf_D) const;

  float getRayPdf(const Intersection& itsc, glm::vec3 dir) const {
    return itsc.itscVtx.cosTheta(dir)<0? 0.0:1.0f/PI2;
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

  void genRay(Intersection& itsc, Ray& ray, float& pdf_A, float& pdf_D) const;

  inline float getItscPdf(const Intersection& itsc, const Ray& rayToLight) const{
    return 1.0f;
  }

  float getRayPdf(const Intersection& itsc, glm::vec3 dir) const {
    return 1.0f/PI4;
  }

};

class DirectionalLight: public Light { 
private:  
  glm::vec3 le;
  glm::vec3 direction;
  BB3 worldBB3;
  float sceneDiameter;

public:
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
    return 4.0f/(PI*sceneDiameter*sceneDiameter);
  }

  void genRay(Intersection& itsc, Ray& ray, float& pdf_A, float& pdf_D) const {
    pdf_D = 1.0f;
    pdf_A = getItscOnLight(itsc, worldBB3.uniSampleAPointInside());
    ray.o = itsc.itscVtx.position;
    ray.d = direction;
  }

  inline float getItscPdf(const Intersection& itsc, const Ray& rayToLight) const{
    return 4.0f/(PI*sceneDiameter*sceneDiameter);
  }

  float getRayPdf(const Intersection& itsc, glm::vec3 dir) const {
    return 0.0f;
  }

};

class EnvironmentLight: public Light { 
private:  
  const Texture* environment;
  BB3 worldBB3;
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

  void genRay(Intersection& itsc, Ray& ray, float& pdf_A, float& pdf_D) const {
    pdf_D = 1.0f;
    pdf_A = getItscOnLight(itsc, glm::vec3(0.0f));
    glm::vec3 p = worldBB3.uniSampleAPointInside();
    ray.d = itsc.itscVtx.normal;
    itsc.itscVtx.position += p;
    ray.o = itsc.itscVtx.position;
  }

  // used for BXDF sample
  void genRayItsc(Intersection& itsc, const Ray& rayToLight, glm::vec3 evaP) const;

  float getRayPdf(const Intersection& itsc, glm::vec3 dir) const {
    return 0.0f;
  }

};