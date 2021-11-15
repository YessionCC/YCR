#pragma once

#include <glm/glm.hpp>

#include "itsc.hpp"
#include "texture.hpp"
#include "sampler.hpp"
#include "ray.hpp"
#include "utility.hpp"

#include "const.hpp"

#define _HasType(ctype, wtype) \
  (BXDF::BXDFType::wtype & ctype)
#define _IsType(ctype, wtype) \
  (BXDF::BXDFType::wtype == ctype)

class BXDF {
public:
  enum BXDFType {
    REFLECT = 0x001, // ray do not pass surafce
    TRANSMISSION = 0x002, // ray only pass surface
    RTBoth = 0x003, // both REFLECT and TRANSMISSION
    DELTA = 0x004, // dirac delta bxdf
    None = 0x000
  };

protected:
  int type;

public:
  virtual ~BXDF() {}
  BXDF(int type): type(type) {}

  inline int getType() const {return type;}

  // return brdf
  virtual glm::vec3 evaluate(
    const Intersection& itsc, const Ray& ray_o, const Ray& ray_i) = 0;
  // return brdf*|cos|/pdf (to reduce unneccessary calc)
  // this func directly calc beta(as pbrt)
  virtual glm::vec3 sample_ev(
    const Intersection& itsc, const Ray& ray_o, Ray& ray_i) = 0;
  // return pdf
  virtual float sample(
    const Intersection& itsc, const Ray& ray_o, Ray& ray_i) = 0;
};

class LambertianDiffuse: public BXDF {
private:
  Texture* texture;

public:
  LambertianDiffuse(Texture* tex): BXDF(BXDFType::REFLECT), texture(tex) {}

  glm::vec3 evaluate(
    const Intersection& itsc, const Ray& ray_o, const Ray& ray_i) override {
    return INV_PI*texture->tex2D(itsc.itscVtx.uv);
  }

  glm::vec3 sample_ev(
    const Intersection& itsc, const Ray& ray_o, Ray& ray_i) override {
    glm::vec2 dir_i = SampleShape::sampler().uniSampleDisk();
    glm::vec3 tanp(
      dir_i.x*glm::cos(dir_i.y), 
      dir_i.x*glm::sin(dir_i.y),
      glm::sqrt(1-glm::min(1.0f, dir_i.x*dir_i.x)));
    ray_i.o = itsc.itscVtx.position;
    ray_i.d = itsc.toWorldSpace(tanp);
    return texture->tex2D(itsc.itscVtx.uv);
  }

  float sample(
    const Intersection& itsc, const Ray& ray_o, Ray& ray_i) override {
    glm::vec2 dir_i = SampleShape::sampler().uniSampleDisk();
    glm::vec3 tanp(
      glm::sqrt(1-glm::min(1.0f, dir_i.x*dir_i.x)), 
      dir_i.x*glm::cos(dir_i.y), 
      dir_i.x*glm::sin(dir_i.y));
    ray_i.o = itsc.itscVtx.position;
    ray_i.d = itsc.toWorldSpace(tanp);
    return INV_PI*glm::cos(dir_i.x);
  }

};

class NoFrSpecular: public BXDF {
private:
  Texture* absorb;

public:
  NoFrSpecular(Texture* tex): 
    BXDF(BXDFType::DELTA|BXDFType::REFLECT), absorb(tex) {}

  glm::vec3 evaluate(
    const Intersection& itsc, const Ray& ray_o, const Ray& ray_i) override {
    return glm::vec3(0.0f);
  }

  glm::vec3 sample_ev(
    const Intersection& itsc, const Ray& ray_o, Ray& ray_i) override {
    ray_i.o = itsc.itscVtx.position;
    ray_i.d = Reflect(ray_o.d, itsc.itscVtx.normal);
    return absorb->tex2D(itsc.itscVtx.uv);
  }

  float sample(
    const Intersection& itsc, const Ray& ray_o, Ray& ray_i) override {
    ray_i.o = itsc.itscVtx.position;
    ray_i.d = Reflect(ray_o.d, itsc.itscVtx.normal);
    return 1.0f;
  }
};

// Fr specular and Fr transimission
class GlassSpecular: public BXDF {
private:
  Texture *absorbR, *absorbT; // reflect, transimission(refract)
  float eataI, eataT; // I, often air, T, medium self

public:
  GlassSpecular(Texture* tex, float eataI, float eataT): 
    BXDF(BXDFType::DELTA|BXDFType::RTBoth), absorbR(tex), absorbT(tex),
    eataI(eataI), eataT(eataT) {}

  GlassSpecular(Texture* texR, Texture* texT, float eataI, float eataT): 
    BXDF(BXDFType::DELTA), absorbR(texR), absorbT(texT),
    eataI(eataI), eataT(eataT) {}

  glm::vec3 evaluate(
    const Intersection& itsc, const Ray& ray_o, const Ray& ray_i) override {
    return glm::vec3(0.0f);
  }

  glm::vec3 sample_ev(
    const Intersection& itsc, const Ray& ray_o, Ray& ray_i) override {
    ray_i.o = itsc.itscVtx.position;
    float cosThetaI = itsc.cosTheta(ray_o.d);
    float cosThetaT; float et = eataT, ei = eataI;
    glm::vec3 normal(itsc.itscVtx.normal);
    if(cosThetaI < 0) {
      std::swap(et, ei);
      normal = -normal;
      cosThetaI = -cosThetaI;
    }
    bool res = Refract(ray_o.d, normal, ei, et, ray_i.d, cosThetaT);
    if(res) return absorbR->tex2D(itsc.itscVtx.uv);
    float fr = FrDielectric(cosThetaI, cosThetaT, ei, et);
    float u = SampleShape::sampler().get1();
    if(u < fr) {
      ray_i.d = Reflect(ray_o.d, normal);
      return absorbR->tex2D(itsc.itscVtx.uv); // fr*R/fr
    }
    else { // (1-fr)*eataI2/eataT2*R/(1-fr)
      // NOTICE: no symmetric item!
      // Here Assume trace from camera, 
      // eataI should be the light enter side, so here use et
      return (et*et/(ei*ei))*absorbT->tex2D(itsc.itscVtx.uv);
    }
  }

  float sample(
    const Intersection& itsc, const Ray& ray_o, Ray& ray_i) override {
    ray_i.o = itsc.itscVtx.position;
    float cosThetaI = itsc.cosTheta(ray_o.d);
    float cosThetaT; float et = eataT, ei = eataI;
    glm::vec3 normal(itsc.itscVtx.normal);
    if(cosThetaI < 0) {
      std::swap(et, ei);
      normal = -normal;
    }
    bool res = Refract(ray_o.d, normal, ei, et, ray_i.d, cosThetaT);
    if(res) return 1.0f;
    float fr = FrDielectric(cosThetaI, cosThetaT, ei, et);
    float u = SampleShape::sampler().get1();
    if(u < fr) return fr;
    else return 1.0f - fr;
  }
};