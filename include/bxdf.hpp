#pragma once

#include <glm/glm.hpp>

#include "itsc.hpp"
#include "texture.hpp"
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
    MEDIUM = 0x008, // medium phase function
    None = 0x000
  };

protected:
  int type;

public:
  virtual ~BXDF() {}
  BXDF(int type): type(type) {}

  inline int getType() const {return type;}

  // return brdf*|cos|
  virtual glm::vec3 evaluate(
    const Intersection& itsc, const Ray& ray_o, const Ray& ray_i) const = 0;
  // return brdf*|cos|/pdf (to reduce unneccessary calc)
  // this func directly calc beta(as pbrt)
  virtual glm::vec3 sample_ev(
    const Intersection& itsc, const Ray& ray_o, Ray& ray_i) const = 0;
};

/************************LambertianDiffuse******************************/

class LambertianDiffuse: public BXDF {
private:
  const Texture* texture;

public:
  LambertianDiffuse(const Texture* tex): BXDF(BXDFType::REFLECT), texture(tex) {}

  glm::vec3 evaluate(
    const Intersection& itsc, const Ray& ray_o, const Ray& ray_i) const override;
  glm::vec3 sample_ev(
    const Intersection& itsc, const Ray& ray_o, Ray& ray_i) const override;
};

/************************NoFrSpecular******************************/

class PureTransmission: public BXDF { // for no surface medium
public:
  PureTransmission(): 
    BXDF(BXDFType::DELTA|BXDFType::TRANSMISSION|BXDFType::MEDIUM) {}

  inline glm::vec3 evaluate(
    const Intersection& itsc, const Ray& ray_o, const Ray& ray_i) const override {
    return glm::vec3(0.0f);
  }

  glm::vec3 sample_ev(
    const Intersection& itsc, const Ray& ray_o, Ray& ray_i) const override {
    ray_i.o = ray_o.o;
    ray_i.d = -ray_o.d;
    return glm::vec3(1.0f);
  }
};

/************************NoFrSpecular******************************/

class NoFrSpecular: public BXDF {
private:
  const Texture* absorb;

public:
  NoFrSpecular(const Texture* tex): 
    BXDF(BXDFType::DELTA|BXDFType::REFLECT), absorb(tex) {}

  inline glm::vec3 evaluate(
    const Intersection& itsc, const Ray& ray_o, const Ray& ray_i) const override {
    return glm::vec3(0.0f);
  }

  glm::vec3 sample_ev(
    const Intersection& itsc, const Ray& ray_o, Ray& ray_i) const override;
};

/************************GlassSpecular******************************/

// Fr specular and Fr transimission
class GlassSpecular: public BXDF {
private:
  const Texture *absorbR, *absorbT; // reflect, transimission(refract)
  float eataI, eataT; // I, often air, T, medium self

public:
  GlassSpecular(const Texture* tex, float eataI, float eataT): 
    BXDF(BXDFType::DELTA|BXDFType::RTBoth), absorbR(tex), absorbT(tex),
    eataI(eataI), eataT(eataT) {}

  GlassSpecular(const Texture* texR, const Texture* texT, float eataI, float eataT): 
    BXDF(BXDFType::DELTA), absorbR(texR), absorbT(texT),
    eataI(eataI), eataT(eataT) {}

  inline glm::vec3 evaluate(
    const Intersection& itsc, const Ray& ray_o, const Ray& ray_i) const override {
    return glm::vec3(0.0f);
  }

  glm::vec3 sample_ev(
    const Intersection& itsc, const Ray& ray_o, Ray& ray_i) const override;
};

/************************HenyeyPhase******************************/

class HenyeyPhase : public BXDF {
private:
  float g;
  glm::vec3 sigmaS;

private:
  float samplePhaseCosTheta() const;

public:
  HenyeyPhase(float g, glm::vec3 sigmaS): BXDF(BXDFType::MEDIUM), 
    g(g), sigmaS(sigmaS) {}

  // return phase distribution value
  inline glm::vec3 evaluate(
    const Intersection& itsc, const Ray& ray_o, const Ray& ray_i) const {
    float cosTheta = glm::dot(ray_o.d, ray_i.d);
    return glm::vec3(0.25f*INV_PI*(1-g*g)/ (1+g*g+2*g*cosTheta));
  }

  glm::vec3 sample_ev(
    const Intersection& itsc, const Ray& ray_o, Ray& ray_i) const ;
};