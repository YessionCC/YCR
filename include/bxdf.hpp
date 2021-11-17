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
    MEDIUM = 0x008, // medium phase function
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

/************************LambertianDiffuse******************************/

class LambertianDiffuse: public BXDF {
private:
  Texture* texture;

public:
  LambertianDiffuse(Texture* tex): BXDF(BXDFType::REFLECT), texture(tex) {}

  glm::vec3 evaluate(
    const Intersection& itsc, const Ray& ray_o, const Ray& ray_i) override;
  glm::vec3 sample_ev(
    const Intersection& itsc, const Ray& ray_o, Ray& ray_i) override;
  float sample(
    const Intersection& itsc, const Ray& ray_o, Ray& ray_i) override;
};

/************************NoFrSpecular******************************/

class NoFrSpecular: public BXDF {
private:
  Texture* absorb;

public:
  NoFrSpecular(Texture* tex): 
    BXDF(BXDFType::DELTA|BXDFType::REFLECT), absorb(tex) {}

  inline glm::vec3 evaluate(
    const Intersection& itsc, const Ray& ray_o, const Ray& ray_i) override {
    return glm::vec3(0.0f);
  }

  glm::vec3 sample_ev(
    const Intersection& itsc, const Ray& ray_o, Ray& ray_i) override;

  float sample(
    const Intersection& itsc, const Ray& ray_o, Ray& ray_i) override;
};

/************************GlassSpecular******************************/

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

  inline glm::vec3 evaluate(
    const Intersection& itsc, const Ray& ray_o, const Ray& ray_i) override {
    return glm::vec3(0.0f);
  }

  glm::vec3 sample_ev(
    const Intersection& itsc, const Ray& ray_o, Ray& ray_i) override;

  float sample(
    const Intersection& itsc, const Ray& ray_o, Ray& ray_i) override;
};

/************************HenyeyPhase******************************/

class HenyeyPhase : public BXDF {
private:
  float g;
  glm::vec3 sigmaS;

private:
  inline float samplePhaseCosTheta() const { //
    float cosTheta;
    float u = SampleShape::sampler().get1();
    if(glm::abs(g) < 1e-3) cosTheta = 1.f-2.f*u;
    else {
      float t = (1-g*g)/(1-g+2*g*u);
      cosTheta = (1+g*g-t*t)/(2*g);
    }
    return cosTheta;
  }

public:
  HenyeyPhase(float g, glm::vec3 sigmaS): BXDF(BXDFType::MEDIUM), 
    g(g), sigmaS(sigmaS) {}

  // return phase distribution value
  inline glm::vec3 evaluate(
    const Intersection& itsc, const Ray& ray_o, const Ray& ray_i) {
    float cosTheta = glm::dot(ray_o.d, ray_i.d);
    return glm::vec3(0.25f*INV_PI*(1-g*g)/ (1+g*g+2*g*cosTheta));
  }

  glm::vec3 sample_ev(
    const Intersection& itsc, const Ray& ray_o, Ray& ray_i);

  float sample(
    const Intersection& itsc, const Ray& ray_o, Ray& ray_i) {
    std::cout<<"ERROR: HenyeyPhase::sample do not implement!"<<std::endl;
    return 1.0f;
  }
};