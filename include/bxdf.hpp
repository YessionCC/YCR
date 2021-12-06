#pragma once

#include <glm/glm.hpp>

#include "itsc.hpp"
#include "texture.hpp"
#include "ray.hpp"
#include "blender.hpp"
#include "utility.hpp"

#include "material.hpp"

#include "const.hpp"

#define _IsType(ctype, wtype) \
  (!((BXDF::BXDFNature::wtype ^ (ctype))&0xf))
#define _HasFeature(ctype, wtype) \
  (BXDF::BXDFNature::wtype & (ctype))

class BXDF: public BXDFNode {
public:
  enum BXDFNature {
    // BXDF Type (mutual)
    REFLECT = 0x001, // ray do not pass surafce
    TRANSMISSION = 0x002, // ray only pass surface
    RTBoth = 0x003, // both REFLECT and TRANSMISSION
    NoSurface = 0x004, // usually medium particle(phase func)

    // BXDF Feature (combined)
    DELTA = 0x010, // dirac delta bxdf
    BSSRDF = 0x020,
    GLOSSY = 0x040, // used for MIS
    NONE = 0x000
  };

protected:
  int type;

public:
  virtual ~BXDF() {}
  BXDF(int type): type(type) {}

  inline virtual int getType() const {return type;}

  // return brdf*|cos|, always not return black
  virtual glm::vec3 evaluate(
    const Intersection& itsc, const Ray& ray_o, const Ray& ray_i) const = 0;
  // return brdf*|cos|/pdf (to reduce unneccessary calc)
  // this func directly calc beta(as pbrt)
  // if return black, the sample ray is invalid!
  virtual glm::vec3 sample_ev(
    const Intersection& itsc, const Ray& ray_o, Ray& ray_i) const = 0;

  // return pdf (respect to solid angle)
  virtual float sample_pdf(
    const Intersection& itsc, const Ray& ray_i, const Ray& ray_o) const = 0;

  inline float getBXDF(
    const Intersection& itsc, const Ray& ray_o, const BXDFNode*& bxdfNode) const override {
    bxdfNode = this;
    return 1.0f;
  }
  
  inline virtual bool needMIS(const Intersection& itsc) const {return false;}
};

/************************BSSRDF Base******************************/

class BSSRDF: public BXDF {

public:
  BSSRDF(): BXDF(BXDFNature::BSSRDF) {}

  glm::vec3 evaluate(
    const Intersection& itsc, const Ray& ray_o, const Ray& ray_i) const override;
  glm::vec3 sample_ev(
    const Intersection& itsc, const Ray& ray_o, Ray& ray_i) const override;
  float sample_pdf(
    const Intersection& itsc, const Ray& ray_i, const Ray& ray_o) const override;
};

/************************LambertianDiffuse******************************/

class LambertianDiffuse: public BXDF {
private:
  const Texture* texture;

public:
  LambertianDiffuse(const Texture* tex): BXDF(BXDFNature::REFLECT), texture(tex) {}

  glm::vec3 evaluate(
    const Intersection& itsc, const Ray& ray_o, const Ray& ray_i) const override;
  glm::vec3 sample_ev(
    const Intersection& itsc, const Ray& ray_o, Ray& ray_i) const override;
  float sample_pdf(
    const Intersection& itsc, const Ray& ray_i, const Ray& ray_o) const override;
};

/************************PerfectSpecular******************************/

class PureTransmission: public BXDF { // for no surface medium
public:
  PureTransmission(): 
    BXDF(BXDFNature::DELTA|BXDFNature::TRANSMISSION) {}

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

  float sample_pdf(
    const Intersection& itsc, const Ray& ray_i, const Ray& ray_o) const override {
    return 0.0f;
  }
};

/************************PerfectSpecular******************************/

class PerfectSpecular: public BXDF {
private:
  const Texture* absorb;

public:
  PerfectSpecular(const Texture* tex): 
    BXDF(BXDFNature::DELTA|BXDFNature::REFLECT), absorb(tex) {}

  inline glm::vec3 evaluate(
    const Intersection& itsc, const Ray& ray_o, const Ray& ray_i) const override {
    return glm::vec3(0.0f);
  }

  glm::vec3 sample_ev(
    const Intersection& itsc, const Ray& ray_o, Ray& ray_i) const override;
  
  float sample_pdf(
    const Intersection& itsc, const Ray& ray_i, const Ray& ray_o) const override {
    return 0.0f;
  }
};

/************************PerfectTransimission******************************/

class PerfectTransimission: public BXDF {
private:
  float IOR;
  const Texture* absorb;

public:
  PerfectTransimission(const Texture* tex, float IOR): 
    BXDF(BXDFNature::DELTA|BXDFNature::TRANSMISSION), IOR(IOR), absorb(tex) {}

  inline glm::vec3 evaluate(
    const Intersection& itsc, const Ray& ray_o, const Ray& ray_i) const override {
    return glm::vec3(0.0f);
  }

  glm::vec3 sample_ev(
    const Intersection& itsc, const Ray& ray_o, Ray& ray_i) const override;
  
  float sample_pdf(
    const Intersection& itsc, const Ray& ray_i, const Ray& ray_o) const override {
    return 0.0f;
  }
};

/************************GGX******************************/

class GGX: public BXDF {
private:
  float eataI, eataT;
  const Texture *roughness, *albedo;

  float roughnessToAlpha(float roughness) const ;
  float normalDistribution(float normalTangent2, float alpha) const ;
  void sampleNormal(float alpha, float& phi, float& tan2Theta) const ;
  float maskShadow(float tan2Theta, float alpha) const ;

public:
  GGX(float eataI, float eataT, const Texture* roughness, const Texture* albedo): 
    BXDF(BXDFNature::REFLECT | BXDFNature::GLOSSY), 
    eataI(eataI), eataT(eataT), roughness(roughness), albedo(albedo) {}

  inline glm::vec3 evaluate(
    const Intersection& itsc, const Ray& ray_o, const Ray& ray_i) const override;

  glm::vec3 sample_ev(
    const Intersection& itsc, const Ray& ray_o, Ray& ray_i) const override;
  
  float sample_pdf(
    const Intersection& itsc, const Ray& ray_i, const Ray& ray_o) const override;
  
  inline virtual bool needMIS(const Intersection& itsc) const {
    //if(roughness->tex2D(itsc.itscVtx.uv).x < 0.1f)
    return true;
  }
};

/************************HenyeyPhase******************************/

class HenyeyPhase : public BXDF {
private:
  float g;
  glm::vec3 sigmaS;

private:
  float samplePhaseCosTheta() const;

public:
  HenyeyPhase(float g, glm::vec3 sigmaS): BXDF(BXDFNature::NoSurface), 
    g(g), sigmaS(sigmaS) {}

  // return phase distribution value
  inline glm::vec3 evaluate(
    const Intersection& itsc, const Ray& ray_o, const Ray& ray_i) const override{
    // convert direction
    float cosTheta = -glm::dot(ray_o.d, ray_i.d);
    return glm::vec3(0.25f*INV_PI*(1-g*g)/ (1+g*g+2*g*cosTheta));
  }

  glm::vec3 sample_ev(
    const Intersection& itsc, const Ray& ray_o, Ray& ray_i) const override;

  float sample_pdf(
    const Intersection& itsc, const Ray& ray_i, const Ray& ray_o) const override;

  inline virtual bool needMIS(const Intersection& itsc) const {
    //if(g > 0.6f || g < -0.4f)
    return true;
  }
};