#pragma once
#include "itsc.hpp"
#include "texture.hpp"

class Blender {
public:
  virtual float getBlendVal(const Intersection& itsc, const Ray& ray_o) const = 0;
};

class FixBlender: public Blender {
private:
  float blendCoe;
public:
  FixBlender(float blendCoe): blendCoe(blendCoe) {}
  float getBlendVal(const Intersection& itsc, const Ray& ray_o) const override {
    return blendCoe;
  }
};

class TexBlender: public Blender {
private:
  const Texture* tex;
public:
  TexBlender(const Texture* tex): tex(tex) {}
  float getBlendVal(const Intersection& itsc, const Ray& ray_o) const override {
    return tex->tex2D(itsc.itscVtx.uv).x;
  }
};

class DielectricFresnelBlender: public Blender {
private:
  float IOR; // IOR = eataT / eataI
private:
  inline float FrDielectric(
  float cosThetaI, float cosThetaT) const {
    float r1 = (IOR*cosThetaI - cosThetaT)/
      (IOR*cosThetaI + cosThetaT);
    float r2 = (cosThetaI - IOR*cosThetaT)/
      (cosThetaI + IOR*cosThetaT);
    return (r1*r1+r2*r2) / 2.0f;
  }
public:
  DielectricFresnelBlender(float IOR): IOR(IOR) {}
  // return reflection proportion
  float getBlendVal(const Intersection& itsc, const Ray& ray_o) const override {
    float idt = 1.0f / IOR;
    float cosThetaI = glm::dot(ray_o.d, itsc.itscVtx.normal);
    if(cosThetaI < 0) {
      idt = IOR;
      cosThetaI = -cosThetaI;
    }
    float sinThetaI = glm::sqrt(1 - glm::min(1.0f, cosThetaI*cosThetaI));
    float sinThetaT = idt*sinThetaI;
    if(sinThetaT>1.0f) return 1.0f; // total reflection
    else {
      float cosThetaT = glm::sqrt(1-glm::min(1.0f, sinThetaT*sinThetaT));
      return FrDielectric(cosThetaI, cosThetaT);
    }
  }
};