#include "bxdf.hpp"

glm::vec3 LambertianDiffuse::evaluate(
  const Intersection& itsc, const Ray& ray_o, const Ray& ray_i) const {
  float cosTheta = glm::max(0.0f, itsc.itscVtx.cosTheta(ray_i.d));
  return cosTheta * INV_PI*texture->tex2D(itsc.itscVtx.uv);
}

glm::vec3 LambertianDiffuse::sample_ev(
  const Intersection& itsc, const Ray& ray_o, Ray& ray_i) const {
  glm::vec2 dir_i = SampleShape::sampler().uniSampleDisk();
  glm::vec3 tanp(
    dir_i.x*glm::cos(dir_i.y), 
    dir_i.x*glm::sin(dir_i.y),
    glm::sqrt(1-glm::min(1.0f, dir_i.x*dir_i.x)));
  ray_i.o = itsc.itscVtx.position;
  ray_i.d = itsc.toWorldSpace(tanp);
  return texture->tex2D(itsc.itscVtx.uv);
}

glm::vec3 NoFrSpecular::sample_ev(
  const Intersection& itsc, const Ray& ray_o, Ray& ray_i) const {
  ray_i.o = itsc.itscVtx.position;
  ray_i.d = Reflect(ray_o.d, itsc.itscVtx.normal);
  return absorb->tex2D(itsc.itscVtx.uv);
}

glm::vec3 GlassSpecular::sample_ev(
  const Intersection& itsc, const Ray& ray_o, Ray& ray_i) const {
  ray_i.o = itsc.itscVtx.position;
  float cosThetaI = itsc.itscVtx.cosTheta(ray_o.d);
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

inline float HenyeyPhase::samplePhaseCosTheta() const { //
  float cosTheta;
  float u = SampleShape::sampler().get1();
  if(glm::abs(g) < 1e-3) cosTheta = 1.f-2.f*u;
  else {
    float t = (1-g*g)/(1-g+2*g*u);
    cosTheta = (1+g*g-t*t)/(2*g);
  }
  return cosTheta;
}

glm::vec3 HenyeyPhase::sample_ev(
  const Intersection& itsc, const Ray& ray_o, Ray& ray_i) const {
  float cosTheta = samplePhaseCosTheta();
  float sinTheta = glm::sqrt(1.0f - glm::min(1.0f, cosTheta*cosTheta));
  float phi = SampleShape::sampler().get1()*PI2;
  glm::vec3 rayP = {
    glm::cos(phi)*sinTheta, 
    glm::sin(phi)*sinTheta,
    cosTheta
  };
  glm::vec3 x, y;
  ray_o.getXYAxis(x, y);
  ray_i.o = ray_o.o;
  ray_i.d = rayP.x*x+rayP.y*y+rayP.z*ray_o.d;
  return sigmaS;
}