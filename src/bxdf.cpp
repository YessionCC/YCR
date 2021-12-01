#include "bxdf.hpp"
#include "sampler.hpp"

glm::vec3 LambertianDiffuse::evaluate(
  const Intersection& itsc, const Ray& ray_o, const Ray& ray_i) const {
  float cosTheta = glm::max(0.0f, itsc.itscVtx.cosTheta(ray_i.d));
  return cosTheta * INV_PI*texture->tex2D(itsc.itscVtx.uv);
}

glm::vec3 LambertianDiffuse::sample_ev(
  const Intersection& itsc, const Ray& ray_o, Ray& ray_i) const {
  glm::vec2 dir_i = _ThreadSampler.uniSampleDisk();
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

float GGX::roughnessToAlpha(float roughness) const {
  roughness = std::max(roughness, 1e-3f);
  float x = std::log(roughness);
  return 1.62142f + 0.819955f * x + 0.1734f * x * x + 0.0171201f * x * x * x +
          0.000640711f * x * x * x * x;
}
float GGX::normalDistribution(float normalTangent2, float alpha) const {
  float normalCos2 = 1.0f/(1.0f+normalTangent2);
  float tmp = alpha*normalCos2*(1+normalTangent2/(alpha*alpha));
  return 1.0f / (PI*tmp*tmp);
}
void GGX::sampleNormal(float alpha, float& phi, float& tan2Theta) const {
  phi = PI2*_ThreadSampler.get1();
  float u2 = _ThreadSampler.get1();
  tan2Theta = u2*alpha*alpha/(1-u2);
}
float GGX::maskShadow(float tan2Theta, float alpha) const {
  return 0.5f*(-1+glm::sqrt(1+(alpha*tan2Theta)*(alpha*tan2Theta)));
}

glm::vec3 GGX::evaluate(
  const Intersection& itsc, const Ray& ray_o, const Ray& ray_i) const {
  float alpha = roughnessToAlpha(roughness->tex2D(itsc.itscVtx.uv).x);
  float tan2ThetaI = glm::max(0.0f, itsc.itscVtx.tan2Theta(ray_i.d));
  float tan2ThetaO = glm::max(0.0f, itsc.itscVtx.tan2Theta(ray_o.d));
  float G = 1.0f/(1.0f+maskShadow(tan2ThetaI, alpha)+maskShadow(tan2ThetaO, alpha));
  glm::vec3 wh = glm::normalize(ray_i.d+itsc.itscVtx.normal);
  float tan2ThetaWh = glm::max(0.0f, itsc.itscVtx.tan2Theta(wh));
  float normalDistr = normalDistribution(tan2ThetaWh, alpha);
  float fr = FrDielectricReflect(ray_o.d, wh, eataI, eataT);
  float cosThetaO = itsc.itscVtx.cosTheta(ray_o.d);
  return normalDistr*G*fr/(4.0f*cosThetaO)*albedo->tex2D(itsc.itscVtx.uv);
}

glm::vec3 GGX::sample_ev(
  const Intersection& itsc, const Ray& ray_o, Ray& ray_i) const {
  float phi, tan2Theta;
  float alpha = roughnessToAlpha(roughness->tex2D(itsc.itscVtx.uv).x);
  sampleNormal(alpha, phi, tan2Theta);
  float cos2Theta = 1.0f/(1.0f+tan2Theta);
  float sinTheta = glm::sqrt(glm::max(0.0f, 1.0f - cos2Theta));
  glm::vec3 tanp(
    sinTheta*glm::cos(phi), 
    sinTheta*glm::sin(phi),
    glm::sqrt(cos2Theta));
  glm::vec3 wh = itsc.toWorldSpace(tanp);
  glm::vec3 refl = Reflect(ray_o.d, wh);
  // do not point inside surface, compare with geoNormal
  if(itsc.cosTheta(refl) <= 0.0f) return glm::vec3(0.0f);
  ray_i.o = itsc.itscVtx.position;
  ray_i.d = refl;
  float tan2ThetaI = glm::max(0.0f, itsc.itscVtx.tan2Theta(ray_i.d));
  float tan2ThetaO = glm::max(0.0f, itsc.itscVtx.tan2Theta(ray_o.d));
  float G = 1.0f/(1.0f+maskShadow(tan2ThetaI, alpha)+maskShadow(tan2ThetaO, alpha));
  float fr = FrDielectricReflect(ray_o.d, wh, eataI, eataT);
  float cosThetaO = itsc.itscVtx.cosTheta(ray_o.d);
  return G*fr/(4.0f*cosThetaO)*albedo->tex2D(itsc.itscVtx.uv); // D(wh) cancel out
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
  float u = _ThreadSampler.get1();
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
  float u = _ThreadSampler.get1();
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
  float phi = _ThreadSampler.get1()*PI2;
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