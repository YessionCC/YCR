#include "bxdf.hpp"
#include "sampler.hpp"

glm::vec3 LambertianReflection::evaluate(
  const Intersection& itsc, const Ray& ray_o, const Ray& ray_i) const {
  float cosTheta = glm::max(0.0f, itsc.itscVtx.cosTheta(ray_i.d));
  return cosTheta * INV_PI*texture->tex2D(itsc.itscVtx.uv);
}

glm::vec3 LambertianReflection::sample_ev(
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

float LambertianReflection::sample_pdf(
  const Intersection& itsc, const Ray& ray_i, const Ray& ray_o) const {
  return INV_PI*glm::max(0.0f, itsc.itscVtx.cosTheta(ray_i.d));
}

glm::vec3 PerfectSpecular::sample_ev(
  const Intersection& itsc, const Ray& ray_o, Ray& ray_i) const {
  ray_i.o = itsc.itscVtx.position;
  ray_i.d = Reflect(ray_o.d, itsc.itscVtx.normal);
  return absorb->tex2D(itsc.itscVtx.uv);
}

glm::vec3 PerfectTransimission::sample_ev(
  const Intersection& itsc, const Ray& ray_o, Ray& ray_i) const {
  ray_i.o = itsc.itscVtx.position;
  float cIOR = itsc.normalReverse?1.0f/IOR:IOR;
  bool res = Refract(ray_o.d, itsc.itscVtx.normal, cIOR, ray_i.d);
  if(!res) return glm::vec3(0.0f);

  return (cIOR*cIOR) * absorb->tex2D(itsc.itscVtx.uv);
}

/************************GGX Utility********************************/
float roughnessToAlpha(float roughness) {
  roughness = std::max(roughness, 1e-3f);
  float x = std::log(roughness);
  return 1.62142f + 0.819955f * x + 0.1734f * x * x + 0.0171201f * x * x * x +
          0.000640711f * x * x * x * x;
}
float normalDistribution(float normalTangent2, float alpha) {
  float normalCos2 = 1.0f/(1.0f+normalTangent2);
  float tmp = alpha*normalCos2*(1+normalTangent2/(alpha*alpha));
  return 1.0f / (PI*tmp*tmp);
}
void sampleNormal(float alpha, float& phi, float& tan2Theta) {
  phi = PI2*_ThreadSampler.get1();
  float u2 = _ThreadSampler.get1();
  tan2Theta = u2*alpha*alpha/(1-u2);
}
float maskShadow(float tan2Theta, float alpha) {
  return 0.5f*(-1+glm::sqrt(1+alpha*alpha*tan2Theta));
}
/********************************************************/

glm::vec3 GGXReflection::evaluate(
  const Intersection& itsc, const Ray& ray_o, const Ray& ray_i) const {
  float alpha = roughnessToAlpha(roughness->tex2D(itsc.itscVtx.uv).x);
  float tan2ThetaI = glm::max(0.0f, itsc.itscVtx.tan2Theta(ray_i.d));
  float tan2ThetaO = glm::max(0.0f, itsc.itscVtx.tan2Theta(ray_o.d));
  float G = 1.0f/(1.0f+maskShadow(tan2ThetaI, alpha)+maskShadow(tan2ThetaO, alpha));
  glm::vec3 wh = glm::normalize(ray_i.d+ray_o.d);
  float tan2ThetaWh = glm::max(0.0f, itsc.itscVtx.tan2Theta(wh));
  float normalDistr = normalDistribution(tan2ThetaWh, alpha);
  float cosThetaO = itsc.itscVtx.cosTheta(ray_o.d);
  if(cosThetaO <= 0.0f) return glm::vec3(0.0f);
  return normalDistr*G/(4.0f*cosThetaO)*albedo->tex2D(itsc.itscVtx.uv);
}

// !!we sample D(wh), and must transform it to wi(pdf from half angle to solid angle)
// pdf(wh) = D(wh)*cos(wh)
// pdf(wo) = pdf(wh) / (4cos(wh, wi))
// brdf*cos(wi)/pdf = D(wh)*G*Fr/(4cos(i,n)*cos(o,n)) * cos(i,n) /(D(wh)cos(wh,n)/4cos(wh, wi))
// == G*Fr*cos(wh,i)/(cos(wh,n)*cos(o,n))
glm::vec3 GGXReflection::sample_ev(
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
  ray_i.o = itsc.itscVtx.position;
  ray_i.d = refl;
  // do not point inside surface, compare with geoNormal
  if(itsc.cosTheta(refl) <= 0.0f) return glm::vec3(0.0f);
  float tan2ThetaI = glm::max(0.0f, itsc.itscVtx.tan2Theta(ray_i.d));
  float tan2ThetaO = glm::max(0.0f, itsc.itscVtx.tan2Theta(ray_o.d));
  float G = 1.0f/(1.0f+maskShadow(tan2ThetaI, alpha)+maskShadow(tan2ThetaO, alpha));
  float cosThetaO = itsc.itscVtx.cosTheta(ray_o.d);
  float cosWh = itsc.itscVtx.cosTheta(wh);
  float cosWhI = glm::max(0.0f, glm::dot(ray_i.d, wh));
  if(cosThetaO <= 0.0f) return glm::vec3(0.0f);
  return G*cosWhI/(cosThetaO*cosWh)*albedo->tex2D(itsc.itscVtx.uv); 
}

// pdf(wh) = D(wh)*cos(wh)
// pdf(wo) = pdf(wh) / (4cos(wh, wi))
float GGXReflection::sample_pdf(
  const Intersection& itsc, const Ray& ray_i, const Ray& ray_o) const {
  float alpha = roughnessToAlpha(roughness->tex2D(itsc.itscVtx.uv).x);
  glm::vec3 wh = glm::normalize(ray_i.d+ray_o.d);
  float tan2ThetaWh = glm::max(0.0f, itsc.itscVtx.tan2Theta(wh));
  float normalDistr = normalDistribution(tan2ThetaWh, alpha);
  float cosWh = itsc.itscVtx.cosTheta(wh);
  float cosWhI = glm::max(0.0f, glm::dot(ray_i.d, wh));
  return normalDistr*cosWh/(4.0f*cosWhI);
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

float HenyeyPhase::sample_pdf(
  const Intersection& itsc, const Ray& ray_i, const Ray& ray_o) const {
  return evaluate(itsc, ray_i, ray_o).x;
}
