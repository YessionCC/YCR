#include "light.hpp"
#include "scene.hpp"
#include "sampler.hpp"

ShapeLight::ShapeLight(const Texture* ltMp, Model& shape): lightMap(ltMp), model(shape) {
    model.setLightForAllMeshes(this);
}

void ShapeLight::addToScene(Scene& scene) {
  model.toPrimitives(vp);
  totArea = 0;
  for(const Primitive* p: vp) {
    float area = p->getArea();
    dist.addPdf(area);
    totArea += area;
  }
  selectP = totArea*lightMap->getAverageLuminance();
  scene.addPrimitives(vp);
  dist.calcCdf();
}

float ShapeLight::getItscOnLight(Intersection& itsc, glm::vec3 evaP) const {
  float pdf = 1.0f;
  int idx = dist.sample(pdf);
  const Primitive* prim = vp[idx];
  pdf*=prim->getAPointOnSurface(itsc);
  return pdf;
}

void ShapeLight::genRay(Intersection& itsc, Ray& ray, float& pdf_A, float& pdf_D) const {
  pdf_D = 1.0f/PI2;
  pdf_A = getItscOnLight(itsc, glm::vec3(0.0f));
  glm::vec2 dir_i = _ThreadSampler.uniSampleHemiSphere();
  float sinTheta = glm::sqrt(1-glm::min(1.0f, dir_i.x*dir_i.x));
  glm::vec3 tanp(
    sinTheta*glm::cos(dir_i.y), 
    sinTheta*glm::sin(dir_i.y),
    dir_i.x);
  ray.o = itsc.itscVtx.position;
  ray.d = itsc.toWorldSpace(tanp);
  itsc.maxErrorOffset(ray.d, ray.o);
}

void PointLight::genRay(Intersection& itsc, Ray& ray, float& pdf_A, float& pdf_D) const {
  glm::vec3 dir;
  glm::vec2 uv = _ThreadSampler.uniSampleSphere();
  dir.y = uv.x;
  float sinTheta = glm::sqrt(1-glm::min(1.0f, uv.x*uv.x));
  dir.x = sinTheta*glm::cos(uv.y);
  dir.z = sinTheta*glm::sin(uv.y);
  ray.o = position;
  ray.d = dir;
  itsc.itscVtx.position = position;
  itsc.itscVtx.normal = dir;
  itsc.prim = nullptr;
  pdf_D = 1.0f/PI4;
  pdf_A = 1.0f;
}

float DirectionalLight::selectProbality(const Scene& scene) {
  // use the circle of the total scene as the area
  // maybe improve in the later
  worldBB3 = scene.getWholeBound();
  sceneDiameter = worldBB3.getDiagonalLength();
  return 0.25f*PI*sceneDiameter*sceneDiameter* Luminance(le);
}

EnvironmentLight::EnvironmentLight(const Texture* tex): environment(tex) {
  if(tex->getType() == Texture::TexType::Image) {
    isSolid = false;
    const ImageTexture* imt = dynamic_cast<const ImageTexture*>(environment);
    avgLuminance = imt->generateDistribution2D(dd2d);
  }
  else if(tex->getType() == Texture::TexType::Solid) {
    isSolid = true;
    avgLuminance = Luminance(tex->tex2D({0,0}));
  }
}

void EnvironmentLight::addToScene(Scene& scene) {
  if(scene.envLight) {
    std::cout<<"Warning: Set environment texture repeatly"<<std::endl;
  }
  scene.envLight = this;
}

float EnvironmentLight::selectProbality(const Scene& scene) {
  // use the circle of the total scene as the area
  // maybe improve in the later
  worldBB3 = scene.getWholeBound();
  sceneDiameter = worldBB3.getDiagonalLength();
  return PI4*sceneDiameter*sceneDiameter*avgLuminance;
}

glm::vec3 EnvironmentLight::evaluate(
  const Intersection& itsc, glm::vec3 dir) const {

  if(isSolid) return environment->tex2D({0,0});
  dir = -dir;
  float theta = glm::acos(glm::clamp(dir.y, -1.0f, 1.0f)); // world up
  float phi = std::atan2(dir.z, dir.x);
  if(phi < 0) phi += PI2;
  return environment->tex2D({phi*INV_PI2, theta*INV_PI});
}

float EnvironmentLight::getItscOnLight(Intersection& itsc, glm::vec3 evaP) const {
  glm::vec3 dir;
  itsc.prim = nullptr; // important
  if(isSolid) {
    glm::vec2 uv = _ThreadSampler.uniSampleSphere();
    dir.y = uv.x;
    float sinTheta = glm::sqrt(1-glm::min(1.0f, uv.x*uv.x));
    dir.x = sinTheta*glm::cos(uv.y);
    dir.z = sinTheta*glm::sin(uv.y);
    // sceneDiameter is light radius
    itsc.itscVtx.position = evaP + dir*sceneDiameter;
    itsc.itscVtx.normal = -dir;
    return 1.0f/(PI4*sceneDiameter*sceneDiameter);
  }
  else {
    float pdf; int x, y;
    // x->width->phi, y->height->theta
    dd2d.sample(pdf, x, y);
    glm::vec2 uv = _ThreadSampler.get2()+glm::vec2(x, y);
    float dux = 1.0f / dd2d.getCol()*PI2;
    float duy = 1.0f / dd2d.getRow()*PI;
    uv.x=uv.x*dux; uv.y=uv.y*duy;
    dir.y = glm::cos(uv.y);
    float sinTheta = glm::sin(uv.y);
    dir.x = sinTheta*glm::cos(uv.x);
    dir.z = sinTheta*glm::sin(uv.x);
    itsc.itscVtx.position = evaP + dir*sceneDiameter;
    itsc.itscVtx.normal = -dir;
    
    return pdf / (dux*duy*sceneDiameter*sceneDiameter*sinTheta);
  }
}

float EnvironmentLight::getItscPdf(const Intersection& itsc, const Ray& rayToLight) const{
  if(isSolid) return 1.0f/(PI4*sceneDiameter*sceneDiameter);
  float theta = glm::acos(glm::clamp(rayToLight.d.y, -1.0f, 1.0f)); // world up
  float phi = std::atan2(rayToLight.d.z, rayToLight.d.x);
  if(phi < 0) phi += PI2;
  int x = phi*INV_PI2*dd2d.getCol();
  int y = theta*INV_PI*dd2d.getRow();
  if(x >= dd2d.getCol()) x -= 1;
  if(y >= dd2d.getCol()) y -= 1;
  float dux = 1.0f / dd2d.getCol()*PI2;
  float duy = 1.0f / dd2d.getRow()*PI;
  return dd2d.getPdf(x, y) / (dux*duy*sceneDiameter*sceneDiameter*glm::sin(theta));
}

void EnvironmentLight::genRayItsc(
  Intersection& itsc, const Ray& rayToLight, glm::vec3 evaP) const {
  itsc.itscVtx.position = evaP + rayToLight.d*sceneDiameter;
  itsc.itscVtx.normal = -rayToLight.d;
  itsc.prim = nullptr;
}