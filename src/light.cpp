#include "light.hpp"
#include "scene.hpp"

ShapeLight::ShapeLight(glm::vec3 le, Model& shape): le(le), model(shape) {
    model.setLightForAllMeshes(this);
  }

void ShapeLight::addToScene(Scene& scene) {
  model.toPrimitives(vp);
  float totArea = 0;
  for(Primitive* p: vp) {
    float area = p->getArea();
    dist.addPdf(area);
    totArea += area;
  }
  selectP = totArea*Luminance(le);
  scene.addPrimitives(vp);
  dist.calcCdf();
}

float ShapeLight::selectProbality(const Scene& scene) {return selectP;}

// calc Le*cosTheta, dir in world space, dir point to outside surface
glm::vec3 ShapeLight::evaluate(const Intersection& itsc, glm::vec3 dir) const {
  return le*glm::max(0.0f, itsc.itscVtx.cosTheta(dir));
}

float ShapeLight::getItscOnLight(Intersection& itsc, glm::vec3 evaP) const {
  float pdf = 1.0f;
  int idx = dist.sample(pdf);
  Primitive* prim = vp[idx];
  pdf*=prim->getAPointOnSurface(itsc);
  return pdf;
}

float DirectionalLight::selectProbality(const Scene& scene) {
  // use the circle of the total scene as the area
  // maybe improve in the later
  BB3 totBB3 = scene.getWholeBound();
  sceneDiameter = totBB3.getDiagonalLength();
  return Luminance(le);
}

EnvironmentLight::EnvironmentLight(Texture* tex, float scale): 
  environment(tex), lumiScale(scale) {
  if(tex->getType() == Texture::TexType::Image) {
    isSolid = false;
    ImageTexture* imt = dynamic_cast<ImageTexture*>(environment);
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
  BB3 totBB3 = scene.getWholeBound();
  sceneDiameter = totBB3.getDiagonalLength();
  return PI4*avgLuminance*lumiScale;
}

glm::vec3 EnvironmentLight::evaluate(
  const Intersection& itsc, glm::vec3 dir) const {

  if(isSolid) return environment->tex2D({0,0});
  dir = -dir;
  float theta = glm::acos(dir.y); // world up
  float phi = std::atan2(dir.z, dir.x);
  if(phi < 0) phi += PI2;
  return lumiScale * environment->tex2D({phi*INV_PI2, theta*INV_PI});
}

float EnvironmentLight::getItscOnLight(Intersection& itsc, glm::vec3 evaP) const {
  glm::vec3 dir;
  itsc.prim = nullptr; // important
  if(isSolid) {
    glm::vec2 uv = SampleShape::sampler().uniSampleSphere();
    dir.y = uv.x;
    float sinTheta = glm::sqrt(1-glm::min(1.0f, uv.x*uv.x));
    dir.x = sinTheta*glm::cos(uv.y);
    dir.z = sinTheta*glm::sin(uv.y);
    itsc.itscVtx.position = evaP + dir*sceneDiameter;
    return 1.0f/(PI*sceneDiameter*sceneDiameter);
  }
  else {
    float pdf; int x, y;
    // x->width->phi, y->height->theta
    dd2d.sample(pdf, x, y);
    glm::vec2 uv = SampleShape::sampler().get2()+glm::vec2(x, y);
    float dux = 1.0f / dd2d.getCol()*PI2;
    float duy = 1.0f / dd2d.getRow()*PI;
    uv.x=uv.x*dux; uv.y=uv.y*duy;
    dir.y = glm::cos(uv.y);
    float sinTheta = glm::sin(uv.y);
    dir.x = sinTheta*glm::cos(uv.x);
    dir.z = sinTheta*glm::sin(uv.x);
    itsc.itscVtx.position = evaP + dir*sceneDiameter;
    return pdf / (dux*duy*sceneDiameter*sceneDiameter);
  }
}