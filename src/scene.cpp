#include "scene.hpp"
#include "debug/analyse.hpp"
#include <iostream>

Scene::~Scene() {
  for(Primitive* p: primitives)
    delete p;
}

void Scene::init() {
  buildBVH();
  if(lights.size()>0) calcLightDistribution();
  else std::cout<<"Warning: No Lights!"<<std::endl;
}

void Scene::addModel(Model& model) {
  model.setMediumForAllMeshes(globalMedium, false);
  model.toPrimitives(primitives);
}

void Scene::addLight(Light* light) {
  light->addToScene(*this);
  lights.push_back(light);
}

void Scene::addMedium(Medium* medium, bool noBXDF) {
  medium->addToScene(*this, noBXDF);
}

// global medium must add before all model
void Scene::addGlobalMedium(Medium* medium) {
  this->globalMedium = medium;
}

void Scene::addPrimitive(Primitive* prim) {
  primitives.push_back(prim);
}
void Scene::addPrimitives(std::vector<Primitive*> prims) {
  primitives.insert(primitives.end(), prims.begin(), prims.end());
}

void Scene::buildBVH() {
  bvh.buildSAHBVH(0, primitives.size(), 0);
}

void Scene::calcLightDistribution() {
  for(Light* light: lights) {
    ldistribution.addPdf(light->selectProbality(*this));
  }
  ldistribution.calcCdf();
}

void Scene::saveBVHHierachyAsPointCloud(PCShower& pc) {
  bvh.generatePointCloud(pc);
}

float Scene::sampleALight(Light*& light) const{
  if(lights.size() == 1) {light = lights[0]; return 1.0f;}
  float pdf = 1.0f;
  int idx = ldistribution.sample(pdf);
  light = lights[idx];
  return pdf;
}

// estimate light distribution dynamically, using le*cosTheta/r2/pdf
// when there are a lot of light, not recommend
float Scene::dynamicSampleALight(Light*& light, glm::vec3 evap) const{
  if(lights.size() == 1) {light = lights[0]; return 1.0f;}
  Intersection litsc; glm::vec3 dir, L;
  DiscreteDistribution1D dd1d(lights.size());
  for(unsigned int i = 0; i<lights.size(); i++) {
    float pdf = lights[i]->getItscOnLight(litsc, evap);
    dir = evap - litsc.itscVtx.position;
    float dis2 = dir.x*dir.x+dir.y*dir.y+dir.z*dir.z;
    L = lights[i]->evaluate(litsc, dir/glm::sqrt(dis2));
    dd1d.addPdf(Luminance(L)/(dis2*pdf), i);
  }
  dd1d.calcCdf();
  float spdf = 1.0f;
  int idx = dd1d.sample(spdf);
  light = lights[idx];
  return spdf;
}

BB3 Scene::getWholeBound() const{
  return bvh.getWholeBound();
}

Intersection Scene::intersect(const Ray& ray, const Primitive* prim) {
  //__StartTimeAnalyse__("itsc_sub")
  Intersection itsc;
  bvh.intersect(ray, itsc, 0, prim);
  if(itsc.prim != nullptr) {
    itsc.prim->handleItscResult(itsc);
  }
  //__EndTimeAnalyse__
  return itsc;
}

bool Scene::intersectTest(const Ray& ray, const Primitive* prim) {
  return bvh.intersectTest(ray, 0, prim);
}

// no volume direct test
bool Scene::occlude(const Ray& ray, float t_limit, const Primitive* prim_avd) {
  Intersection itsc;
  itsc.t = t_limit;
  bvh.intersect(ray, itsc, 0, prim_avd);
  return itsc.prim;
}

// For volume direct light test
// medium: the medium the ray.o in
bool Scene::occlude(const Ray& ray, float t_limit, glm::vec3& tr,
  const Medium* medium, const Primitive* prim_avd) {

  tr = glm::vec3(1.0f);
  Intersection itsc;
  Ray testRay = ray;
  for(int bounce = 0; bounce < 24; bounce++) { // limit test times
    itsc.t = t_limit;
    itsc = intersect(testRay, nullptr);
    if(medium) tr*=medium->tr(itsc.t);
    // if no itsc, it can only be caused by numerical error
    // when this case, the light have itsc in fact
    if(!itsc.prim || itsc.prim == prim_avd) return false;
    Mesh* mesh = itsc.prim->getMesh();
    if(_HasType(mesh->getType(), MEDIUM)) {
      t_limit -= itsc.t;
      testRay.o = itsc.itscVtx.position;
      itsc.maxErrorOffset(testRay.d, testRay.o);
      if(itsc.cosTheta(testRay.d) < 0) medium = mesh->mediumInside;
      else medium = mesh->mediumOutside;
    }
    else return true;
  }
  std::cout<<"Scene::Occlude: Lots of test, "
    "may caused bu complex model or numerical error"<<std::endl;
  return true;
}

bool Scene::occlude(const Intersection& it1, const Intersection& it2, 
  Ray& testRay, float& rayLen) {
    return bvh.occlude(it1, it2, testRay, rayLen);
}