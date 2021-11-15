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
  model.toPrimitives(primitives);
}

void Scene::addLight(Light* light) {
  light->addToScene(*this);
  lights.push_back(light);
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

float Scene::sampleALight(Light*& light) {
  float pdf = 1.0f;
  int idx = ldistribution.sample(pdf);
  light = lights[idx];
  return pdf;
}

BB3 Scene::getWholeBound() const{
  return bvh.getWholeBound();
}

Intersection Scene::intersect(const Ray& ray, const Primitive* prim) {
  //__StartTimeAnalyse__("itsc_sub")
  Intersection itsc;
  bvh.intersect(ray, itsc, 0, prim);
  
  // for debug
  if(itsc.prim && itsc.t < 1e-4f) {
    std::cout<<"detect very near itsc"<<std::endl;
  }

  if(itsc.prim != nullptr) {
    itsc.prim->handleItscResult(itsc);
  }
  //__EndTimeAnalyse__
  return itsc;
}

bool Scene::intersectTest(const Ray& ray, const Primitive* prim) {
  return bvh.intersectTest(ray, 0, prim);
}

bool Scene::occlude(const Ray& ray, float t_limit, const Primitive* prim_avd) {
  Intersection itsc;
  itsc.t = t_limit;
  bvh.intersect(ray, itsc, 0, prim_avd);
  // for debug
  if(itsc.prim && itsc.t < 1e-4f) {
    std::cout<<"Detect very near itsc"<<std::endl;
  }
  return itsc.prim;
}

bool Scene::occlude(const Intersection& it1, const Intersection& it2, 
  Ray& testRay, float& rayLen) {
    return bvh.occlude(it1, it2, testRay, rayLen);
  }