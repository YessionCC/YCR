#pragma once

#include <vector>

#include "primitive.hpp"
#include "model.hpp"
#include "bb3.hpp"
#include "bvh.hpp"
#include "distribution.hpp"

#include "debug/pcshow.hpp"

class Scene {
public:
  EnvironmentLight* envLight;
private:
  std::vector<Primitive*> primitives;
  std::vector<Light*> lights;
  DiscreteDistribution1D ldistribution; // light distribution
  
  BB3 sceneBB3;
  BVH bvh;

  void buildBVH();
  void calcLightDistribution();

public:
  Scene(): envLight(nullptr), bvh(primitives) {}
  ~Scene();

  void addModel(Model& model);
  void addLight(Light* light);
  void addPrimitive(Primitive* prim);
  void addPrimitives(std::vector<Primitive*> prims);

  void init();

  // prim used to avoid intersect self when the scene do not have curve surface
  Intersection intersect(const Ray& ray, const Primitive* prim = nullptr);
  bool intersectTest(const Ray& ray, const Primitive* prim = nullptr);
  bool occlude(const Ray& ray, float t_limit, 
    const Primitive* prim_avd = nullptr);
  bool occlude(const Intersection& it1, const Intersection& it2, 
  Ray& testRay, float& rayLen);
  BVH& getBVH() {return bvh;}
  BB3 getWholeBound() const;

  // return pdf
  float sampleALight(Light*& light);

  // For Debug
  void saveBVHHierachyAsPointCloud(PCShower& pc);
};