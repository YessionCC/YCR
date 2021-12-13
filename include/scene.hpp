#pragma once

#include <vector>
#include <map>

#include "primitive.hpp"
#include "medium.hpp"
#include "model.hpp"
#include "bb3.hpp"
#include "bvh.hpp"
#include "camera.hpp"
#include "distribution.hpp"

#include "debug/pcshow.hpp"

class Scene {
public:
  const EnvironmentLight* envLight = nullptr;
private:
  std::vector<const Primitive*> primitives;
  std::vector<Light*> lights;
  std::map<const Light*, int> ltIdx;
  DiscreteDistribution1D ldistribution; // light distribution
  
  const Medium* globalMedium = nullptr;
  bool hasMedium = false;

  BB3 sceneBB3;
  BVH bvh;

  void buildBVH();
  void calcLightDistribution();

public:
  Scene(): bvh(primitives) {}
  ~Scene();

  void addModel(Model& model);
  void addLight(Light* light);
  void addMedium(Medium* medium, bool noBXDF = true);
  void addGlobalMedium(const Medium* medium);
  void addPrimitive(const Primitive* prim);
  void addPrimitives(std::vector<const Primitive*> prims);

  void init();

  // prim used to avoid intersect self when the scene do not have curve surface
  Intersection intersect(const Ray& ray,
    const Primitive* prim = nullptr, float t_limit = FLOAT_MAX) const;
  Intersection intersectDirectly(
    const Ray& ray, const Medium* medium, glm::vec3& tr) const ;

  bool intersectTest(const Ray& ray, const Primitive* prim = nullptr) const;

  bool occlude(const Ray& ray, float t_limit, glm::vec3& tr,
    const Medium* medium = nullptr, const Primitive* prim_avd = nullptr) const;
  bool occlude(const Ray& ray, float t_limit, 
    const Primitive* prim_avd = nullptr) const;
  bool occlude(const Intersection& it1, const Intersection& it2, 
    Ray& testRay, float& rayLen) const;

  BB3 getWholeBound() const;
  inline const BVH& getBVH() const {return bvh;}
  inline const Medium* getGlobalMedium() const {return globalMedium;}
  inline bool hasMediumInScene() const {return hasMedium;}

  // return pdf
  float sampleALight(const Light*& light) const;
  float dynamicSampleALight(const DiscreteDistribution1D& ldd1d, const Light*& light) const;
  float dynamicSampleALight(const Light*& light, glm::vec3 evap) const;
  void getPositionLightDD1D(glm::vec3 evap, DiscreteDistribution1D& dd1d) const;
  float getLightPdf(const DiscreteDistribution1D& ldd1d, const Light* lt) const;

  // For Debug
  void saveBVHHierachyAsPointCloud(PCShower& pc);
};