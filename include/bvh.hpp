#pragma once

#include <vector>

#include "primitive.hpp"
#include "itsc.hpp"

#include "debug/pcshow.hpp"

struct BVHNode {
  BB3 bb3;
  int offset, _size;
  int lft, rgt;
};

class BVH {
private:
  std::vector<const Primitive*> & prims; // from scene
  std::vector<BVHNode> bvhNodes;
  int maxDeep, maxPrimsInNode; // if maxDeep == -1, deep no restriction
public:
  BVH(std::vector<const Primitive*> & primitives, int maxDeep = 15, int maxPrimsInNode = 8): 
    prims(primitives), maxDeep(maxDeep), maxPrimsInNode(maxPrimsInNode){}
  int buildSAHBVH(int start, int end, int deep);

  void intersect(const Ray& ray, Intersection& itsc, 
    int nIdx, const Primitive* prim = nullptr) const;
  bool intersectTest(const Ray& ray, int nIdx, const Primitive* prim = nullptr) const;
  bool occlude(const Intersection& it1, const Intersection& it2, 
    Ray& testRay, float& rayLen) const;
  // For Debug, after build BVH
  void generatePointCloud(PCShower& pc);
  BB3 getWholeBound() const;

  inline const std::vector<const Primitive*> & getPrims() const {return prims;}
  inline const std::vector<BVHNode> & getNodes() const {return bvhNodes;}
};