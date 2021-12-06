#pragma once 
#include "itsc.hpp"
#include "sampler.hpp"
#include "blender.hpp"

// Tree recursion with inheritance and polymorphism
class BXDFNode {
public:
  // return weight, and also return the node
  virtual float getBXDF(
    const Intersection& itsc, const Ray& ray_o, const BXDFNode*& bxdfNode) const = 0;
};

class WeightedBXDF: public BXDFNode {
private:
  const BXDFNode* bxdfNode; 
  const Blender* blender;
  
public:
  WeightedBXDF(const BXDFNode* bxdfNode, const Blender* blender):
    bxdfNode(bxdfNode), blender(blender) {}
  
  inline float getBXDF(
    const Intersection& itsc, const Ray& ray_o, const BXDFNode*& bxdfNode) const override {
    return blender->getBlendVal(itsc, ray_o) *
      this->bxdfNode->getBXDF(itsc, ray_o, bxdfNode);
  }
};

class MixedBXDF: public BXDFNode {
private:
  const BXDFNode* bxdfNode1; 
  const BXDFNode* bxdfNode2; 
  const Blender* blender;
  
public:
  MixedBXDF(const BXDFNode* bxdfNode1, const BXDFNode* bxdfNode2, const Blender* blender):
    bxdfNode1(bxdfNode1), bxdfNode2(bxdfNode2), blender(blender) {}

  inline float getBXDF(
    const Intersection& itsc, const Ray& ray_o, const BXDFNode*& bxdfNode) const override {
    float blend = blender->getBlendVal(itsc, ray_o);
    return _ThreadSampler.get1() < blend ?
      bxdfNode1->getBXDF(itsc, ray_o, bxdfNode):
      bxdfNode2->getBXDF(itsc, ray_o, bxdfNode);
  }
};

// class Material {
// public:
//   const BXDFNode* bxdfNode = nullptr;
//   const Medium* mediumInside = nullptr;
//   const Medium* mediumOutside = nullptr;
//   const Light* light = nullptr; //if null, it is not emissive
//   const Texture* normalMap = nullptr;

// };