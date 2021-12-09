#pragma once 

#include "sampler.hpp"
#include "blender.hpp"

// Tree recursion with inheritance and polymorphism
class BXDFNode {
public:
  virtual ~BXDFNode() {}
  // return weight, and also return the node
  virtual float getBXDF(
    const Intersection& itsc, const Ray& ray_o, const BXDFNode*& bxdfNode) const = 0;
};

class WeightedBXDF: public BXDFNode {
private:
  const BXDFNode* bxdfNode; 
  const Blender* blender; // blender do not share
  
public:
  virtual ~WeightedBXDF() {delete blender;}
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
  virtual ~MixedBXDF() {delete blender;}
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

class Medium;
class Light;
class Texture;
class BXDF;

class Material {
public:
  const BXDFNode* bxdfNode = nullptr;
  const Medium* mediumInside = nullptr;
  const Medium* mediumOutside = nullptr;
  const Light* light = nullptr; //if null, it is not emissive
  const Texture* normalMap = nullptr;

public:
  float getBXDF(
    const Intersection& itsc, const Ray& ray_o, const BXDF*& bxdfNode) const;

  void bumpMapping(Intersection& itsc) const;

};