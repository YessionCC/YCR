#include "bvh.hpp"

#include <algorithm>
#include <iostream>
#include <fstream>
#include <iomanip>

inline unsigned int leftShift3(unsigned int x) {
  x = (x|(x<<16)) & 0b00000011000000000000000011111111;
  x = (x|(x<<8)) &  0b00000011000000001111000000001111;
  x = (x|(x<<4)) &  0b00000011000011000011000011000011;
  x = (x|(x<<2)) &  0b00001001001001001001001001001001;
  return x;
}

unsigned int encode(glm::vec3 p) {
  unsigned int res = 0;
  res |= leftShift3((unsigned int)p.x);
  res |= leftShift3((unsigned int)p.y)<<1;
  res |= leftShift3((unsigned int)p.z)<<2;
  return res;
}

int BVH::buildSAHBVH(int start, int end, int deep) {
  if(end <= start) return -1;
  if(end - start == 1) {
    bvhNodes.push_back({prims[start]->getBB3(), start, 1, -1, -1});
    return bvhNodes.size() - 1;
  }

  BB3 totbb3; int nPrims = end - start;
  for(unsigned int i = start; i<end; i++) 
    totbb3.Union_(prims[i]->getBB3());
    
  if(maxDeep != -1 && deep >= maxDeep) {
    bvhNodes.push_back({totbb3, start, nPrims, -1, -1});
    return bvhNodes.size() - 1;
  }
    
  int axis = totbb3.getMaxAxis();
  std::sort(prims.begin()+start, prims.begin()+end, 
    [axis](Primitive* p1, Primitive* p2){
      return p1->getCenter()[axis]<p2->getCenter()[axis];});
  float invCalc = 1.0f/(totbb3.getSurfaceArea()*nPrims);

  std::vector<BB3> bb3s(end-start-1); BB3 tmp;
  for(unsigned int i = end-1; i>=start+1; i--) {
    tmp.Union_(prims[i]->getBB3());
    bb3s[i-start-1] = tmp;
  }
  
  BB3 lftbb3; int split = -1; float minSAH = FLOAT_MAX;
  for(unsigned int i = start; i<end-1; i++) {
    lftbb3.Union_(prims[i]->getBB3());
    BB3 rgtbb3 = bb3s[i-start];
    float sah = rgtbb3.getSurfaceArea()*invCalc*(end-i-1)+
      lftbb3.getSurfaceArea()*invCalc*(i-start+1) + .5f;
    if(minSAH>sah) {
      minSAH = sah;
      split = i;
    }
  }

  if(nPrims>maxPrimsInNode || minSAH<1) { // interior node
    bvhNodes.push_back({totbb3, start, nPrims, -1, -1}); 
    int nodePos = bvhNodes.size() - 1;
    int lft = buildSAHBVH(start, split+1, deep+1);
    int rgt = buildSAHBVH(split+1, end, deep+1);
    bvhNodes[nodePos].lft = lft;
    bvhNodes[nodePos].rgt = rgt;
    return nodePos;// to keep root nodepos 0 and make leftson be the next
  }
  else {
    bvhNodes.push_back({totbb3, start, nPrims, -1, -1}); // leaf node
    return bvhNodes.size() - 1;
  }
}

void BVH::intersect(const Ray& ray, Intersection& itsc, 
  int nIdx, const Primitive* prim) {

  BVHNode& curNode = bvhNodes[nIdx];
  float tMin, tMax;
  if(curNode.bb3.intersect(ray, tMin, tMax)) {
    if(itsc.t < tMin) return;
    if(curNode.lft == -1 && curNode.rgt == -1) { //leaf
      for(unsigned int i=curNode.offset; i<curNode.offset+curNode._size; i++) {
        if(prims[i] == prim) continue;
        prims[i]->intersect(ray, itsc);
      }
    }
    else {
      intersect(ray, itsc, curNode.lft, prim);
      intersect(ray, itsc, curNode.rgt, prim);
    }
  }
} 

bool BVH::intersectTest(const Ray& ray, int nIdx, const Primitive* prim) {
  BVHNode& curNode = bvhNodes[nIdx];
  float tMin, tMax;
  if(curNode.bb3.intersect(ray, tMin, tMax)) {
    if(curNode.lft == -1 && curNode.rgt == -1) { //leaf
      for(unsigned int i=curNode.offset; i<curNode.offset+curNode._size; i++) {
        if(prims[i] == prim) continue;
        if(prims[i]->intersectTest(ray)) return true;
      } 
    }
    else return intersectTest(ray, curNode.lft, prim) || 
                intersectTest(ray, curNode.rgt, prim);
  }
  return false;
}

void BVH::generatePointCloud(PCShower& pc) {
  int nPrim = prims.size();
  for(BVHNode& node: bvhNodes) {
    glm::vec3 cs[8];
    node.bb3.get8Cornor(cs);
    float col = powf(1.0f*node._size/nPrim, 0.4f);
    for(int i=0; i<8; i++)
      pc.addItem(cs[i], glm::vec3(col));
  }
}

// return false: no occlude
// return testRay(it1->it2) and its length
bool BVH::occlude(const Intersection& it1, const Intersection& it2, 
  Ray& testRay, float& rayLen) {

  testRay.o = it1.itscVtx.position;
  glm::vec3 dir = it2.itscVtx.position - it1.itscVtx.position;
  rayLen = glm::length(dir);
  testRay.d = glm::normalize(dir);
  Intersection itsc;
  itsc.t = rayLen;
  //intersect(testRay, itsc, 0, it1.prim); 
  intersect(testRay, itsc, 0, it2.prim); 

  // for debug
  if(itsc.prim && itsc.t < 1e-4f) {
    //std::cout<<"Detect very near itsc"<<std::endl;
  }

  if(itsc.prim) return true;
  //if(itsc.prim && itsc.prim != it2.prim) return true;
  return false;
}
