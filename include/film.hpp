#pragma once 

#include <glm/glm.hpp>

#include <iostream>

#include <atomic>

#include "filter.hpp"

class Film {
private:
  int resolutionX, resolutionY, totPix;
  float fov;

  float sensorX, sensorY, sXhalf, sYhalf;

  float rasterPropX, rasterPropY;

  glm::vec3* pixels;
  float* pWeights;

  Filter* filter;
  float fradius;

  bool toneMap;
  float exposure;

  std::atomic<int>* mutexMat;

public:
  Film() {}
  //reX: image width, reY: image height, fov: degree
  Film(int reX, int reY, float fov, 
    bool toneMap = false, float exposure = 1.0f, 
    Filter* filter = new BoxFilter(0.5f));

  Film(const Film&) = delete;
  const Film& operator=(const Film&) = delete;

  ~Film() {delete pixels; delete pWeights; delete filter;}

  // result z axis is default -1
  inline glm::vec2 raster2camera(glm::vec2 raster) const {
    return {raster.x*rasterPropX - sXhalf, -raster.y*rasterPropY + sYhalf};
  }

  inline glm::vec2 camera2raster(glm::vec3 camera) const {
    // in the back of the film, always return invalid position
    if(camera.z > 0.0f) return {-1,-1};
    float cx = -camera.x / camera.z;
    float cy = -camera.y / camera.z;
    glm::vec2 res((cx+sXhalf)/rasterPropX, (sYhalf-cy)/rasterPropY);
    return res;
  }

  inline bool isValidRasPos(glm::vec2 rasPos) const {
    return rasPos.x >=0 && rasPos.x < resolutionX &&
      rasPos.y >=0 && rasPos.y <resolutionY;
  }

  // when in sumMode, the radiance to add will always increase luminance,
  // not average the luminance (i.e. do not add weight)
  void addSplat(glm::vec3 L, glm::vec2 center, bool sumMode=false);
  //px: w, py: h
  void addRadiance(glm::vec3 L, float weight, int px, int py, bool sumMode=false);

  void generateImage(const char* filename) const ;
  void generateImage(unsigned char* imgMat) const ;

  inline void clear() {
    for(int i=0; i<totPix; i++) {
      pixels[i] = glm::vec3(0.0f);
      pWeights[i] = 0.0f;
    }
  }

  inline int getReX() const {return resolutionX;}
  inline int getReY() const {return resolutionY;}

};