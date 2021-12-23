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

  std::atomic<int>* mutexMat;

public:
  Film() {}
  //reX: image width, reY: image height, fov: degree
  Film(int reX, int reY, float fov, 
    Filter* filter = new BoxFilter(0.5f), bool toneMap = false);

  Film(const Film&) = delete;
  const Film& operator=(const Film&) = delete;

  ~Film() {delete pixels; delete pWeights; delete filter;}

  // result z axis is default -1
  inline glm::vec2 raster2camera(glm::vec2 raster) const {
    return {raster.x*rasterPropX - sXhalf, -raster.y*rasterPropY + sYhalf};
  }

  inline glm::vec2 camera2raster(glm::vec3 camera) const {
    float cx = -camera.x / camera.z;
    float cy = -camera.y / camera.z;
    return {(cx+sXhalf)/rasterPropX, (sYhalf-cy)/rasterPropY};
  }

  inline bool isValidRasPos(glm::vec2 rasPos) const {
    return rasPos.x >=0 && rasPos.x < resolutionX &&
      rasPos.y >=0 && rasPos.y <resolutionY;
  }

  void addSplat(glm::vec3 L, glm::vec2 center);
  //px: w, py: h
  void addRadiance(glm::vec3 L, float weight, int px, int py);

  void generateImage(const char* filename, float exposure = 1.0f) const ;
  void generateImage(unsigned char* imgMat, float exposure = 1.0f) const ;

  inline void clear() {
    for(int i=0; i<totPix; i++) {
      pixels[i] = glm::vec3(0.0f);
      pWeights[i] = 0.0f;
    }
  }

  inline int getReX() const {return resolutionX;}
  inline int getReY() const {return resolutionY;}

};