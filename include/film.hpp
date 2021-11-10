#pragma once 

#include <glm/glm.hpp>

#include <iostream>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

class Film {
private:
  int resolutionX, resolutionY, totPix;
  float fov;

  float sensorX, sensorY, sXhalf, sYhalf;

  float rasterPropX, rasterPropY;

  glm::vec3* pixels;
  float* pWeights;

public:
  //reX: image width, reY: image height, fov: degree
  Film(int reX, int reY, float fov): 
    resolutionX(reX), resolutionY(reY), fov(fov) {
    
    float t = 2.0f*glm::tan(glm::radians(.5f*fov));
    sensorX = t, sensorY = t*reY/reX;
    sXhalf = .5f*sensorX, sYhalf = .5f*sensorY;
    rasterPropX = sensorX/reX;
    rasterPropY = sensorY/reY;

    totPix = reX*reY;
    pixels = new glm::vec3[totPix];
    pWeights = new float[totPix];
  }

  Film() {}

  Film(const Film&) = delete;
  const Film& operator=(const Film&) = delete;

  ~Film() {delete pixels; delete pWeights;}

  // result z axis is default -1
  glm::vec2 raster2camera(glm::vec2 raster) const {
    return {raster.x*rasterPropX - sXhalf, -raster.y*rasterPropY + sYhalf};
  }

  glm::vec2 camera2raster(glm::vec3 camera) const {
    float cx = -camera.x / camera.z;
    float cy = -camera.y / camera.z;
    return {(cx+sXhalf)/rasterPropX, (sYhalf-cy)/rasterPropY};
  }

  //px: w, py: h
  void addRadiance(glm::vec3 L, float weight, int px, int py) {
    int pos = py*resolutionX+px;
    if(pos>totPix) {
      std::cout<<"Add Radiance out of range"<<std::endl;
      return;
    }
    pixels[pos] += weight*L;
    pWeights[pos] += weight;
  }

  void generateImage(const char* filename) {
    unsigned char* output = new unsigned char[totPix*3];
    for(int i=0; i<totPix; i++) {
      // debug error detect
      if(pWeights[i]<0.5f && pixels[i].x+pixels[i].y+pixels[i].z > 1e-6){
        std::cout<<"detect weight error"<<std::endl;
      }
      
      if(pWeights[i] > 0.0f) {
        pixels[i] /= pWeights[i];
      }
      pixels[i] = 255.0f*glm::clamp(pixels[i], 0.0f, 1.0f);
      output[i*3] = (unsigned char)(pixels[i].x);
      output[i*3+1] = (unsigned char)(pixels[i].y);
      output[i*3+2] = (unsigned char)(pixels[i].z);
    }
    stbi_write_jpg(filename, resolutionX, resolutionY, 3, output, 0);
    delete output;
  }

  void clear() {
    for(int i=0; i<totPix; i++) {
      pixels[i] = glm::vec3(0.0f);
      pWeights[i] = 0.0f;
    }
  }

  int getReX() const {return resolutionX;}
  int getReY() const {return resolutionY;}

};