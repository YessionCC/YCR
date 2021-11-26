#include "film.hpp"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

Film::Film(int reX, int reY, float fov, Filter* filter): 
  resolutionX(reX), resolutionY(reY), 
  fov(fov), filter(filter), fradius(filter->getRadius()-0.5f) {
  
  float t = 2.0f*glm::tan(glm::radians(.5f*fov));
  sensorX = t, sensorY = t*reY/reX;
  sXhalf = .5f*sensorX, sYhalf = .5f*sensorY;
  rasterPropX = sensorX/reX;
  rasterPropY = sensorY/reY;

  totPix = reX*reY;
  pixels = new glm::vec3[totPix];
  pWeights = new float[totPix];
  if(fradius > 0.5f)
    mutexMat = new std::atomic<int>[totPix];
  else mutexMat = nullptr;
}

void Film::addRadiance(glm::vec3 L, float weight, int px, int py) {
  int pos = py*resolutionX+px;
  
  if(mutexMat) {
    int expect = 0;
    while(!mutexMat[pos].compare_exchange_weak(expect, 1));
  }
  
  if(pos>totPix) {
    std::cout<<"Add Radiance out of range"<<std::endl;
    return;
  }
  pixels[pos] += weight*L;
  pWeights[pos] += weight;

  if(mutexMat) mutexMat[pos] = 0;
}

void Film::addSplat(glm::vec3 L, glm::vec2 center) {
  if(fradius == 0.0f) {
    int x = (int)center.x, y = (int)center.y;
    addRadiance(L, filter->evaluate(center-glm::vec2(x+0.5f, y+0.5f)), x, y);
  }
  int bxl = glm::max((int)(center.x - fradius), 0);
  int bxr = glm::min((int)(center.x + fradius), resolutionX-1);
  int byl = glm::max((int)(center.y - fradius), 0);
  int byr = glm::min((int)(center.y + fradius), resolutionY-1);
  for(int i = bxl; i <= bxr; i++) {
    for(int j = byl; j <= byr; j++) {
      addRadiance(L, filter->evaluate(center-glm::vec2(i+0.5f, j+0.5f)), i, j);
    }
  }
}

void Film::generateImage(const char* filename) {
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
  stbi_write_jpg(filename, resolutionX, resolutionY, 3, output, 100);
  delete output;
}
