
#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"

#include "texture.hpp"


ImageTexture::ImageTexture(const char* imgname, 
  InterpolateMode mode): mode(mode), scale(1.0f), offset(0.0f) {
  img = stbi_load(imgname, &width, &height, &channel, 0);
  tot = width*height*channel;
}

glm::vec3 ImageTexture::getPixel(int x, int y) {
  int pos = (y*width+x)*channel;
  if(pos >= tot) pos = tot - channel;
  return 1.0f/255.0f*glm::vec3(img[pos], img[pos+1], img[pos+2]);
}

glm::vec3 ImageTexture::tex2D(glm::vec2 uv) {
  uv = uv*scale+offset;
  uv.x = uv.x - (int)uv.x;
  uv.y = uv.y - (int)uv.y;
  uv.x*=width; uv.y*=height;

  if(mode == InterpolateMode::BILINEAR) {
    int y = (int)uv.y;
    int x = (int)uv.x;
    float dx = uv.x - x;
    float dy = uv.y - y;
    glm::vec3 colx = (1.0f-dx)*getPixel(x, y)+dx*getPixel(x+1, y);
    glm::vec3 coly = (1.0f-dx)*getPixel(x, y+1)+dx*getPixel(x+1, y+1);
    return (1.0f-dy)*colx+dy*coly;
  }
  else if (mode == InterpolateMode::NEAR) {
    return glm::vec3(0.0f); //TODO;
  }
}