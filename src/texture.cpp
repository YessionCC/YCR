
#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"

#define STB_IMAGE_RESIZE_IMPLEMENTATION
#include "stb/stb_image_resize.h"

#include "texture.hpp"


ImageTexture::ImageTexture(const char* imgname, 
  float pixelScale, InterpolateMode mode):
  Texture(TexType::Image), mode(mode), scale(1.0f), offset(0.0f), pixelScale(pixelScale) {
  img = nullptr;
  img = stbi_load(imgname, &width, &height, &channel, 0);
  if(img) {
    std::cout<<"Image: "<<imgname<<" load successfully, "<<
      width<<"*"<<height<<"*"<<channel<<std::endl;
  }
  else {
    std::cout<<"ERROR: Image load failed!"<<std::endl;
  }
  tot = width*height*channel;
}

glm::vec3 ImageTexture::getPixel(int x, int y) const{
  int pos = (y*width+x)*channel;
  if(pos >= tot) {
    pos = tot - channel;
    std::cout<<"Warning: GetPixel out of range"<<std::endl;
  }
  if(channel == 1) return 1.0f/255.0f*glm::vec3(img[pos]);
  return 1.0f/255.0f*glm::vec3(img[pos], img[pos+1], img[pos+2]);
}

glm::vec3 ImageTexture::tex2D(glm::vec2 uv) const{
  uv = uv*scale+offset;
  uv.x = uv.x - (int)uv.x;
  uv.y = uv.y - (int)uv.y;
  uv.x*=width; uv.y*=height;
  glm::vec3 col(0.0f);
  if(mode == InterpolateMode::BILINEAR) {
    int xl = (int)(std::floor(uv.x - 0.5f));
    int yl = (int)(std::floor(uv.y - 0.5f));
    float dx = uv.x - xl - 0.5f;
    float dy = uv.y - yl - 0.5f;
    int xr = glm::min(xl+1, width - 1); xl = glm::max(xl, 0);
    int yr = glm::min(yl+1, height - 1); yl = glm::max(yl, 0);
    glm::vec3 colx = (1.0f-dx)*getPixel(xl, yl)+dx*getPixel(xr, yl);
    glm::vec3 coly = (1.0f-dx)*getPixel(xl, yr)+dx*getPixel(xr, yr);
    col = (1.0f-dy)*colx+dy*coly;
  }
  else if (mode == InterpolateMode::NEAR) {
    col = getPixel((int)uv.x, (int)uv.y);
  }

  return pixelScale*col;
}

float ImageTexture::generateDistribution2D(DiscreteDistribution2D& dd2d) const {
  dd2d.init(dd2dSize, dd2dSize);
  unsigned char* odata = new unsigned char[dd2dSize*dd2dSize*channel];
  stbir_resize(img, width, height, 0, odata, dd2dSize, dd2dSize, 0, 
    STBIR_TYPE_UINT8, channel, STBIR_ALPHA_CHANNEL_NONE, 0,
    STBIR_EDGE_CLAMP, STBIR_EDGE_CLAMP,
    STBIR_FILTER_BOX, STBIR_FILTER_BOX,
    STBIR_COLORSPACE_SRGB, nullptr
  );
  float tot_lumi = 0;
  for(int i = 0; i<dd2dSize; i++) {
    for(int j = 0; j<dd2dSize; j++) {
      int pos = (i*dd2dSize+j)*channel;
      glm::vec3 col;
      if(channel == 1) col = pixelScale * glm::vec3(odata[pos]);
      else col = pixelScale * glm::vec3(odata[pos], odata[pos+1], odata[pos+2]);
      float lumi = Luminance(col);
      tot_lumi += lumi;
      dd2d.addPdf(lumi, i);
    }  
  }
  dd2d.calcCdf();
  return tot_lumi / (dd2dSize*dd2dSize);
}

float ImageTexture::getAverageLuminance() const {
  float tot_lumi = 0.0f;
  for(int i = 0; i<height; i++) {
    for(int j = 0; j<width; j++) {
      int pos = (i*width+j)*channel;
      glm::vec3 col;
      if(channel == 1) col = pixelScale * glm::vec3(img[pos]);
      else col = pixelScale * glm::vec3(img[pos], img[pos+1], img[pos+2]);
      tot_lumi += Luminance(col);
    }  
  }
  return tot_lumi / (height*width);
}