#pragma once

#include <glm/glm.hpp>

#include <string>


class Texture {
public:
  enum TexType {

  };

protected:
  TexType type;


public:
  
  virtual glm::vec3 tex2D(glm::vec2) = 0;

};


class SolidTexture : public Texture {

private:
  glm::vec3 col;

public:
  SolidTexture(glm::vec3 col): col(col) {}

  glm::vec3 tex2D(glm::vec2 uv) override {return col;}

};

class ImageTexture: public Texture {
public:
  enum InterpolateMode {
    BILINEAR,
    NEAR
  };

private:
  InterpolateMode mode;
  int width, height, channel, tot;
  glm::vec2 scale, offset;
  unsigned char *img; // TODO: deconstruct

private:
  glm::vec3 getPixel(int x, int y);

public:
  ImageTexture(const char* imgname, 
    InterpolateMode mode = InterpolateMode::BILINEAR);

  glm::vec3 tex2D(glm::vec2 uv) override;

  void setUVScale(glm::vec2 scale) {
    this->scale = scale;
  }

  void setUVOffset(glm::vec2 offset) {
    this->offset = offset;
  }

};