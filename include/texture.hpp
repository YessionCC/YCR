#pragma once

#include <glm/glm.hpp>

#include <string>

#include "distribution.hpp"

class Texture {
public:
  enum TexType {
    Solid,
    Image
  };

protected:
  TexType type;

public:
  Texture(TexType type): type(type) {}
  virtual ~Texture() {}
  virtual glm::vec3 tex2D(glm::vec2) const = 0;
  inline TexType getType() const {return type;}
};


class SolidTexture : public Texture {

private:
  glm::vec3 col;

public:
  SolidTexture(glm::vec3 col): Texture(TexType::Solid), col(col) {}
  SolidTexture(float col): Texture(TexType::Solid), col(col) {}

  glm::vec3 tex2D(glm::vec2 uv) const override {return col;}

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
  const unsigned char *img; // TODO: deconstruct

  const int dd2dSize = 64;

private:
  glm::vec3 getPixel(int x, int y) const;

public:
  ~ImageTexture() {delete img;}
  ImageTexture(const char* imgname, 
    InterpolateMode mode = InterpolateMode::BILINEAR);

  glm::vec3 tex2D(glm::vec2 uv) const override;

  void setUVScale(glm::vec2 scale) {
    this->scale = scale;
  }

  void setUVOffset(glm::vec2 offset) {
    this->offset = offset;
  }
  // return avg_lumi
  float generateDistribution2D(DiscreteDistribution2D& dd2d) const;

};