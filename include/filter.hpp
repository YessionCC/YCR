#pragma once

#include <glm/glm.hpp>

class Filter {
protected:
  float radius; // rect radius

public:
  virtual ~Filter() {}
  Filter(float r): radius(r) {}

  inline float getRadius() const {return radius;}

  // uv is the relative pos
  virtual float evaluate(glm::vec2 uv) const = 0;

};

class BoxFilter: public Filter {
public:
  BoxFilter(float r): Filter(r){}

  float evaluate(glm::vec2 uv) const override {
    return 1.0f;
  }
};

class GaussianFilter: public Filter {
private:
  float alpha;
  float sub;

  inline float _eva(float x) const {
    return glm::exp(-alpha*x*x) - sub;
  }

public:
  GaussianFilter(float r, float alpha = 1.0f): 
    Filter(r), alpha(alpha) {
    sub = glm::exp(-alpha*radius*radius);
  }

  float evaluate(glm::vec2 uv) const override {
    return _eva(uv.x)*_eva(uv.y);
  }
};

class MitchellFilter: public Filter {
private:
  float B, C;
  float sub;

  inline float _eva(float x) const {
    x = glm::abs(2*x);
    if(x>1)
      return ((-B-6*C)*x*x*x+(6*B+30*C)*x*x+
        (-12*B-48*C)*x+(8*B+24*C))*(1.f/6.f);
    else 
      return ((12-9*B-6*C)*x*x*x+
        (-18+12*B+6*C)*x*x+
        (6-2*B))*(1.f/6.f);
  }

public:
  // recommend B+2C = 1
  MitchellFilter(float r, float B=.4f, float C=.3f): 
    Filter(r), B(B), C(C) {}

  float evaluate(glm::vec2 uv) const override {
    return _eva(uv.x/radius)*_eva(uv.y/radius);
  }
};