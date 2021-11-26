#pragma once

#include <random>
#include <glm/glm.hpp>

#include "const.hpp"

#define _ThreadSampler GeneralSampler::getThreadSampler()

class StratifiedSampler2D { // Stractify Sampler
private:
  int stract_w;
  float inv_w;
  int sample_cnt = 0;
  std::default_random_engine eng;
  std::uniform_real_distribution<float> urd;
  
public:
  StratifiedSampler2D(int w, int seed = 0): 
    stract_w(w), inv_w(1.0f/w),
    eng(seed), urd(0.0f, 1.0f){}

  glm::vec2 get2() {
    sample_cnt %= (stract_w*stract_w);
    int row = sample_cnt/stract_w;
    int col = sample_cnt%stract_w;
    sample_cnt++;
    return inv_w*glm::vec2(row+urd(eng), col+urd(eng));
  }

  void clear() {
    sample_cnt = 0;
  }

};

class GeneralSampler {
private:
  std::default_random_engine eng;
  std::uniform_real_distribution<float> urd;


public:
  GeneralSampler(): eng(0), urd(0.0f, 1.0f){}

  static GeneralSampler& getThreadSampler();

  inline void resetSeed(int seed) {eng.seed(seed);}

  inline glm::vec2 get2() {return {urd(eng), urd(eng)};}

  inline float get1() {return urd(eng);}

  // return r, theta, all can be treated as sinTheta, phi
  inline glm::vec2 uniSampleDisk() {return {glm::sqrt(urd(eng)), PI2*urd(eng)};}

  // return theta, phi
  inline glm::vec2 cosWeightHemi() {
    glm::vec2 rt = uniSampleDisk();
    return {glm::asin(rt.x), rt.y};
  }

  // return cosTheta, phi
  inline glm::vec2 uniSampleSphere() {
    return {1.0f - 2.0f*urd(eng), PI2*urd(eng)};
  }

  // return uv
  inline glm::vec2 uniSampleTriangle() {
    float zeta1 = glm::sqrt(urd(eng));
    return {1-zeta1, urd(eng)*zeta1};
  }

  inline float expSampleMedium(float sigmaT) {
    return -std::log(urd(eng)) / sigmaT;
  }
};