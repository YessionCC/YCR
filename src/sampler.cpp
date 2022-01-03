#include "sampler.hpp"

thread_local GeneralSampler threadSampler;

GeneralSampler& GeneralSampler::getThreadSampler() {
  return threadSampler;
}

inline float ReverseInt32Base2(unsigned int n) {
  n = (n<<16) | (n>>16);
  n = ((n&0x00ff00ff)<<8) | ((n&0xff00ff00)>>8);
  n = ((n&0x0f0f0f0f)<<4) | ((n&0xf0f0f0f0)>>4);
  n = ((n&0x33333333)<<2) | ((n&0xcccccccc)>>2);
  n = ((n&0x55555555)<<1) | ((n&0xaaaaaaaa)>>1);
  const double t =  1.0/(1ll<<32);
  return n*t;
} 

inline float ReverseInt32Base3(unsigned int n) {
  float invbaseN = 1.0f;
  float invbase = 1.0f/3;
  unsigned int res = 0;
  while(n) {
    int r = n/3;
    int d = n - r*3;
    res = res*3+d;
    invbaseN *= invbase;
    n = r;
  }
  return res*invbaseN;
}

glm::vec2 HaltonSampler2D::get2(int sp_index) {
  sp_index = (unsigned int)sp_index;
  return glm::vec2(ReverseInt32Base2(sp_index), ReverseInt32Base3(sp_index));
}