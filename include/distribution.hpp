#pragma once

#include <vector>
#include <iostream>
#include <algorithm>

#include "sampler.hpp"

class DiscreteDistribution {

private:
  std::vector<float> cdf;
  float sum_pdf;

  bool cdfHaveCalc = false;

public:
  DiscreteDistribution(){}
  // will calc pdf
  DiscreteDistribution(std::vector<float>& pdf): cdf(pdf) {
    calcCdf();
  }
  
  // will not calc pdf, after all pdf added, need call calcCdf explicitly
  void addPdf(float pdf) {
    cdf.push_back(pdf);
  }

  void calcCdf() {
    if(cdfHaveCalc) return;
    for(unsigned int i = 1; i<cdf.size(); i++) cdf[i]+=cdf[i-1];
    sum_pdf = cdf.back();
    for(unsigned int i = 0; i<cdf.size(); i++) cdf[i]/=sum_pdf;
    cdfHaveCalc = true;
  }

  // return sample idx, pdf
  int sample(float& pdf) {
    if(!cdfHaveCalc) {
      std::cout<<"sample distribution before initiating!"<<std::endl;
      return -1;
    }
    float u1 = SampleShape::sampler().get1();
    unsigned int idx = std::lower_bound(cdf.begin(), cdf.end(), u1) - cdf.begin();
    if(idx >= cdf.size()) idx = cdf.size() - 1;
    if(idx == 0) pdf = cdf[0];
    else pdf = cdf[idx] - cdf[idx-1];
    return idx;
  }

};