#pragma once

#include <vector>
#include <iostream>
#include <algorithm>

#include "sampler.hpp"

class DiscreteDistribution1D {

private:
  std::vector<float> cdf;
  float sum_pdf;

  bool cdfHaveCalc = false;

public:
  DiscreteDistribution1D(){}
  // will calc pdf
  DiscreteDistribution1D(const std::vector<float>& pdf): cdf(pdf) {
    calcCdf();
  }
  DiscreteDistribution1D(int num) {
    cdf.resize(num, 0);
  }
  
  // will not calc pdf, after all pdf added, need call calcCdf explicitly
  void addPdf(float pdf) {
    cdf.push_back(pdf);
  }

  // must pre resize
  void addPdf(float pdf, int idx) {
    cdf[idx] = pdf;
  }

  void calcCdf() {
    if(cdfHaveCalc) return;
    for(unsigned int i = 1; i<cdf.size(); i++) cdf[i]+=cdf[i-1];
    sum_pdf = cdf.back();
    if(sum_pdf == 0.0f) {
      std::cout<<"WARNING: Distribution has 0 cdf, "<<
        "default to uniform distribution"<<std::endl;
      float uni = 1.0f / cdf.size();
      for(unsigned int i = 0; i<cdf.size(); i++) cdf[i]=uni*(i+1);
    }
    else for(unsigned int i = 0; i<cdf.size(); i++) cdf[i]/=sum_pdf;
    cdfHaveCalc = true;
  }

  // return sample idx, pdf
  int sample(float& pdf) const{
    if(!cdfHaveCalc) {
      std::cout<<"sample distribution1D before initiating!"<<std::endl;
      return -1;
    }
    float u1 = _ThreadSampler.get1();
    unsigned int idx = std::lower_bound(cdf.begin(), cdf.end(), u1) - cdf.begin();
    if(idx >= cdf.size()) idx = cdf.size() - 1;
    if(idx == 0) pdf = cdf[0];
    else pdf = cdf[idx] - cdf[idx-1];
    return idx;
  }

  inline float getSumPdf() const {return sum_pdf;}
  inline float getPdf(int pos) const {
    if(cdf.size() == 0) {
      std::cout << "ERROR: DD1D 0 size!"<<std::endl;
      return 0.0f;
    }
    if(pos >= cdf.size()) pos = cdf.size() - 1;
    if(pos == 0) return cdf[pos];
    return cdf[pos]-cdf[pos-1];
  }
};

class DiscreteDistribution2D {
private:
  std::vector<float> ccdf;
  std::vector<DiscreteDistribution1D> ppdf;
  float sum_ccdf;
  bool cdfHaveCalc = false;
  int row, col;

public:
  DiscreteDistribution2D(int row, int col): row(row), col(col) {
    init(row, col);
  }
  DiscreteDistribution2D() {}

  void init(int row, int col) {
    this->row = row;
    this->col = col;
    sum_ccdf = 0;
    ccdf.resize(row, 0);
    ppdf.resize(row);
  }

  void addPdf(float pdf, int row) {
    if(row >= ccdf.size()) {
      std::cout<<"ERROR: DD2D add Pdf out of range"<<std::endl;
      return;
    }
    ppdf[row].addPdf(pdf);
  }

  // must pre resize
  void addPdf(float pdf, int row, int col) {
    if(row >= this->row || col >= this->col) {
      std::cout<<"ERROR: DD2D add Pdf out of range"<<std::endl;
      return;
    }
    ppdf[row].addPdf(pdf, col);
  }

  void calcCdf() {
    for(unsigned int i = 0; i<ccdf.size(); i++) {
      ppdf[i].calcCdf();
      ccdf[i] = ppdf[i].getSumPdf();
    }
    for(unsigned int i = 1; i<ccdf.size(); i++) ccdf[i] += ccdf[i-1];
    sum_ccdf = ccdf.back();
    if(sum_ccdf == 0.0f) {
      std::cout<<"ERROR! DD2D has all zero value!"<<std::endl;
      return;
    }
    for(unsigned int i = 0; i<ccdf.size(); i++) ccdf[i] /= sum_ccdf;
    cdfHaveCalc = true;
  }

  bool sample(float& pdf, int& sx, int& sy) const {
    if(!cdfHaveCalc) {
      std::cout<<"sample distribution2D before initiating!"<<std::endl;
      return false;
    }
    float u1 = _ThreadSampler.get1();
    unsigned int idx = std::lower_bound(ccdf.begin(), ccdf.end(), u1) - ccdf.begin();
    if(idx >= ccdf.size()) idx = ccdf.size() - 1;
    if(idx == 0) pdf = ccdf[0];
    else pdf = ccdf[idx] - ccdf[idx-1];
    float pdf2;
    int idx2 = ppdf[idx].sample(pdf2);
    pdf *= pdf2;
    sy = idx; sx = idx2;
    return true;
  }

  bool sample01(float& pdf, int& sx, int& sy) const {
    bool res = sample(pdf, sx, sy);
    sx /= col; sy /= row;
    return res;
  }

  inline int getRow() const {return row;}
  inline int getCol() const {return col;}

  inline float getPdf(int posx, int posy) const {
    if(posy >= ccdf.size()) posy = ccdf.size();
    return ppdf[posy].getPdf(posx);
  }
};