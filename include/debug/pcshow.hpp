#pragma once

#include <fstream>
#include <sstream>
#include <glm/glm.hpp>

class PCShower {
private:
  std::stringstream ss;
  int elementsNum;

public:
  PCShower(): elementsNum(0) {}

  void addItem(glm::vec3 pos, glm::vec3 col) {
    ss<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<" "<<
      col[0]<<" "<<col[1]<<" "<<col[2]<<std::endl;
    elementsNum++;
  }

  void write(const char* filename) {
    std::ofstream of(filename);
    of<<"ply\n"
    "format ascii 1.0\n"
    "element vertex "<<elementsNum<<"\n"
    "property float x\n"
    "property float y\n"
    "property float z\n"
    "property float red\n"
    "property float green\n"
    "property float blue\n"
    "end_header\n";
    of<<ss.str();
    of.close();
  }

};