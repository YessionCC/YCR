#pragma once 

#include <opencv2/opencv.hpp>

#include <glm/glm.hpp>

#include <thread>

#include "film.hpp"

class RenderProcShower {
private:
  const Film& film;
  bool terminateThread = false;

  void showRenderProc() const {
    int reX = film.getReX(), reY = film.getReY();
    unsigned char* imgMat = new unsigned char[reX*reY*3];
    cv::Mat im(film.getReY(), film.getReX(), CV_8UC3, imgMat);
    cv::namedWindow("Render", 0);
    cv::resizeWindow("Render", {800, 800});
    while(true) {
      if(terminateThread) break;
      film.generateImage(imgMat);
      cv::cvtColor(im, im, cv::COLOR_BGR2RGB);
      cv::imshow("Render", im);
      cv::waitKey(100);
    }
    delete imgMat;
  }

public:

  RenderProcShower(const Film& film): film(film){}

  void terminate() {terminateThread = true;}

  void showProc() const {
    std::thread sht(&RenderProcShower::showRenderProc, this);
    sht.detach(); 
    // if thread deconstruct without detach or join before, 
    // program will be terminated
  }
};  