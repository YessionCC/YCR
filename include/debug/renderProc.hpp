#pragma once 

#include <opencv2/opencv.hpp>

#include <glm/glm.hpp>

#include <thread>
#include <string>
#include <chrono>

#include "film.hpp"
#include "parallel.hpp"

class RenderProcShower {
private:
  const Film& film;
  bool terminateThread = false;
  std::chrono::system_clock::time_point startTime;

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
  void showProgressiveRenderProc(
    const ProgressiveRenderer& pRender, 
    float save_interval_sec, std::string save_prefix) const {
    int reX = film.getReX(), reY = film.getReY();
    unsigned char* imgMat = new unsigned char[reX*reY*3];
    cv::Mat im(film.getReY(), film.getReX(), CV_8UC3, imgMat);
    cv::namedWindow("Render", 0);
    cv::resizeWindow("Render", {800, 800});
    int save_cnt = 1;
    while(true) {
      if(terminateThread) break;
      film.generateImage(imgMat);
      cv::cvtColor(im, im, cv::COLOR_BGR2RGB);

      auto endTime = std::chrono::system_clock::now();
      auto usedTime = std::chrono::duration<double>(endTime - startTime);
      double elapse = usedTime.count();
      int cur_tot_spp = pRender.getTotSpp();
      if(elapse/save_interval_sec > save_cnt) {
        cv::imwrite(save_prefix+"_spp_"+std::to_string(cur_tot_spp)+
          "_time_"+std::to_string(elapse)+".jpg", im);
        std::cout<<"save image, spp = "<<cur_tot_spp<<std::endl;
        save_cnt ++;
      }

      cv::putText(im, "spp: "+std::to_string(cur_tot_spp), {15,15}, cv::FONT_HERSHEY_PLAIN, 1.2, {1,1,1}, 1.2);
      cv::putText(im, "time: "+std::to_string(elapse)+"s", {15,35}, cv::FONT_HERSHEY_PLAIN, 1.2, {1,1,1}, 1.2);
      cv::imshow("Render", im);
      cv::waitKey(100);
    }
    delete imgMat;
  }

public:

  RenderProcShower(const Film& film): film(film){
    startTime = std::chrono::system_clock::now();
  }

  void terminate() {terminateThread = true;}

  void showProc() const {
    std::thread sht(&RenderProcShower::showRenderProc, this);
    sht.detach(); 
    // if thread deconstruct without detach or join before, 
    // program will be terminated
  }
  void showProc(const ProgressiveRenderer& pRender, 
    std::string save_prefix = "res", 
    float save_interval_sec = 60) const {
    std::thread sht(
      &RenderProcShower::showProgressiveRenderProc, this, 
      std::ref(pRender), save_interval_sec, save_prefix);
    sht.detach(); 
  }
};  