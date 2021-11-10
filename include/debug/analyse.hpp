#pragma once 

#include <chrono>
#include <map>
#include <string>
#include <iostream>

#define __StartTimeAnalyse__(describe)\
 TimeAnalyser::analyser().addKeyPoint__Time_Analyse(describe);

#define __EndTimeAnalyse__\
 TimeAnalyser::analyser().terminateKey__Time_Analyse();

#define __ShowTimeAnalyse__\
  TimeAnalyser::analyser().showAnalyse();

class TimeAnalyser {
private:
  using KeyT = std::chrono::system_clock::time_point;
  std::map<std::string, double> itemConsume;

  std::string lastDescribe;
  KeyT lastKey;
  bool ifStartCount;

public:
  TimeAnalyser(): ifStartCount(false) {}

  static TimeAnalyser& analyser() {
    static TimeAnalyser any;
    return any;
  }

  void addKeyPoint__Time_Analyse(std::string describe) {
    if(ifStartCount) {
      std::cout<<"Time Analyser Warning: should close last counting"<<std::endl;
      return;
    }
    KeyT keyt = std::chrono::system_clock::now();
    lastKey = keyt;
    lastDescribe = describe;
    ifStartCount = true;
  }

  void terminateKey__Time_Analyse() {
    if(!ifStartCount) {
      std::cout<<"Time Analyser Warning: close counting twice"<<std::endl;
      return;
    }
    KeyT keyt = std::chrono::system_clock::now();
    std::chrono::duration<double> d(keyt - lastKey);
    itemConsume[lastDescribe]+=d.count();
    ifStartCount = false;
  }

  void showAnalyse() {
    double tot = 0;
    for(auto& item: itemConsume) {
      tot+=item.second;
    }
    std::cout<<"Total Time: "<<tot<<std::endl;
    for(auto& item: itemConsume) {
      std::cout<<"Analyse: "<<item.first<<
        ", consume: "<<item.second<<", accounting for: "<<
        item.second / tot * 100 <<"%"<<std::endl;
    }
  }

};