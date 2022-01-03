#include "parallel.hpp"

#include <iostream>
#include <cstdio>
#include <chrono>

NonProgressiveRenderThread::NonProgressiveRenderThread(
  const Scene& scene, const Integrator* integrator, 
  const Camera& cam, NonProgressiveRenderer& pMan, Film& film, int spp): 
  RenderThread(scene, integrator, film), pMan(pMan), rayGen(cam, spp){}

void NonProgressiveRenderThread::render(){
  Block2D curBlock;
  while(pMan.getOneBlock(curBlock)) {
    rayGen.reset(curBlock);
    integrator->render(scene, &rayGen, film);
  }
} 

ProgressiveRenderThread::ProgressiveRenderThread(
  const Scene& scene, const Integrator* integrator, 
  const Camera& cam, ProgressiveRenderer& pMan, Film& film, int totth, int thidx): 
  RenderThread(scene, integrator, film), pMan(pMan), rayGen(cam), 
  tot_thread_num(totth), thread_idx(thidx), cur_spp(0){}

void ProgressiveRenderThread::render(){
  while(true) {
    rayGen.reset(tot_thread_num*cur_spp+thread_idx);
    integrator->render(scene, &rayGen, film);
    cur_spp++;
  }
} 


NonProgressiveRenderer::NonProgressiveRenderer(
  const Scene& scene, const Camera& cam, const Integrator* integrator,
  Film& film, int spp, int threadNum, int expectCalculation): 
  ParallelRenderer(film, threadNum),
  filmX(cam.getReX()), filmY(cam.getReY()), spp(spp), 
  expectCalculation(expectCalculation){

  for(int i = 0; i<threadNum; i++) {
    pRenders.emplace_back(NonProgressiveRenderThread(scene, integrator, cam, *this, film, spp));
  }
}

bool NonProgressiveRenderer::calcBlocks(int expectCalculation) {
  if(threadNum == 1) {
    rBlocks.push(Block2D{filmX, filmY, 0, 0});
    return true;
  }
  unsigned long long totCalc = 1ull * filmX * filmY * spp;
  unsigned long long expBlock = totCalc / expectCalculation;
  if(expBlock == 0) {
    std::cout<<"Calculation too large, please reduce it!"<<std::endl;
    return false;
  }
  if(expBlock < threadNum) {
    std::cout<<
    "Calculation too large, "
    "it may reduce performance"
    <<std::endl;
  }
  int cubeSize = (int)std::sqrt(filmX * filmY / expBlock);
  if(cubeSize == 0) {
    std::cout<<"Calculation too large, please reduce it!"<<std::endl;
    return false;
  }
  blockNum = 0;
  for(int i = 0; i<filmX+cubeSize; i+=cubeSize) {
    int width = std::min(filmX - i, cubeSize);
    if(width == 0) continue;
    for(int j = 0; j<filmY+cubeSize; j+=cubeSize) {
      int height = std::min(filmY - j, cubeSize);
      if(height == 0) continue;
      rBlocks.push(Block2D{width, height, i, j});
      blockNum ++;
    }
  }
  std::cout<<
    "Blocks calc complete: "<<
    blockNum<<" blocks totally, "<<
    "block size: "<<cubeSize<<"*"<<cubeSize<<std::endl;
  return true;
}

bool NonProgressiveRenderer::getOneBlock(Block2D& block) {
  std::lock_guard<std::mutex> lock(locker);
  if(rBlocks.empty()) return false;
  block = rBlocks.front();
  rBlocks.pop();
  std::printf("%d/%d blocks complete\n", blockNum - (int)rBlocks.size(), blockNum);
  return true;
}

void NonProgressiveRenderer::render(const char* outputDir) {
  if(!calcBlocks(expectCalculation)) {
    std::cout<<"Block calc failed, render terminated!"<<std::endl;
    return;
  }
  auto startTime = std::chrono::system_clock::now();
  for(int i = 0; i<threadNum-1; i++) {
    //!! NOTICE: When transfer params with ref(&), the thread will always
    // use its value(copy) as the param (no matter whether the func take ref as param or not)
    // it will cause the params used in thread are not related what expected
    // use std::ref to ensure the params transfering as ref(&)
    renderThreads.push_back(new std::thread(&NonProgressiveRenderThread::render, std::ref(pRenders[i])));
  }
  pRenders.back().render();
  for(int i = 0; i<threadNum-1; i++) {
    renderThreads[i]->join();
  }
  film.generateImage(outputDir);
  auto endTime = std::chrono::system_clock::now();
  auto usedTime = std::chrono::duration<double>(endTime - startTime);
  std::cout<<"Render complete in "<<usedTime.count()<<"s"<<std::endl;
} 


ProgressiveRenderer::ProgressiveRenderer(
  const Scene& scene, const Camera& cam, 
  const Integrator* integrator,
  Film& film, int threadNum): ParallelRenderer(film, threadNum) {

  for(int i = 0; i<threadNum; i++) {
    pRenders.emplace_back(ProgressiveRenderThread(scene, integrator, cam, *this, film, threadNum, i));
  }
}

void ProgressiveRenderer::render(const char* outputDir) {
  
  for(int i = 0; i<threadNum-1; i++) {
    renderThreads.push_back(new std::thread(&ProgressiveRenderThread::render, std::ref(pRenders[i])));
  }
  pRenders.back().render();
}