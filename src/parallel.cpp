#include "parallel.hpp"

#include <iostream>
#include <cstdio>
#include <chrono>

ParallelRenderThread::ParallelRenderThread(
  const Scene& scene, const PathIntegrator& integrator, 
  const Camera& cam, ParallelRenderer& pMan, Film& film, int spp): 
  film(film), scene(scene), integrator(integrator), pMan(pMan), rayGen(cam, spp){}

void ParallelRenderThread::render(){
  Block2D curBlock;
  while(pMan.getOneBlock(curBlock)) {
    rayGen.reset(curBlock);
    integrator.render(scene, subpathGen, rayGen, film);
  }
} 



ParallelRenderer::ParallelRenderer(const Scene& scene, const Camera& cam, 
  Film& film, int spp, int threadNum):
  film(film), filmX(cam.getReX()), filmY(cam.getReY()), 
  spp(spp), threadNum(threadNum){

  for(int i = 0; i<threadNum; i++) {
    pRenders.emplace_back(ParallelRenderThread(scene, integrator, cam, *this, film, spp));
  }
}

bool ParallelRenderer::calcBlocks(int expectCalculation) {
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

bool ParallelRenderer::getOneBlock(Block2D& block) {
  std::lock_guard<std::mutex> lock(locker);
  if(rBlocks.empty()) return false;
  block = rBlocks.front();
  rBlocks.pop();
  std::printf("%d/%d blocks complete\n", blockNum - (int)rBlocks.size(), blockNum);
  return true;
}

void ParallelRenderer::render(const char* outputDir, int expectCalculation) {
  if(!calcBlocks(expectCalculation)) {
    std::cout<<"Block calc failed, render terminated!"<<std::endl;
    return;
  }
  auto startTime = std::chrono::system_clock::now();
  for(int i = 0; i<threadNum-1; i++) {
    renderThreads.push_back(new std::thread(&ParallelRenderThread::render, pRenders[i]));
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