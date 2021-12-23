#pragma once

#include <vector>
#include <thread>
#include <queue>
#include <mutex>

#include "scene.hpp"
#include "sampler.hpp"
#include "integrator.hpp"

class ParallelRenderer;

class ParallelRenderThread {
private:
  Film& film;
  const Scene& scene;
  const Integrator* integrator;
  ParallelRenderer& pMan;
  RayGenerator rayGen;

public:
  ParallelRenderThread(const Scene& scene, const Integrator* integrator, 
    const Camera& cam, ParallelRenderer& pMan, Film& film, int spp);
  void render();
};

class ParallelRenderer {
private:
  Film& film;
  int filmX, filmY, spp;
  int threadNum, blockNum;
  
  std::vector<ParallelRenderThread> pRenders;
  std::vector<std::thread*> renderThreads;
  std::queue<Block2D> rBlocks;

  std::mutex locker;

private:
  bool calcBlocks(int expectCalculation);

public:
  ParallelRenderer(
    const Scene& scene, const Camera& cam, 
    const Integrator* integrator,
    Film& film, int spp, int threadNum);
  void render(const char* outputDir, int expectCalculation = (1<<22));
  bool getOneBlock(Block2D& block);
};