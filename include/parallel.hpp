#pragma once

#include <vector>
#include <thread>
#include <queue>
#include <mutex>

#include "scene.hpp"
#include "sampler.hpp"
#include "integrator.hpp"

class NonProgressiveRenderer;
class ProgressiveRenderer;

class RenderThread {
protected:
  Film& film;
  const Scene& scene;
  const Integrator* integrator;

public:
  ~RenderThread() {}
  RenderThread(const Scene& scene, const Integrator* integrator, Film& film):
    film(film), scene(scene), integrator(integrator) {}
  virtual void render() = 0;
};

class NonProgressiveRenderThread: public RenderThread {
private:
  NonProgressiveRenderer& pMan;
  StractifiedRGen rayGen;

public:
  NonProgressiveRenderThread(const Scene& scene, const Integrator* integrator, 
    const Camera& cam, NonProgressiveRenderer& pMan, Film& film, int spp);
  void render();
};

class ProgressiveRenderThread: public RenderThread {
private:
  ProgressiveRenderer& pMan;
  HaltonRGen rayGen;
  int tot_thread_num, thread_idx;
  volatile int cur_spp;

public:
  ProgressiveRenderThread(const Scene& scene, const Integrator* integrator, 
    const Camera& cam, ProgressiveRenderer& pMan, Film& film, int totth, int thidx);
  void render();
  int getSpp() const {
    return cur_spp;
  }
};

class ParallelRenderer {
protected:
  Film& film;
  int threadNum;
  std::vector<std::thread*> renderThreads;
public:
  ~ParallelRenderer(){}
  ParallelRenderer(Film& film, int threadNum): film(film), threadNum(threadNum){}
  virtual void render(const char* outputDir) = 0;
};

// using stractify
class NonProgressiveRenderer: public ParallelRenderer {
private:
  int filmX, filmY, spp;
  int blockNum, expectCalculation;
  
  std::vector<NonProgressiveRenderThread> pRenders;
  std::queue<Block2D> rBlocks;

  std::mutex locker;

private:
  bool calcBlocks(int expectCalculation);

public:
  NonProgressiveRenderer(
    const Scene& scene, const Camera& cam, 
    const Integrator* integrator,
    Film& film, int spp, int threadNum, int expectCalculation=(1<<22));
  void render(const char* outputDir);
  bool getOneBlock(Block2D& block);
};

class ProgressiveRenderer: public ParallelRenderer {
private:
  std::vector<ProgressiveRenderThread> pRenders;
public:
  ProgressiveRenderer(
    const Scene& scene, const Camera& cam, 
    const Integrator* integrator,
    Film& film, int threadNum);
  void render(const char* outputDir);
  inline int getTotSpp() const {
    int res = 0;
    for(int i = 0; i<pRenders.size(); i++) {
      res += pRenders[i].getSpp();
    }
    return res;
  }
};