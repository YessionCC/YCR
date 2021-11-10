#include<iostream>

#include "model.hpp"
#include "scene.hpp"
#include "camera.hpp"

#include "debug/pcshow.hpp"

int main() {
  glm::vec3 vtxs[3] = {{-100, -100, 0}, {100, -100, 0}, {0, 100, 0}};
  Model model(Mesh::CreateTriangle(vtxs));
  PCShower pc;
  Scene scene;

  Film film(400, 200, 60);
  Camera cam(film, {0, 0, 6}, {0, 0, -1});
  RayGenerator rGen(cam, 1);

  scene.addModel(model);
  scene.init();

  model.toPointClouds(pc, 1e4, glm::vec3(1.0f));

  Ray ray; glm::vec2 raster;
  while(rGen.genNextRay(ray, raster)) {
    Intersection itsc = scene.intersect(ray);
    float t = itsc.prim ? itsc.t : 20;
    ray.saveSegLineAsPointCloud(pc, t, {0,1,0}, 30);
  }

  pc.write("/home/yession/Code/Cpp/ycr/pc.ply");
  return 0;
}