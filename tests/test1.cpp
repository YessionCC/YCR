#include <iostream>

#include "model.hpp"
#include "scene.hpp"
#include "camera.hpp"
#include "path.hpp"

#include "debug/pcshow.hpp"
#include "debug/analyse.hpp"

#include <iomanip>

// NOTICE: glm::vec no default init value, you can not regard it as 0 !!!

int main() {
  glm::vec3 vtxsl[3] = {{-8,6,0}, {-8,7,0}, {-8,6,1}};
  glm::vec3 vtxsl2[3] = {{0,0,0}, {0,0,4}, {4,4,4}};
  glm::vec3 vtxs2[3] = {{0,0,0}, {100,0,0}, {0,100,0}};
  glm::vec3 vtxs3[3] = {{0,0,0}, {0,100,0}, {0,0,100}};
  glm::vec3 vtxs4[3] = {{0,0,0}, {0,0,100}, {100,0,0}};

  Model nano("/home/yession/Code/Cpp/ycr/models/Nanosuit/nanosuit.obj");
  Model cube("/home/yession/Code/Cpp/ycr/models/Cube/cube.obj");
  Model sphere(CustomMesh::CreateSphere(glm::vec3(0, -3, 0), 3));
  Model bkg(VertexMesh::CreateTriangle(vtxs2));
  bkg.addMesh(VertexMesh::CreateTriangle(vtxs3));
  bkg.addMesh(VertexMesh::CreateTriangle(vtxs4));

  Model lgt(VertexMesh::CreateTriangle(vtxsl));
  Model lgt2(VertexMesh::CreateTriangle(vtxsl2));

  Light* light = new Light(glm::vec3(500.0f));
  Light* light2 = new Light(glm::vec3(150.0f));
  BXDF* bxdf = new GlassSpecular(new SolidTexture(glm::vec3(1.0f)), 1.0f, 1.4f);
  ImageTexture* imgtex = 
    new ImageTexture("/home/yession/Code/Cpp/ycr/models/Cube/grid.jpeg");
  imgtex->setUVScale({5.0f, 5.0f});
  imgtex->setUVOffset({0.6f, 0.2f});
  BXDF* bxdf2 = new LambertianDiffuse(imgtex);
  BXDF* bxdf3 = new PerfectSpecular(new SolidTexture(glm::vec3(1.0f)));
  BXDF* bxdf4 = new LambertianDiffuse(new SolidTexture(glm::vec3(1.0f)));
  
  bkg.setBxdfForAllMeshes(bxdf4);
  lgt.setLightFoeAllMeshes(light);
  lgt2.setLightFoeAllMeshes(light2);
  sphere.setBxdfForAllMeshes(bxdf);
  cube.setBxdfForAllMeshes(bxdf4);
  cube.scale({6,6,6});
  cube.translate({0, 0, -4});
  //sphere.rotate({0,0,0}, 20);
  nano.translate({0, -7.5, 0});
  bkg.translate({-10, -7.5, -10});
  lgt2.translate({8, -4, 6});

  PCShower pc;
  Scene scene;
  //scene.addModel(nano);
  scene.addModel(lgt);
  //scene.addModel(lgt2);
  scene.addModel(bkg);
  scene.addModel(sphere);
  //scene.addModel(cube);

  scene.init();

  //nano.toPointClouds(pc, 1e4, glm::vec3(1.0f));
  bkg.toPointClouds(pc, 3e4, glm::vec3(0.4, 0.6, 0.1));
  lgt.toPointClouds(pc, 1e4, glm::vec3(1, 0, 0));
  //lgt2.toPointClouds(pc, 1e4, glm::vec3(1, 0, 0));
  //sphere.toPointClouds(pc, 1e4, glm::vec3(1));
  cube.toPointClouds(pc, 1e4, glm::vec3(1.0f));

  Film film(800, 800, 60);
  Camera cam(film, {0, 0, 14}, {0, 0, -1});
  RayGenerator rGen(cam, 1);

  cam.visualizePointCloud(pc, 18, 40);
  
  PathIntegrator integrator;
  
  integrator.render(rGen, scene, film);
  film.generateImage("/home/yession/Code/Cpp/ycr/img/glass_t2.jpg");
  film.clear();

  integrator.visualizeRender(pc, rGen, scene, film);

  std::cout<<"Render Complete"<<std::endl;
  //__ShowTimeAnalyse__
  //system("shutdown -h now");
  pc.write("/home/yession/Code/Cpp/ycr/pc.ply");

  return 0;
}