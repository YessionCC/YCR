#include <iostream>

#include "model.hpp"
#include "scene.hpp"
#include "camera.hpp"
#include "path.hpp"
#include "parallel.hpp"

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
  glm::vec3 vtxs5[4] = {{0,0,0}, {0,0,100}, {100,0,100}, {100, 0, 0}};

  Model nano("/home/yession/Code/Cpp/ycr/models/Nanosuit/nanosuit.obj");
  Model cube("/home/yession/Code/Cpp/ycr/models/Cube/cube.obj");
  Model sphere(CustomMesh::CreateSphere(glm::vec3(-4, -4, 1.2f), 3));
  Model sphere3(CustomMesh::CreateSphere(glm::vec3(-4, -4, 1.2f), 1.0f));
  Model bkg(VertexMesh::CreateTriangle(vtxs2));
  Model bkg2(VertexMesh::CreateRectangle(vtxs5));
  bkg.addMesh(VertexMesh::CreateTriangle(vtxs3));
  bkg.addMesh(VertexMesh::CreateTriangle(vtxs4));

  Model lgt(VertexMesh::CreateTriangle(vtxsl));
  Model lgt2(VertexMesh::CreateTriangle(vtxsl2));

  EnvironmentLight* envLight = 
    new EnvironmentLight(
      new ImageTexture("/home/yession/Code/Cpp/ycr/models/Cube/polar.jpg"));
  Light* light = new ShapeLight(glm::vec3(800.0f), lgt);
  Light* light2 = new ShapeLight(glm::vec3(150.0f), lgt2);
  Light* pLight = new PointLight(glm::vec3(1000), glm::vec3(6,6,6));
  Light* dLight = new DirectionalLight(glm::vec3(3), glm::vec3(0,-1,0));

  BXDF* bxdf = new GlassSpecular(new SolidTexture(glm::vec3(1.0f)), 1.0f, 1.4f);
  ImageTexture* imgtex = 
    new ImageTexture("/home/yession/Code/Cpp/ycr/models/Cube/grid2.jpeg");
  imgtex->setUVScale({5.0f, 5.0f});
  imgtex->setUVOffset({0.6f, 0.2f});
  ImageTexture* imgtex2 = 
    new ImageTexture("/home/yession/Code/Cpp/ycr/models/Cube/grid.jpeg");
  BXDF* bxdf2 = new LambertianDiffuse(imgtex);
  BXDF* bxdf5 = new LambertianDiffuse(imgtex2);
  BXDF* bxdf3 = new NoFrSpecular(new SolidTexture(glm::vec3(1.0f)));
  BXDF* bxdf4 = new LambertianDiffuse(new SolidTexture(glm::vec3(1.0f)));
  
  bkg.setBxdfForAllMeshes(bxdf4);
  bkg2.setBxdfForAllMeshes(bxdf2);
  sphere.setBxdfForAllMeshes(bxdf);
  sphere3.setBxdfForAllMeshes(bxdf5);
  cube.setBxdfForAllMeshes(bxdf5);
  nano.setBxdfForAllMeshes(bxdf);
  cube.rotate({0,1,0}, 30);
  cube.scale({5,5,5});
  cube.translate({-2, -4, -6});
  //sphere.rotate({0,0,0}, 20);
  nano.translate({0, -7.5, 0});
  bkg.translate({-10, -7.5, -10});
  bkg2.rotate({0,1,0}, 45);
  bkg2.translate({-70, -7.5, 0});
  lgt.scale(glm::vec3(4));
  lgt.translate({22,-13,2});
  lgt2.translate({8, -4, 6});
  lgt2.translate({-3, 0, -14});
  sphere3.scale(glm::vec3(1.5));
  sphere3.rotate({0, 0, 1}, 45);
  sphere3.translate({-4,0,5});
  sphere3.translate({-2,18,-1});

  Model sphere2 = sphere;
  sphere2.setBxdfForAllMeshes(bxdf3);
  sphere2.translate({8, 0, 0});
  Model cube2 = cube;
  cube2.setBxdfForAllMeshes(bxdf);
  cube2.scale(glm::vec3(500, 500, 500));
  cube2.translate({16,25,20});
  sphere.scale(glm::vec3(1.5));
  sphere.translate(glm::vec3(-1, 14, 0));

  glm::vec3 sigmaS(0.6); float sT = 0.006;
  Medium* medium = new Medium(
    sT, sigmaS, nullptr, new HenyeyPhase(0.5, sigmaS));

  PCShower pc;
  Scene scene;
  Film film(800, 800, 60);
  //Camera cam(film, {0, 0, 14}, {0, -0.4f, -1});
  Camera cam(film, {0, 14, 35}, {0, -0.4f, -1});

  scene.addGlobalMedium(medium);
  //scene.addModel(nano);
  scene.addLight(light);
  //scene.addLight(envLight);
  //scene.addLight(light2);
  //scene.addModel(bkg);
  scene.addModel(bkg2);
  //scene.addModel(sphere);
  //scene.addModel(sphere2);
  scene.addModel(sphere);
  //scene.addModel(cube);
  //scene.addMedium(medium, false);
  //scene.addModel(cube2);

  scene.init();

  //nano.toPointClouds(pc, 1e4, glm::vec3(1.0f));
  //bkg.toPointClouds(pc, 3e4, glm::vec3(0.4, 0.6, 0.1));
  bkg2.toPointClouds(pc, 3e4, glm::vec3(0.4, 0.6, 0.1));
  lgt.toPointClouds(pc, 1e4, glm::vec3(1, 0, 0));
  //lgt2.toPointClouds(pc, 1e4, glm::vec3(1, 0, 0));
  //sphere.toPointClouds(pc, 1e4, glm::vec3(1));
  //sphere2.toPointClouds(pc, 1e4, glm::vec3(1));
  sphere3.toPointClouds(pc, 1e4, glm::vec3(1));
  //cube.toPointClouds(pc, 1e4, glm::vec3(1.0f));
  //cube2.toPointClouds(pc, 1e4, glm::vec3(1.0f));

  cam.visualizePointCloud(pc, 18, 40);
  
  ParallelRenderer pRenderer(scene, cam, film, 4096*4, 12);
  pRenderer.render("/home/yession/Code/Cpp/ycr/img/glass_fb4.jpg");

  //integrator.visualizeRender(pc, rGen, scene, film);

  //__ShowTimeAnalyse__
  //system("shutdown -h now");
  pc.write("/home/yession/Code/Cpp/ycr/pc.ply");

  return 0;
}