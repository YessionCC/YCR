#include <iostream>

#include "model.hpp"
#include "scene.hpp"
#include "camera.hpp"
#include "path.hpp"
#include "bdpt.hpp"
#include "parallel.hpp"
#include "bxdfc.hpp"

#include "debug/pcshow.hpp"
#include "debug/analyse.hpp"
#include "debug/renderProc.hpp"

#include <iomanip>

PCShower pc;
Scene scene;
Film film(1366, 768, 90, true);
//Camera cam(film, {0, 0, 14}, {0, -0.4f, -1});
//Camera cam(film, {0, 6, 14}, {0, -0.45f, -1});
//Camera cam(film, {0, 14, 35}, {0, -0.4f, -1});
//Camera cam(film, {0, 14, 50}, {0, -0.4f, -1});
//Camera cam(film, {-4.1,2,0}, {1, -0.3, 0});
Camera cam(film, {-2.3439, 2.10188, 1.25665}, {0.845742, -0.497306, -0.193408});
RenderProcShower rShower(film);
ProgressiveRenderer pRenderer(scene, cam, new PathIntegrator(8), film, 12);

int main() {
  glm::vec3 vtxsl[3] = {{-8,6,0}, {-8,7,0}, {-8,6,1}};
  glm::vec3 vtxsl2[3] = {{0,4,0}, {4,4,4}, {0,4,4}};
  glm::vec3 vtxs2[3] = {{0,0,0}, {100,0,0}, {0,100,0}};
  glm::vec3 vtxs3[3] = {{0,0,0}, {0,100,0}, {0,0,100}};
  glm::vec3 vtxs4[3] = {{0,0,0}, {0,0,100}, {100,0,0}};
  glm::vec3 vtxs5[4] = {{0,0,0}, {0,0,100}, {100,0,100}, {100, 0, 0}};
  glm::vec3 vtxs6[4] = {{0,0,0}, {5,0,2}, {0,5,0}};


  Model casa("/home/yession/Code/Cpp/ycrr/models/Casa/casa2.obj");

  Model lgt(VertexMesh::CreateTriangle(vtxsl));
  Model lgt2(VertexMesh::CreateTriangle(vtxsl2));

  EnvironmentLight* envLight2 = 
    new EnvironmentLight(
      new SolidTexture(1.0f));
  Light* light = new ShapeLight(new SolidTexture(300.0f), lgt);
  Light* light2 = new ShapeLight(new SolidTexture(800.0f), lgt2);
  Light* pLight = new PointLight(glm::vec3(1000), glm::vec3(6,6,6));
  Light* dLight = new DirectionalLight(glm::vec3(3), glm::vec3(0,-1,0));

  
  BXDFNode* ggx_grey = new StandardGGXRefl(
    1.45f, new SolidTexture(0.04f), 
    new SolidTexture(glm::vec3(0.4f, 0.4f, 0.4f)), 
    new SolidTexture(1.0f)
  );
  BXDFNode* ggx_black = new StandardGGXRefl(
    1.45f, new SolidTexture(0.08f), 
    new SolidTexture(glm::vec3(0.02f, 0.02f, 0.02f)), 
    new SolidTexture(0.4f)
  );
  BXDFNode* ggx_white = new StandardGGXRefl(
    1.45f, new SolidTexture(0.04f), 
    new SolidTexture(glm::vec3(1.0f)), 
    new SolidTexture(1.0f)
  );
  BXDFNode* ggx_wood = new StandardGGXRefl(
    1.45f, new SolidTexture(0.002f), 
    new ImageTexture("/home/yession/Code/Cpp/ycr/models/Cube/wood2.jpg"), 
    new SolidTexture(1.0f)
  );
  BXDFNode* ggx_wall = new StandardGGXRefl(
    1.45f, new SolidTexture(0.1f), 
    new SolidTexture({1.0, 1.0, 0.74}), 
    new SolidTexture(0.4f)
  );
  BXDFNode* metal = new PerfectSpecular(new SolidTexture(1.0f));

  BXDFNode* glass = new PerfectGlass(1.4f, new SolidTexture(1.0f));

  BXDFNode* lam_white = new LambertianReflection(new SolidTexture(0.9f));
  BXDFNode* lam_brown = new LambertianReflection(new SolidTexture({0.29, 0.128, 0.03}));

  BXDFNode* cere = new StandardSpecular(
    1.4f, new SolidTexture(1.0f), new SolidTexture(1.0f)
  );

  lgt.scale(glm::vec3(4));
  lgt.translate({22,-13,2});
  lgt2.translate({8, -4, 6});
  lgt2.translate({-3, 0, -14});

  casa.setBxdfForAllMeshes(ggx_white);
  casa.setBxdfForOneMesh(glass, 8);
  casa.setBxdfForOneMesh(ggx_wood, 4);
  casa.setBxdfForOneMesh(cere, 14);
  casa.setBxdfForOneMesh(ggx_wall, 0);
  casa.setBxdfForMeshes(ggx_black, {276, 268, 292, 284});
  casa.setBxdfForMeshes(ggx_grey, {9, 10});
  casa.setBxdfForRangeMeshes(lam_white, 26, 29);

  casa.setBxdfForRangeMeshes(metal, 277, 283);
  casa.setBxdfForRangeMeshes(metal, 269, 275);
  casa.setBxdfForRangeMeshes(metal, 285, 291);
  casa.setBxdfForRangeMeshes(metal, 293, 299);

  casa.setBxdfForRangeMeshes(lam_brown, 15, 20);
  casa.setBxdfForRangeMeshes(ggx_grey, 302, 305);

  scene.addLight(light);
  //scene.addLight(envLight2);
  scene.addModel(casa);

  scene.init();

  rShower.showProc(pRenderer, "/home/yession/Code/Cpp/ycr/img/BDPT_RES/res", 1000.0f);
  pRenderer.render("/home/yession/Code/Cpp/ycr/img/BDPT_RES/res.jpg");
  rShower.terminate();

  return 0;
}