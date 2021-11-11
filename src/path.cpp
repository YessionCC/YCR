#include "path.hpp"
#include "debug/analyse.hpp"

int SubPathGenerator::createSubPath(
  const Ray& start_ray, Scene& scene, 
  int max_bounce) {

  Ray ray = start_ray, sampleRay;
  PathVertex pvtx;
  glm::vec3 beta(1.0f);
  Intersection itsc;

  for(int i=0; i<max_bounce; i++) {
    //itsc = scene.intersect(ray, itsc.prim);
    itsc = scene.intersect(ray, nullptr);
    // when there are curve primitive, should not specify the ignored prim

    if(!itsc.prim) {tstate = TerminateState::NoItsc; return i;}
    Mesh* mesh = itsc.prim->getMesh();

    if(mesh->light) {
      if(i == 0 || _HasType(pathVertices[i-1].bxdfType, DELTA)) {
        diracRadiance = mesh->light->evaluate(itsc, -ray.d)*beta;
      } 
      tstate = TerminateState::ItscLight;
      return i;
    }

    BXDF* bxdf = itsc.prim->getMesh()->bxdf;
    if(bxdf) pvtx.bxdfType = bxdf->getType();
    else pvtx.bxdfType = BXDF::BXDFType::None;
    
    ray.d = -ray.d;
    ray.o = itsc.itscVtx.position; 

    if(bxdf) beta *= bxdf->sample_ev(itsc, ray, sampleRay);
    ray = sampleRay;

    // when there are not curve primitive, error offset can comment
    itsc.maxErrorOffset(ray.d, ray.o);//
    // if the bxdf is RTBoth, then the offset is up to the direct light
    if(!_IsType(pvtx.bxdfType, RTBoth)) {
      itsc.itscVtx.position = ray.o;
    }
    pvtx.beta = beta;
    pvtx.dir_o = -ray.d;
    pvtx.itsc = itsc;
    pathVertices.push_back(pvtx);
  }
  tstate = TerminateState::UpToMaxBounce;
  return max_bounce;
}

void PathIntegrator::render(RayGenerator& rayGen, Scene& scene, Film& film) {

  Ray ray; glm::vec2 rasPos;
  auto& pathVtxs = subPathGenerator.getPathVtxs();
  while(rayGen.genNextRay(ray, rasPos)) {

    // if((int)rasPos.x == 161 && (int)rasPos.y == 520) {
    //   int c = 1;
    // }
    // if((int)rasPos.x == 523 && (int)rasPos.y == 521) {
    //   int c = 1;
    // }

    subPathGenerator.clear();

    //__StartTimeAnalyse__("subpath")
    int bounce = subPathGenerator.createSubPath(ray, scene, max_bounce);
    //__EndTimeAnalyse__

    Ray rayToLight, lastRay;
    Intersection litsc; float len;
    glm::vec3 radiance = subPathGenerator.getDiracLight();

    //__StartTimeAnalyse__("dirlight")
    for(unsigned int i = 0; i<pathVtxs.size(); i++) {
      
      if(_HasType(pathVtxs[i].bxdfType, DELTA)) continue;

      Primitive* lightPrim;
      float lpdf = scene.sampleALight(lightPrim);
      lpdf *= lightPrim->getAPointOnSurface(litsc);
      // TODO
      glm::vec3 dirToLight = litsc.itscVtx.position - 
        pathVtxs[i].itsc.itscVtx.position;
      len = glm::length(dirToLight);
      dirToLight /= len;

      // REFLECT BXDF exitant radiance always on the same side with normal
      if(_IsType(pathVtxs[i].bxdfType, REFLECT) && 
        pathVtxs[i].itsc.cosTheta(dirToLight) < 0) continue; 
      // TR BXDF exitant radiance always on the opposite side with normal
      else if(_IsType(pathVtxs[i].bxdfType, TRANSMISSION) && 
        pathVtxs[i].itsc.cosTheta(dirToLight) > 0) continue; 
      // RTBoth BXDF exitant radiance uncertain
      else if(_IsType(pathVtxs[i].bxdfType, RTBoth)) {
        pathVtxs[i].itsc.maxErrorOffset(
          dirToLight, 
          pathVtxs[i].itsc.itscVtx.position);
      }

      rayToLight.o = pathVtxs[i].itsc.itscVtx.position;
      rayToLight.d = dirToLight;

      if(scene.occlude(rayToLight, len, litsc.prim)) continue; 
      //if(scene.occlude(pathVtxs[i].itsc, litsc, rayToLight, len)) continue; 

      float invdis2 = 1.0f/(len*len);
      glm::vec3 leCosDivR2 = 
        invdis2*lightPrim->getMesh()->light->evaluate(litsc, -rayToLight.d);
      BXDF* lastBxdf = pathVtxs[i].itsc.prim->getMesh()->bxdf;
      if(!lastBxdf) continue;
      lastRay.o = rayToLight.o;
      lastRay.d = pathVtxs[i].dir_o;
      glm::vec3 lastBeta = 
        glm::abs(pathVtxs[i].itsc.cosTheta(rayToLight.d)) *
        lastBxdf->evaluate(pathVtxs[i].itsc, lastRay, rayToLight);

      // debug error detect
      glm::vec3 pathrad = pathVtxs[i].beta*leCosDivR2*lastBeta / lpdf;
      if(pathrad.x<0 || pathrad.y<0 || pathrad.z<0) {
        std::cout<<"find negative radiance: "
          <<rasPos.x<<" "<<rasPos.y<<std::endl;
      }

      radiance += pathrad;
    }
    //__EndTimeAnalyse__

    //film.addRadiance(radiance, 1, rasPos.x, rasPos.y);
    film.addSplat(radiance, rasPos);
  }
  rayGen.clear();
}

// this func need to be changed after
void PathIntegrator::visualizeRender(
  PCShower& pc, RayGenerator& rayGen, Scene& scene, Film& film) {

  glm::vec3 coll(1, 0.5, 1), colr;
  int pnum = 600; float rate = 1.4e-4;
  Ray ray; glm::vec2 rasPos;
  auto& pathVtxs = subPathGenerator.getPathVtxs();
  while(rayGen.genNextRay(ray, rasPos)) {
    if(SampleShape::sampler().get1()>rate) continue;
    subPathGenerator.clear();
    int bounce = subPathGenerator.createSubPath(ray, scene, max_bounce);
    for(unsigned int i = 0; i<pathVtxs.size(); i++) {
      if(i%3 == 0) colr = glm::vec3(1,0,0);
      if(i%3 == 1) colr = glm::vec3(0,1,0);
      if(i%3 == 2) colr = glm::vec3(0,0,1);
      if(i == 0) ray.saveSegLineAsPointCloud(pc, pathVtxs[0].itsc.t, colr, pnum);
      else {
        Ray vsr;
        vsr.o = pathVtxs[i-1].itsc.itscVtx.position;
        vsr.d = -pathVtxs[i].dir_o;
        vsr.saveSegLineAsPointCloud(pc, pathVtxs[i].itsc.t, colr, pnum);
      }
    }
    if(bounce == 0 && 
      subPathGenerator.getTerminateState() == PathTerminateState::ItscLight){
      continue;
    }

    Intersection litsc; Ray rayToLight; float len;
    for(unsigned int i = 0; i<pathVtxs.size(); i++) {

      if(_HasType(pathVtxs[i].bxdfType, DELTA)) continue;

      Primitive* lightPrim;
      float lpdf = scene.sampleALight(lightPrim);
      lpdf *= lightPrim->getAPointOnSurface(litsc);

      if(scene.occlude(pathVtxs[i].itsc, litsc, rayToLight, len)) continue; 

      Ray r;
      r.o = pathVtxs[i].itsc.itscVtx.position;
      r.d = glm::normalize(litsc.itscVtx.position - r.o);
      r.saveSegLineAsPointCloud(pc, 20, coll, pnum);
    }
  }
  rayGen.clear();
}
