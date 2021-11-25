#include "path.hpp"
#include "bxdf.hpp"
#include "debug/analyse.hpp"

int SubPathGenerator::createSubPath(
  const Ray& start_ray, const Scene& scene, 
  int max_bounce) {

  Ray ray = start_ray, sampleRay;
  PathVertex pvtx;
  Intersection itsc;
  glm::vec3 beta(1.0f);
  const Medium* inMedium = scene.getGlobalMedium();
  
  for(int bounce=0; bounce<max_bounce; bounce++) {
    //itsc = scene.intersect(ray, itsc.prim);
    itsc = scene.intersect(ray, nullptr);
    if(inMedium)
      beta *= inMedium->sampleNextItsc(ray, itsc);

    if(!itsc.prim) {
      if(scene.envLight &&  // handle infinite far envLight
        (bounce == 0 || _HasType(pathVertices[bounce-1].bxdfType, DELTA))) {
        diracRadiance = scene.envLight->evaluate(itsc, -ray.d)*beta;
      }
      tstate = TerminateState::NoItsc; 
      return bounce;
    }

    const Mesh* mesh = itsc.prim->getMesh();
    const BXDF* bxdf = mesh->bxdf;

    if(mesh->light) { // handle dirac light
      if(bounce == 0 || _HasType(pathVertices[bounce-1].bxdfType, DELTA)) {
        diracRadiance = mesh->light->evaluate(itsc, -ray.d)*beta ;
      } 
      tstate = TerminateState::ItscLight;
      return bounce;
    }
    
    ray.o = itsc.itscVtx.position; 
    ray.d = -ray.d;
    beta *= bxdf->sample_ev(itsc, ray, sampleRay);
    ray = sampleRay;

    pvtx.bxdfType = bxdf->getType();
    pvtx.inMedium = inMedium;
    pvtx.dir_o = -ray.d;
    pvtx.beta = beta;
    pvtx.itsc = itsc;
    pathVertices.push_back(pvtx);

    // if the bxdf is medium(not has medium, like pureTrans), its not surface, so no shift
    // if the bxdf is RTBoth, then the offset is up to the direct light
    if(!_IsType(pvtx.bxdfType, MEDIUM) &&
       !_IsType(pvtx.bxdfType, RTBoth))
      itsc.maxErrorOffset(ray.d, ray.o);//

    // if is medium particle, it always in medium
    if(_IsType(pvtx.bxdfType, MEDIUM)) continue;
    if(itsc.cosTheta(ray.d) < 0) inMedium = mesh->mediumInside;
    else inMedium = mesh->mediumOutside;
    
  }
  tstate = TerminateState::UpToMaxBounce;
  return max_bounce;
}

void PathIntegrator::render(RayGenerator& rayGen, const Scene& scene, Film& film) {

  Ray ray; glm::vec2 rasPos;
  auto& pathVtxs = subPathGenerator.getPathVtxs();

  while(rayGen.genNextRay(ray, rasPos)) {
    // if((int)rasPos.x == 344 && (int)rasPos.y == 169) {
    //   int a = 0;
    // }

    subPathGenerator.clear();

    //__StartTimeAnalyse__("subpath")
    int bounce = subPathGenerator.createSubPath(ray, scene, max_bounce);
    //__EndTimeAnalyse__

    Ray rayToLight, lastRay;
    Intersection litsc; const Light* lt; float len;
    glm::vec3 radiance = subPathGenerator.getDiracLight(), tr = glm::vec3(1.0f);

    //__StartTimeAnalyse__("dirlight")
    for(unsigned int i = 0; i<pathVtxs.size(); i++) {
      
      if(_HasType(pathVtxs[i].bxdfType, DELTA)) continue;

      //float lpdf = scene.sampleALight(lt);
      float lpdf = scene.dynamicSampleALight(
        lt, pathVtxs[i].itsc.itscVtx.position);
      lpdf *= lt->getItscOnLight(litsc, pathVtxs[i].itsc.itscVtx.position);
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

      const Mesh* mesh = pathVtxs[i].itsc.prim->getMesh();

      //if(scene.occlude(rayToLight, len, litsc.prim)) continue; 
      if(scene.occlude(rayToLight, len, tr, pathVtxs[i].inMedium, litsc.prim)) 
        continue; 
      //if(scene.occlude(pathVtxs[i].itsc, litsc, rayToLight, len)) continue; 

      float invdis2 = 1.0f/(len*len);
      glm::vec3 leCosDivR2 = 
        invdis2*lt->evaluate(litsc, -rayToLight.d);
      const BXDF* lastBxdf = mesh->bxdf;
      if(!lastBxdf) continue; //
      lastRay.o = rayToLight.o;
      lastRay.d = pathVtxs[i].dir_o;

      glm::vec3 lastBeta = lastBxdf->evaluate(pathVtxs[i].itsc, lastRay, rayToLight);
      // debug error detect
      glm::vec3 pathRad = tr*pathVtxs[i].beta*leCosDivR2*lastBeta / lpdf;

      if(pathRad.x<0 || pathRad.y<0 || pathRad.z<0) {
        std::cout<<"find negative radiance: "
          <<rasPos.x<<" "<<rasPos.y<<std::endl;
      }
      if(pathRad.x>1e5 || pathRad.y>1e5 || pathRad.z>1e5) {
        std::cout<<"find huge radiance: "
          <<rasPos.x<<" "<<rasPos.y<<std::endl;
      }

      radiance += pathRad;
    }
    //__EndTimeAnalyse__

    film.addRadiance(radiance, 1, rasPos.x, rasPos.y);
    //film.addSplat(radiance, rasPos);
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

    Intersection litsc; Ray rayToLight; const Light* lt; float len;
    for(unsigned int i = 0; i<pathVtxs.size(); i++) {

      if(_HasType(pathVtxs[i].bxdfType, DELTA)) continue;

      float lpdf = scene.sampleALight(lt);
      lpdf *= lt->getItscOnLight(litsc, pathVtxs[i].itsc.itscVtx.position);

      if(scene.occlude(pathVtxs[i].itsc, litsc, rayToLight, len)) continue; 

      Ray r;
      r.o = pathVtxs[i].itsc.itscVtx.position;
      r.d = glm::normalize(litsc.itscVtx.position - r.o);
      r.saveSegLineAsPointCloud(pc, 20, coll, pnum);
    }
  }
  rayGen.clear();
}
