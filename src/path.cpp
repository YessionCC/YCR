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

void PathIntegrator::render(const Scene& scene, SubPathGenerator& subpathGen,
  RayGenerator& rayGen, Film& film) const {

  Ray ray; glm::vec2 rasPos;
  auto& pathVtxs = subpathGen.getPathVtxs();

  while(rayGen.genNextRay(ray, rasPos)) {
    // if((int)rasPos.x == 344 && (int)rasPos.y == 169) {
    //   int a = 0;
    // }

    subpathGen.clear();

    //__StartTimeAnalyse__("subpath")
    int bounce = subpathGen.createSubPath(ray, scene, max_bounce);
    //__EndTimeAnalyse__

    Ray rayToLight, lastRay;
    Intersection litsc; const Light* lt; float len;
    glm::vec3 radiance = subpathGen.getDiracLight(), tr = glm::vec3(1.0f);

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
}

// this func need to be changed after
void PathIntegrator::visualizeRender(
  PCShower& pc, RayGenerator& rayGen, Scene& scene, Film& film) {

}
