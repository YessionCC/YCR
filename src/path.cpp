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

    if(IsBlack(beta)) {
      tstate = TerminateState::TotalBlack;
      return bounce;
    }
    //itsc = scene.intersect(ray, itsc.prim);
    itsc = scene.intersect(ray, nullptr);
    if(inMedium)
      beta *= inMedium->sampleNextItsc(ray, itsc);

    if(!itsc.prim) {
      if(scene.envLight &&  // handle infinite far envLight
        (bounce == 0 || _HasFeature(pathVertices[bounce-1].bxdfType, DELTA))) {
        diracRadiance = scene.envLight->evaluate(itsc, -ray.d)*beta;
      }
      tstate = TerminateState::NoItsc; 
      return bounce;
    }

    const Mesh* mesh = itsc.prim->getMesh();
    const BXDF* bxdf = mesh->bxdf;

    if(mesh->light) { // handle dirac light
      if(bounce == 0 || _HasFeature(pathVertices[bounce-1].bxdfType, DELTA)) {
        diracRadiance += mesh->light->evaluate(itsc, -ray.d)*beta ;
      } 
      if(!bxdf) { // we allow no bxdf light
        tstate = TerminateState::ItscLight;
        return bounce;
      }
    }
    if(!bxdf) {
      std::cout<<"WARNING: Detect No BXDF Material(not light)"<<std::endl;
      return bounce;
    }
    
    ray.o = itsc.itscVtx.position; 
    ray.d = -ray.d;

    pvtx.dir_o = ray.d;
    pvtx.beta = beta;
    pvtx.bxdfType = bxdf->getType();
    pvtx.inMedium = inMedium;
    pvtx.itsc = itsc;
    pathVertices.push_back(pvtx);

    beta *= bxdf->sample_ev(itsc, ray, sampleRay);
    ray = sampleRay;//

    // if the bxdf is medium(not has medium, like pureTrans), its not surface, so no shift
    // if the bxdf is RTBoth, then the offset is up to the direct light
    if(!_IsType(pvtx.bxdfType, NoSurface) &&
       !_IsType(pvtx.bxdfType, RTBoth))
      itsc.maxErrorOffset(ray.d, ray.o);//

    // if is medium particle, it always in medium
    if(_IsType(pvtx.bxdfType, NoSurface)) continue;
    if(itsc.cosTheta(ray.d) < 0) inMedium = mesh->mediumInside;
    else inMedium = mesh->mediumOutside;
    
  }
  tstate = TerminateState::UpToMaxBounce;
  return max_bounce;
}

/**************************EstimateDirectLightByLi*****************************/

void PathIntegrator::estimateDirectLightByLi(
  const Scene& scene,  const DiscreteDistribution1D& ldd1d,
  PathVertex& pvtx, glm::vec3& L, bool needMIS) const{
  //float lpdf = scene.sampleALight(lt);
  // float lpdf = scene.dynamicSampleALight(
  //   lt, pathVtxs[i].itsc.itscVtx.position);
  // lpdf *= lt->getItscOnLight(litsc, pathVtxs[i].itsc.itscVtx.position);
  Ray rayToLight, lastRay;
  Intersection litsc; const Light* lt; float len;
  glm::vec3 tr(1.0f); L = glm::vec3(0.0f);

  float lpdf_A = scene.dynamicSampleALight(ldd1d, lt);
  lpdf_A *= lt->getItscOnLight(litsc, pvtx.itsc.itscVtx.position);
  // TODO
  glm::vec3 dirToLight = litsc.itscVtx.position - 
    pvtx.itsc.itscVtx.position;
  len = glm::length(dirToLight);
  dirToLight /= len;

  float lCosTheta = litsc.itscVtx.cosTheta(-dirToLight);
  if(lCosTheta <= 0.0f) return;

  // REFLECT BXDF exitant radiance always on the same side with normal
  if(_IsType(pvtx.bxdfType, REFLECT) && 
    pvtx.itsc.cosTheta(dirToLight) < 0) return; 
  // TR BXDF exitant radiance always on the opposite side with normal
  else if(_IsType(pvtx.bxdfType, TRANSMISSION) && 
    pvtx.itsc.cosTheta(dirToLight) > 0) return; 
  // RTBoth BXDF exitant radiance uncertain
  else if(_IsType(pvtx.bxdfType, RTBoth)) {
    pvtx.itsc.maxErrorOffset(
      dirToLight, 
      pvtx.itsc.itscVtx.position);
  }

  rayToLight.o = pvtx.itsc.itscVtx.position;
  rayToLight.d = dirToLight;

  const Mesh* mesh = pvtx.itsc.prim->getMesh();

  //if(scene.occlude(rayToLight, len, litsc.prim)) return; 
  if(scene.occlude(rayToLight, len, tr, pvtx.inMedium, litsc.prim)) 
    return; 
  //if(scene.occlude(pvtx.itsc, litsc, rayToLight, len)) return; 

  float invdis2 = 1.0f/(len*len);
  glm::vec3 leCosDivR2 = 
    invdis2*lCosTheta*lt->evaluate(litsc, -rayToLight.d);
  const BXDF* lastBxdf = mesh->bxdf;
  if(!lastBxdf) return; //
  lastRay.o = rayToLight.o;
  lastRay.d = pvtx.dir_o;

  glm::vec3 lastBeta = lastBxdf->evaluate(pvtx.itsc, lastRay, rayToLight);
  // debug error detect
  L = tr*pvtx.beta*leCosDivR2*lastBeta / lpdf_A;

  if(needMIS) {
    float lpdf_S = PaToPw(lpdf_A, len*len, lCosTheta);
    float pdfBxdf = lastBxdf->sample_pdf(pvtx.itsc, rayToLight, lastRay);
    L *= PowerHeuristicWeight(lpdf_S, pdfBxdf);
  }
}

/**************************EstimateDirectLightByBSDF*****************************/

void PathIntegrator::estimateDirectLightByBSDF(
  const Scene& scene, const DiscreteDistribution1D& ldd1d,
  PathVertex& pvtx, glm::vec3& L, bool needMIS) const {

  Ray rayToLight, lastRay; 
  glm::vec3 tr(1.0f); L = glm::vec3(0.0f);
  const Mesh* mesh = pvtx.itsc.prim->getMesh();
  const BXDF* bxdf = mesh->bxdf;

  lastRay.o = pvtx.itsc.itscVtx.position;
  lastRay.d = pvtx.dir_o;
  
  glm::vec3 beta = bxdf->sample_ev(pvtx.itsc, lastRay, rayToLight);
  if(IsBlack(beta)) return;

  Intersection itsc = scene.intersectDirectly(rayToLight, pvtx.inMedium, tr);

  const Light* lt = nullptr; float cosTheta = 1.0f;
  if(itsc.prim) {
    lt = itsc.prim->getMesh()->light;
    if(lt) {
      cosTheta = itsc.itscVtx.cosTheta(-rayToLight.d);
      if(cosTheta <= 0.0f) return;
      L = tr*pvtx.beta*beta*lt->evaluate(itsc, -rayToLight.d);
    }
    else return;
  }
  else {
    if(scene.envLight) {// TODO: currently we only need to take envLight into account
      lt = scene.envLight;
      scene.envLight->genRayItsc(itsc, rayToLight, pvtx.itsc.itscVtx.position);
      L = tr*pvtx.beta*beta*scene.envLight->evaluate(itsc, -rayToLight.d);
    }
    else return;
  }

  if(needMIS) {
    float bxdfPdf = bxdf->sample_pdf(pvtx.itsc, rayToLight, lastRay);
    float lpdf_A = scene.getLightPdf(ldd1d, lt)*lt->getItscPdf(itsc, rayToLight);
    glm::vec3 dir = itsc.itscVtx.position - pvtx.itsc.itscVtx.position;
    float dist2 = dir.x*dir.x+dir.y*dir.y+dir.z*dir.z;
    float lpdf_S = PaToPw(lpdf_A, dist2, cosTheta); // from area to solid angle;
    L *= PowerHeuristicWeight(bxdfPdf, lpdf_S);
  }
}

/**************************Main Render Loop*****************************/

void PathIntegrator::render(const Scene& scene, SubPathGenerator& subpathGen,
  RayGenerator& rayGen, Film& film) const {

  Ray ray; glm::vec2 rasPos;
  DiscreteDistribution1D ldd1d; // light power distr for every shading point
  auto& pathVtxs = subpathGen.getPathVtxs();

  while(rayGen.genNextRay(ray, rasPos)) {
    // if((int)rasPos.x == 178 && (int)rasPos.y == 239) {
    //   int a = 0;
    // }

    subpathGen.clear();

    //__StartTimeAnalyse__("subpath")
    int bounce = subpathGen.createSubPath(ray, scene, max_bounce);
    //__EndTimeAnalyse__

    glm::vec3 radiance = subpathGen.getDiracLight(), L1, L2;

    //__StartTimeAnalyse__("dirlight")
    for(unsigned int i = 0; i<pathVtxs.size(); i++) {
      
      PathVertex& pvtx = pathVtxs[i];
      if(_HasFeature(pvtx.bxdfType, DELTA)) continue;

      scene.getPositionLightDD1D(pvtx.itsc.itscVtx.position, ldd1d);

      bool needMis = false;
      const BXDF* bxdf = pvtx.itsc.prim->getMesh()->bxdf;
      if(bxdf) needMis = bxdf->needMIS(pvtx.itsc);

      // Default sample by Li
      estimateDirectLightByLi(scene, ldd1d, pvtx, L1, needMis);
      CheckRadiance(L1, rasPos);
      radiance += L1;

      if(needMis) {
        estimateDirectLightByBSDF(scene, ldd1d, pvtx, L2, needMis);
        CheckRadiance(L2, rasPos);
        radiance += L2;
      }
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
