#include "path.hpp"
#include "bxdf.hpp"
#include "debug/analyse.hpp"

SubPathGenerator::TransportMode SubPathGenerator::transportMode = 
  SubPathGenerator::TransportMode::FromCamera;

int SubPathGenerator::createSubPath(
  const Ray& start_ray, const Scene& scene, 
  int max_bounce) {

  Ray ray = start_ray, sampleRay;
  PathVertex pvtx;
  Intersection itsc;
  glm::vec3 beta(1.0f);
  const Medium* inMedium = scene.getGlobalMedium();
  const BXDF* bxdf; float bxdfWeight;
  bool hasMedium = scene.hasMediumInScene();
  
  for(int bounce=0; bounce<max_bounce; bounce++) {

    if(IsBlack(beta)) {
      tstate = TerminateState::TotalBlack;
      return bounce;
    }
    if(!ray.checkDir()) {
      tstate = TerminateState::CalcERROR;
      return bounce;
    }
    //itsc = scene.intersect(ray, itsc.prim);
    itsc = scene.intersect(ray, nullptr);
    if(inMedium)
      beta *= inMedium->sampleNextItsc(ray, itsc);

    if(!itsc.prim) {
      // handle infinite far envLight
      if(scene.envLight && transportMode==TransportMode::FromCamera) {
        if(bounce == 0 || _HasFeature(pathVertices[bounce-1].bxdf->getType(), DELTA))
          diracRadiance = scene.envLight->evaluate(itsc, -ray.d)*beta;
        lastVertex.beta = beta; // record envlight
        lastVertex.dir_o = -ray.d;
        lastVertex.lt = scene.envLight;
        scene.envLight->genRayItsc(itsc, ray, ray.o);
        lastVertex.itsc = itsc;
      }
      tstate = TerminateState::NoItsc; 
      return bounce;
    }

    const Material& mat = itsc.prim->getMesh()->material;
    if(itsc.cosTheta(ray.d)>0.0f) itsc.reverseNormal();

    if(mat.light) { // handle first/specular to shape light
      if(!itsc.normalReverse && transportMode==TransportMode::FromCamera) {
        if(bounce == 0 || _HasFeature(pathVertices[bounce-1].bxdf->getType(), DELTA))
          diracRadiance += mat.light->evaluate(itsc, -ray.d)*beta;
        if(!mat.bxdfNode) { // record nobxdf light
          lastVertex.beta = beta;
          lastVertex.dir_o = -ray.d;
          lastVertex.lt = mat.light;
          lastVertex.itsc = itsc;
        }
      } 
      if(!mat.bxdfNode) { // we allow no bxdf light
        tstate = TerminateState::ItscLight;
        return bounce;
      }
    }
    if(!mat.bxdfNode) {
      std::cout<<"WARNING: Detect No BXDF Material(not light)"<<std::endl;
      return bounce;
    }
    
    ray.o = itsc.itscVtx.position; 
    ray.d = -ray.d;

    bxdfWeight = mat.getBXDF(itsc, ray, bxdf);

    pvtx.dir_o = ray.d;
    pvtx.beta = beta; // set dir_o and beta before update
    pvtx.bxdf = bxdf;
    // for particles, mediumOutside = medium
    // for surface, for direct light evaluate only use outside medium
    pvtx.inMedium = mat.mediumOutside;

    beta *= bxdfWeight * bxdf->sample_ev(itsc, ray, sampleRay);
    ray = sampleRay;//

    // if the bxdf is medium(not has medium, like pureTrans), its not surface, so no shift
    // if the bxdf is RTBoth, then the offset is up to the direct light
    if(!_IsType(bxdf->getType(), NoSurface))
      itsc.maxErrorOffset(ray.d, ray.o);
    if(!_IsType(bxdf->getType(), NoSurface) &&
       !_IsType(bxdf->getType(), RTBoth))
      itsc.itscVtx.position = ray.o;

    pvtx.itsc = itsc;
    pathVertices.push_back(pvtx);

    // if is medium particle, it always in medium
    if(_IsType(bxdf->getType(), NoSurface) || !hasMedium) continue;
    inMedium = itsc.isRayToInside(ray)? mat.mediumInside:mat.mediumOutside;
    
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
  if(_IsType(pvtx.bxdf->getType(), REFLECT) && 
    pvtx.itsc.cosTheta(dirToLight) < 0) return; 
  // TR BXDF exitant radiance always on the opposite side with normal
  else if(_IsType(pvtx.bxdf->getType(), TRANSMISSION) && 
    pvtx.itsc.cosTheta(dirToLight) > 0) return; 
  // RTBoth BXDF exitant radiance uncertain
  else if(_IsType(pvtx.bxdf->getType(), RTBoth)) {
    pvtx.itsc.maxErrorOffset(
      dirToLight, 
      pvtx.itsc.itscVtx.position);
  }

  rayToLight.o = pvtx.itsc.itscVtx.position;
  rayToLight.d = dirToLight;

  if(scene.hasMediumInScene()) {
    if(scene.occlude(rayToLight, len, tr, pvtx.inMedium, litsc.prim)) return; 
  }
  else { // if there are no medium, we can run more effectively
    if(scene.occlude(rayToLight, len, litsc.prim)) return; 
  }

  float invdis2 = 1.0f/(len*len);
  glm::vec3 leCosDivR2 = 
    invdis2*lCosTheta*lt->evaluate(litsc, -rayToLight.d);

  lastRay.o = rayToLight.o;
  lastRay.d = pvtx.dir_o;

  glm::vec3 lastBeta = pvtx.bxdf->evaluate(pvtx.itsc, lastRay, rayToLight);
  // debug error detect
  L = tr*pvtx.beta*leCosDivR2*lastBeta / lpdf_A;

  if(needMIS) {
    float lpdf_S = PaToPw(lpdf_A, len*len, lCosTheta);
    float pdfBxdf = pvtx.bxdf->sample_pdf(pvtx.itsc, rayToLight, lastRay);
    L *= PowerHeuristicWeight(lpdf_S, pdfBxdf);
  }
}

/**************************EstimateDirectLightByBSDF*****************************/
// has medium
void PathIntegrator::estimateDirectLightByBSDF(
  const Scene& scene, const DiscreteDistribution1D& ldd1d,
  PathVertex& pvtx, glm::vec3& L, bool needMIS) const {

  Ray rayToLight, lastRay; 
  glm::vec3 tr(1.0f); L = glm::vec3(0.0f);

  lastRay.o = pvtx.itsc.itscVtx.position;
  lastRay.d = pvtx.dir_o;
  
  glm::vec3 beta = pvtx.bxdf->sample_ev(pvtx.itsc, lastRay, rayToLight);
  if(IsBlack(beta)) return;

  Intersection itsc = scene.intersectDirectly(rayToLight, pvtx.inMedium, tr);

  const Light* lt = nullptr; 
  if(itsc.prim) {
    lt = itsc.prim->getMesh()->material.light;
    if(lt) {
      if(itsc.normalReverse) return;
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
    float bxdfPdf = pvtx.bxdf->sample_pdf(pvtx.itsc, rayToLight, lastRay);
    float lpdf_A = scene.getLightPdf(ldd1d, lt)*lt->getItscPdf(itsc, rayToLight);
    glm::vec3 dir = itsc.itscVtx.position - pvtx.itsc.itscVtx.position;
    float dist2 = dir.x*dir.x+dir.y*dir.y+dir.z*dir.z;
    float cosTheta = itsc.itscVtx.cosTheta(-rayToLight.d);
    float lpdf_S = PaToPw(lpdf_A, dist2, cosTheta); // from area to solid angle;
    L *= PowerHeuristicWeight(bxdfPdf, lpdf_S);
  }
}
// no medium, so we can reuse samples
void PathIntegrator::estimateDirectLightByBSDF(
    const Scene& scene, const DiscreteDistribution1D& ldd1d,
    PathVertex* pvtx1, PathVertex* pvtx2, const LastVertex* lstvtx,
    glm::vec3& L, bool needMIS) const{
  L = glm::vec3(0.0f);
  const Light* lt;
  Intersection itsc;
  Ray rayi, rayo;
  if(!lstvtx) {
    lt = pvtx2->itsc.prim->getMesh()->material.light;
    if(!lt || pvtx2->itsc.normalReverse) return;
    L = pvtx2->beta * lt->evaluate(pvtx2->itsc, pvtx2->dir_o);
    rayi.d = -pvtx2->dir_o;
    itsc = pvtx2->itsc;
  }
  else {
    lt = lstvtx->lt;
    if(!lt) return;
    L = lstvtx->beta * lt->evaluate(lstvtx->itsc, lstvtx->dir_o); 
    rayi.d = -lstvtx->dir_o;
    itsc = lstvtx->itsc;
  }
  rayo.d = pvtx1->dir_o;
  rayi.o = rayo.o = pvtx1->itsc.itscVtx.position;
  if(needMIS) {
    float bxdfPdf = pvtx1->bxdf->sample_pdf(pvtx1->itsc, rayi, rayo);
    float lpdf_A = scene.getLightPdf(ldd1d, lt)*lt->getItscPdf(itsc, rayi);
    glm::vec3 dir = itsc.itscVtx.position - pvtx1->itsc.itscVtx.position;
    float dist2 = dir.x*dir.x+dir.y*dir.y+dir.z*dir.z;
    float cosTheta = itsc.itscVtx.cosTheta(-rayi.d);
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
  bool hasMedium = scene.hasMediumInScene();

  while(rayGen.genNextRay(ray, rasPos)) {
    // if((int)rasPos.x == 255 && (int)rasPos.y == 228) {
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
      if(_HasFeature(pvtx.bxdf->getType(), DELTA)) continue;

      scene.getPositionLightDD1D(pvtx.itsc.itscVtx.position, ldd1d);

      bool needMis = pvtx.bxdf->needMIS(pvtx.itsc);

      // Default sample by Li
      estimateDirectLightByLi(scene, ldd1d, pvtx, L1, needMis);
      CheckRadiance(L1, rasPos);
      radiance += L1;

      if(needMis) {
        if(hasMedium)
          estimateDirectLightByBSDF(scene, ldd1d, pvtx, L2, needMis);
        else {
          if(i == pathVtxs.size() - 1)
            estimateDirectLightByBSDF(scene, ldd1d, &pathVtxs[i], nullptr, 
              &subpathGen.getLastVertex(), L2, needMis);
          else 
            estimateDirectLightByBSDF(scene, ldd1d, &pathVtxs[i], &pathVtxs[i+1], 
              nullptr, L2, needMis);
        }
        CheckRadiance(L2, rasPos);
        radiance += L2;
      }
    }
    //__EndTimeAnalyse__

    film.addRadiance(radiance, 1, rasPos.x, rasPos.y);
    //film.addSplat(radiance, rasPos);
  }
}

float BDPTIntegrator::calcMISWeight(
    const std::vector<PathVertex>& pcam,
    const std::vector<PathVertex>& plt,
    int s, int t) const {
  

}

void BDPTIntegrator::calcFwdPdfs(
  const std::vector<PathVertex>& pvtx, std::vector<float>& res) const {
  for(int i = 1; i<pvtx.size(); i++) {
    if(_HasFeature(pvtx[i-1].bxdf->getType(), DELTA)) continue;
    Ray rayo{pvtx[i-1].itsc.itscVtx.position, pvtx[i-1].dir_o};
    Ray rayi{pvtx[i-1].itsc.itscVtx.position, -pvtx[i].dir_o};
    float pdfs = 
      pvtx[i-1].bxdf->sample_pdf(pvtx[i-1].itsc, rayi, rayo);
    glm::vec3 rd = pvtx[i].itsc.itscVtx.position - 
      pvtx[i-1].itsc.itscVtx.position;
    float len2 = rd.x*rd.x+rd.y*rd.y+rd.z*rd.z;
    float cosTheta = pvtx[i].itsc.itscVtx.cosTheta(pvtx[i].dir_o);
    float pdfa = PwToPa(pdfs, len2, cosTheta);
    res[i] = pdfa;
  }
}

void BDPTIntegrator::calcRevPdfs(
  const std::vector<PathVertex>& pvtx, std::vector<float>& res) const {
  for(int i = (int)pvtx.size()-3; i>=0; i--) {
    if(_HasFeature(pvtx[i+1].bxdf->getType(), DELTA)) continue;
    Ray rayo{pvtx[i+1].itsc.itscVtx.position, -pvtx[i+2].dir_o};
    Ray rayi{pvtx[i+1].itsc.itscVtx.position, pvtx[i+1].dir_o};
    float pdfs = 
      pvtx[i+1].bxdf->sample_pdf(pvtx[i+1].itsc, rayi, rayo);
    glm::vec3 rd = pvtx[i].itsc.itscVtx.position - 
      pvtx[i+1].itsc.itscVtx.position;
    float len2 = rd.x*rd.x+rd.y*rd.y+rd.z*rd.z;
    float cosTheta = pvtx[i].itsc.itscVtx.cosTheta(-pvtx[i+1].dir_o);
    float pdfa = PwToPa(pdfs, len2, cosTheta);
    res[i] = pdfa;
  }
}

void BDPTIntegrator::render(const Scene& scene, SubPathGenerator& subpathGen,
    RayGenerator& rayGen, Film& film) const {

  DiscreteDistribution1D ldd1d;
  Ray rayFromCam, rayFromLight; 
  glm::vec2 rasPosCam, rasPosLight;
  const Light* lt; Intersection litsc;

  std::vector<float> camFwdPdf, camRevPdf;
  std::vector<float> ltFwdPdf, ltRevPdf;

  while(rayGen.genNextRay(rayFromCam, rasPosCam)) {

    subpathGen.clear();
    SubPathGenerator::transportMode = PathTransportMode::FromCamera;
    int bcam = subpathGen.createSubPath(rayFromCam, scene, max_bounce);
    LastVertex lsvtx = subpathGen.getLastVertex();
    std::vector<PathVertex> pvtxs_cam = subpathGen.getPathVtxs();


    if(pvtxs_cam.size() == 0) {
      if(!lsvtx.lt) continue;
      glm::vec3 L = lsvtx.lt->evaluate(lsvtx.itsc, lsvtx.dir_o);
      film.addSplat(L, rasPosCam);
      continue;
    }

    subpathGen.clear();
    float pdf_l = scene.sampleALight(lt);
    float pdf_A, pdf_D;
    lt->genRay(litsc, rayFromLight, pdf_A, pdf_D);
    SubPathGenerator::transportMode = PathTransportMode::FromLight;
    int blt = subpathGen.createSubPath(rayFromLight, scene, max_bounce);
    std::vector<PathVertex> pvtxs_lt = subpathGen.getPathVtxs();

    camFwdPdf.resize(pvtxs_cam.size());
    camRevPdf.resize(pvtxs_cam.size());
    ltFwdPdf.resize(pvtxs_lt.size());
    ltRevPdf.resize(pvtxs_lt.size());

    calcFwdPdfs(pvtxs_cam, camFwdPdf);
    calcFwdPdfs(pvtxs_lt, ltFwdPdf);
    calcRevPdfs(pvtxs_cam, camRevPdf);
    calcRevPdfs(pvtxs_lt, ltRevPdf);

    for(int i = 0; i<pvtxs_cam.size(); i++) {

    }
  }
}
