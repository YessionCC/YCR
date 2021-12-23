#include "bdpt.hpp"
#include "bxdf.hpp"

// make sure that the first pvtx is ini_pvtx
void SubPathGenerator::createSubPath(
    const Ray& start_ray, const Scene& scene, 
    std::vector<PathVertex>& pathVertices, 
    TerminateState& tstate, int max_bounce) {
  
  Intersection itsc;  // surface itsc(change every bounce)
  Ray ray = start_ray, sample_ray;
  glm::vec3 beta = pathVerticesCam[0].beta;
  const Medium* inMedium = scene.getGlobalMedium();

  bool hasMedium = scene.hasMediumInScene();

  for(int bounce = 0; bounce<max_bounce; bounce++) {
    itsc = scene.intersect(ray, nullptr);
    if(inMedium)
      beta *= inMedium->sampleNextItsc(ray, itsc);

    // if no itsc, two possible cases:
    // no envLight, really no itsc
    // has envLight, hit the envlight
    if(!itsc.prim) {
      if(scene.envLight) {
        scene.envLight->genRayItsc(itsc, ray, pathVertices.back().itsc.itscVtx.position);
        pathVertices.push_back(PathVertex(itsc, -ray.d, beta, nullptr, scene.envLight));
        tstate = TerminateState::NoBXDF;
      }
      else tstate = TerminateState::NoItsc;
      return;
    }

    if(itsc.cosTheta(ray.d)>0.0f) itsc.reverseNormal();
    const Material& mat = itsc.prim->getMesh()->material;
    
    // if no bxdf, we only allow shape light without bxdf
    // otherwise report error
    if(!mat.bxdfNode) {
      if(mat.light) 
        pathVertices.push_back(PathVertex(itsc, -ray.d, beta, nullptr, mat.light));
      else 
        std::cout<<"WARNING: Detect No BXDF Material(not light)"<<std::endl;
      tstate = TerminateState::NoBXDF;
      return;
    }

    ray.o = itsc.itscVtx.position; 
    ray.d = -ray.d;

    const BXDF* bxdf;
    float bxdfWeight = mat.getBXDF(itsc, ray, bxdf);
    glm::vec3 nbeta = bxdfWeight * bxdf->sample_ev(itsc, ray, sample_ray);

    if(!_IsType(bxdf->getType(), NoSurface)){
      itsc.maxErrorOffset(sample_ray.d, sample_ray.o);
      itsc.itscVtx.position = sample_ray.o;
    }

    // for particles, mediumOutside = medium
    // for surface, for direct light evaluate only use outside medium
    pathVertices.push_back(PathVertex(itsc, ray.d, beta, bxdf, mat.light, mat.mediumOutside));

    if(IsBlack(nbeta)) {tstate=TerminateState::TotalBlack; return;}
    if(!sample_ray.checkDir()) {tstate=TerminateState::CalcERROR; return;}

    beta *= nbeta;
    ray = sample_ray;
    // if is medium particle, it always in medium
    if(_IsType(bxdf->getType(), NoSurface) || !hasMedium) continue;
    inMedium = itsc.isRayToInside(ray)? mat.mediumInside:mat.mediumOutside;

  }
  tstate = TerminateState::UpToMaxBounce;
}

void SubPathGenerator::createSubPath(
  const Ray& start_ray, const Scene& scene, 
  TMode tmode, PathVertex& iniVtx, int max_bounce) {

  Integrator::setTransportMode(tmode);
  if(tmode == TMode::FromCamera) {
    pathVerticesCam.push_back(iniVtx);
    createSubPath(start_ray, scene, pathVerticesCam, tstateCam, max_bounce);
  }
  if(tmode == TMode::FromLight) {
    pathVerticesLt.push_back(iniVtx);
    createSubPath(start_ray, scene, pathVerticesLt, tstateLt, max_bounce);
  }
}

// return tr*Visi/len^2
// also return conn vector(p2->p1)
glm::vec3 geometryTerm(
  const Scene& scene, const PathVertex& pvtx1, 
  const PathVertex& pvtx2, glm::vec3& conn) {
  
  glm::vec3 L(0.0f);
  conn = pvtx1.itsc.itscVtx.position - pvtx2.itsc.itscVtx.position;
  float len = glm::length(conn);
  conn /= len; // normalize it

  if(pvtx2.bxdf) {
    // REFLECT BXDF exitant radiance always on the same side with normal
    if(_IsType(pvtx2.bxdf->getType(), REFLECT) && 
      pvtx2.itsc.cosTheta(conn) < 0) return L; 
    // TR BXDF exitant radiance always on the opposite side with normal
    else if(_IsType(pvtx2.bxdf->getType(), TRANSMISSION) && 
      pvtx2.itsc.cosTheta(conn) > 0) return L; 
  }
  if(pvtx1.bxdf) {
    if(_IsType(pvtx1.bxdf->getType(), REFLECT) && 
      pvtx1.itsc.cosTheta(-conn) < 0) return L; 
    else if(_IsType(pvtx1.bxdf->getType(), TRANSMISSION) && 
      pvtx1.itsc.cosTheta(-conn) > 0) return L; 
  }

  glm::vec3 tr(0.0f); // for volume tr
  Ray testRay{pvtx2.itsc.itscVtx.position, conn};
  bool isOcclude = scene.hasMediumInScene() ? 
    scene.occlude(testRay, len, tr, pvtx2.inMedium, pvtx1.itsc.prim):
    scene.occlude(testRay, len, pvtx1.itsc.prim);
  if(isOcclude) return L;
  return (1.0f/(len*len))*tr;
}

// L = (tr*Visi)*bxdf/pdf*cos_sf*le*cos_lt/r^2
glm::vec3 connectLightAndSurfaceItsc(
  const Scene& scene,
  const PathVertex& pvtx_lt, const PathVertex& pvtx_sf) {
  
  glm::vec3 conn;
  glm::vec3 L = geometryTerm(scene, pvtx_lt, pvtx_sf, conn);
  if(IsBlack(L)) return L;

  Ray testRay{pvtx_sf.itsc.itscVtx.position, conn};
  Ray rayo{pvtx_sf.itsc.itscVtx.position, pvtx_sf.dir_o};

  float leCos = pvtx_lt.itsc.itscVtx.cosTheta(-conn);
  if(leCos <= 0.0f) return L;

  glm::vec3 le = pvtx_lt.light->evaluate(pvtx_lt.itsc, -testRay.d);
  glm::vec3 beta = pvtx_sf.bxdf->evaluate(pvtx_sf.itsc, rayo, testRay);

  return leCos*L*le*beta;
}

// return (tr*Visi/len^2)*bxdf1*cos1/pdf1*bxdf2*cos2/pdf2
glm::vec3 connectSurfaceAndSurfaceItsc(
  const Scene& scene,
  const PathVertex& pvtx1, const PathVertex& pvtx2) {
  
  glm::vec3 conn;
  glm::vec3 L = geometryTerm(scene, pvtx1, pvtx2, conn);
  if(IsBlack(L)) return L;

  Ray testRay2{pvtx2.itsc.itscVtx.position, conn};
  Ray testRay1{pvtx1.itsc.itscVtx.position, -conn};
  Ray rayo2{pvtx2.itsc.itscVtx.position, pvtx2.dir_o};
  Ray rayo1{pvtx1.itsc.itscVtx.position, pvtx1.dir_o};

  glm::vec3 beta2 = pvtx2.bxdf->evaluate(pvtx2.itsc, rayo2, testRay2);
  glm::vec3 beta1 = pvtx1.bxdf->evaluate(pvtx1.itsc, rayo1, testRay1);

  return L*beta1*beta2;
}

// return (tr*Visi)*bxdf*cos/pdf
glm::vec3 connectSurfaceAndCamItsc(
  const Scene& scene,
  const PathVertex& pvtx_sf, const PathVertex& pvtx_cam) {
  
  glm::vec3 conn;
  glm::vec3 L = geometryTerm(scene, pvtx_cam, pvtx_sf, conn);
  if(IsBlack(L)) return L;

  Ray testRay{pvtx_sf.itsc.itscVtx.position, conn};
  Ray rayo{pvtx_sf.itsc.itscVtx.position, pvtx_sf.dir_o};

  glm::vec3 beta = pvtx_sf.bxdf->evaluate(pvtx_sf.itsc, rayo, testRay);

  return L*beta;
}

float pwToPa(const PathVertex& pvtx_from, const PathVertex& pvtx_to, float pw){
  glm::vec3 dir = pvtx_from.itsc.itscVtx.position - 
    pvtx_to.itsc.itscVtx.position;
  float len = glm::length(dir);
  dir /= len;
  float cosTheta = pvtx_to.itsc.itscVtx.cosTheta(dir);
  return PwToPa(pw, len*len, cosTheta);
}

// do not check whether start or end is valid! 
void calcFwdPdf(std::vector<PathVertex>& pvtxs, int start, int end) {
  for(int i = start; i<end; i++) {
    PathVertex& pvtx_pre = pvtxs[i-1];
    Ray rayo{pvtx_pre.itsc.itscVtx.position, pvtx_pre.dir_o};
    Ray rayi{pvtx_pre.itsc.itscVtx.position, -pvtxs[i].dir_o};
    float pdf = pvtx_pre.bxdf->sample_pdf(pvtx_pre.itsc, rayo, rayi);
    if(pdf == 0.0f) pdf = 1.0f;
    pvtxs[i].fwdPdf = pwToPa(pvtx_pre, pvtxs[i], pdf);
  }
}

// do not check whether start or end is valid! 
void calcRevPdf(std::vector<PathVertex>& pvtxs, int start, int end) {
  for(int i = start; i<end; i++) {
    PathVertex& pvtx_n1 = pvtxs[i+1];
    PathVertex& pvtx_n2 = pvtxs[i+2];
    Ray rayo{pvtx_n1.itsc.itscVtx.position, -pvtx_n2.dir_o};
    Ray rayi{pvtx_n1.itsc.itscVtx.position, pvtx_n1.dir_o};
    float pdf = pvtx_n1.bxdf->sample_pdf(pvtx_n1.itsc, rayo, rayi);
    if(pdf == 0.0f) pdf = 1.0f;
    pvtxs[i].revPdf = pwToPa(pvtx_n1, pvtxs[i], pdf);
  }
}

// from->to<-pre
void calcConnPdf(
  const PathVertex& p_from, 
  const PathVertex& p_to,
  const PathVertex& p_to_pre,
  float& toRevPdf, float& toPreRevPdf) {

  glm::vec3 dirToConn = p_to.itsc.itscVtx.position - p_from.itsc.itscVtx.position;
  dirToConn = glm::normalize(dirToConn);

  Ray rayo{p_from.itsc.itscVtx.position, p_from.dir_o};
  Ray rayi{p_from.itsc.itscVtx.position, dirToConn};

  toRevPdf = p_from.bxdf->sample_pdf(p_from.itsc, rayo, rayi);
  toRevPdf = pwToPa(p_from, p_to, toRevPdf);

  Ray rayo2{p_to.itsc.itscVtx.position, -dirToConn};
  Ray rayi2{p_to.itsc.itscVtx.position, p_to.dir_o};

  toPreRevPdf = p_from.bxdf->sample_pdf(p_from.itsc, rayo2, rayi2);
  toPreRevPdf = pwToPa(p_from, p_to, toPreRevPdf);
}

void calcConnPdf(
  const PathVertex& p_from, 
  const PathVertex& p_to,
  float& toRevPdf) {

  glm::vec3 dirToConn = p_to.itsc.itscVtx.position - p_from.itsc.itscVtx.position;
  dirToConn = glm::normalize(dirToConn);

  Ray rayo{p_from.itsc.itscVtx.position, p_from.dir_o};
  Ray rayi{p_from.itsc.itscVtx.position, dirToConn};

  toRevPdf = p_from.bxdf->sample_pdf(p_from.itsc, rayo, rayi);
  toRevPdf = pwToPa(p_from, p_to, toRevPdf);
}

float calcConnPdfFromLt(
  const PathVertex& p_from, 
  const PathVertex& p_to,
  const PathVertex& p_to_pre,
  float& toRevPdf, float& toPreRevPdf) {

  glm::vec3 dirToConn = p_to.itsc.itscVtx.position - p_from.itsc.itscVtx.position;
  dirToConn = glm::normalize(dirToConn);
  toRevPdf = p_from.light->getRayPdf(p_from.itsc, dirToConn);
  toRevPdf = pwToPa(p_from, p_to, toRevPdf);
  
  Ray rayo2{p_to.itsc.itscVtx.position, -dirToConn};
  Ray rayi2{p_to.itsc.itscVtx.position, p_to.dir_o};

  toPreRevPdf = p_from.bxdf->sample_pdf(p_from.itsc, rayo2, rayi2);
  toPreRevPdf = pwToPa(p_from, p_to, toPreRevPdf);
}

float calcConnPdfFromCam(
  const PathVertex& p_from, 
  const PathVertex& p_to,
  const PathVertex& p_to_pre,
  float& toRevPdf, float& toPreRevPdf) {

  glm::vec3 dirToConn = p_to.itsc.itscVtx.position - p_from.itsc.itscVtx.position;
  dirToConn = glm::normalize(dirToConn);
  toRevPdf = 1.0f;
  
  Ray rayo2{p_to.itsc.itscVtx.position, -dirToConn};
  Ray rayi2{p_to.itsc.itscVtx.position, p_to.dir_o};

  toPreRevPdf = p_from.bxdf->sample_pdf(p_from.itsc, rayo2, rayi2);
  toPreRevPdf = pwToPa(p_from, p_to, toPreRevPdf);
}

// 0~end (include end)
void calcHalfMIS(const std::vector<PathVertex>& pvtxs, 
  int end, float revPdf, float& res) {

  float cur = res*(revPdf / pvtxs[end].fwdPdf);
  res += cur;
  for(int i = end-1; i>=0; i++) {
    cur *= pvtxs[i].revPdf/pvtxs[i].fwdPdf;
    res += cur;
  }
}

// make sure that 0~connlt and 0~conncam fwdPdf and revPdf are valid
// connlt and conncam not 0 at the same time
float BDPT_MIS(const std::vector<PathVertex>& pvtxs_lt, 
  const std::vector<PathVertex>& pvtxs_cam, int connlt, int conncam) {
  
  const PathVertex& pclt = pvtxs_lt[connlt];
  const PathVertex& pccm = pvtxs_cam[conncam];
  if(connlt && conncam) {
    float toRevPdf, toPreRevPdf;
    calcConnPdf(pclt, pccm, pvtxs_cam[conncam-1], toRevPdf, toPreRevPdf);
    float res1 = toRevPdf / pccm.fwdPdf;
    calcHalfMIS(pvtxs_cam, conncam-1, toPreRevPdf, res1);

    float fromRevPdf, fromPreRevPdf;
    calcConnPdf(pccm, pclt, pvtxs_lt[connlt-1], fromRevPdf, fromPreRevPdf);
    float res2 = fromRevPdf / pclt.fwdPdf;
    calcHalfMIS(pvtxs_lt, connlt-1, fromPreRevPdf, res2);
    return 1.0f/(1.0f+res1+res2);
  }
  else if(!connlt) {
    float toRevPdf, toPreRevPdf;
    calcConnPdfFromLt(pclt, pccm, pvtxs_cam[conncam-1], toRevPdf, toPreRevPdf);
    float res1 = toRevPdf / pccm.fwdPdf;
    calcHalfMIS(pvtxs_cam, conncam-1, toPreRevPdf, res1);

    float fromRevPdf;
    calcConnPdf(pccm, pclt, fromRevPdf);
    float res2 = fromRevPdf / pclt.fwdPdf;
    return 1.0f/(1.0f+res1+res2);
  }
  else if(!conncam) {
    float toRevPdf;
    calcConnPdf(pclt, pccm, toRevPdf);
    float res1 = toRevPdf / pccm.fwdPdf;

    float fromRevPdf, fromPreRevPdf;
    calcConnPdfFromCam(pccm, pclt, pvtxs_lt[connlt-1], fromRevPdf, fromPreRevPdf);
    float res2 = fromRevPdf / pclt.fwdPdf;
    calcHalfMIS(pvtxs_lt, connlt-1, fromPreRevPdf, res2);
    return 1.0f/(1.0f+res1+res2);
  }
  std::cout<<"Calc MIS ERROR: Invalid return"<<std::endl;
  return 1.0f;
}

// for fromCam path directly hit light, pvtxs[tidx] must have light
float BDPT_MIS(const Scene& scene,
  const std::vector<PathVertex>& pvtxs, int tidx) {
  
  const PathVertex& ltVtx = pvtxs[tidx];
  float ltRevPdf = scene.getLightPdf(ltVtx.light); // pdf for select light
  Ray rayToLight{pvtxs[tidx-1].itsc.itscVtx.position, -ltVtx.dir_o};
  // pdf for select itsc on light
  ltRevPdf *= ltVtx.light->getItscPdf(ltVtx.itsc, rayToLight);

  float rayPdf = ltVtx.light->getRayPdf(ltVtx.itsc, ltVtx.dir_o);
  rayPdf = pwToPa(ltVtx, pvtxs[tidx-1], rayPdf);

  float res = ltVtx.fwdPdf / ltRevPdf;
  calcHalfMIS(pvtxs, tidx - 1, rayPdf, res);
  return 1.0f/(1.0f+res);
}


void BDPTIntegrator::render(
  const Scene& scene, RayGenerator& rayGen, Film& film) const{
  
  PathVertex solidCamVtx;
  solidCamVtx.itsc.itscVtx.position = rayGen.getCamPos();
  solidCamVtx.beta = glm::vec3(1.0f);
  solidCamVtx.fwdPdf = 1.0f;
  const Medium* globalMedium = scene.getGlobalMedium();
  solidCamVtx.inMedium = globalMedium;

  Ray camStartRay, ltStartRay;
  glm::vec2 camRasPos, ltRasPos;

  SubPathGenerator subpathGen;

  std::vector<PathVertex>& psCam = subpathGen.pathVerticesCam;
  std::vector<PathVertex>& psLt = subpathGen.pathVerticesLt;

  while(rayGen.genNextRay(camStartRay, camRasPos)) {
    subpathGen.clear();

    subpathGen.createSubPath(camStartRay, scene, 
      TMode::FromCamera, solidCamVtx, max_sub_path_bounce);

    if(psCam.size() == 1) continue; // nothing intersect

    /***************select light and gen ray***************/
    const Light* light;
    float selectLtPdf = scene.sampleALight(light);
    Intersection itsc_lt; float areaLtPdf, rayLtPdf;
    light->genRay(itsc_lt, ltStartRay, areaLtPdf, rayLtPdf);
    areaLtPdf *= selectLtPdf;
    /**************************************************8/

    /*******generate first light vetex********/
    PathVertex ltVtx;
    ltVtx.itsc = itsc_lt;
    ltVtx.beta = (1.0f/(areaLtPdf*rayLtPdf)) * 
      light->evaluate(itsc_lt, ltStartRay.d);
    ltVtx.fwdPdf = areaLtPdf;
    ltVtx.light = light;
    ltVtx.inMedium = globalMedium;
    /*****************************************/

    subpathGen.createSubPath(ltStartRay, scene, 
      TMode::FromLight, ltVtx, max_sub_path_bounce);

    psCam[1].fwdPdf = 1.0f;
    if(psCam.size() > 2) {
      calcFwdPdf(psCam, 2, psCam.size());
      calcRevPdf(psCam, 0, psCam.size()-2);
    }
    psLt[1].fwdPdf = rayLtPdf;
    if(psLt.size() > 2) {
      calcFwdPdf(psLt, 2, psLt.size());
      calcRevPdf(psLt, 0, psLt.size()-2);
    }

    glm::vec3 L(0.0f);
    for(int i=1; i<psLt.size(); i++) {// for lightpath >1 and campath =1
      if(psLt[i].bxdf && !_HasFeature(psLt[i].bxdf->getType(), DELTA)) {
        glm::vec3 radiance = psCam[0].beta * psLt[i].beta*
          connectSurfaceAndCamItsc(scene, psLt[i], psCam[0]);
        float mis = BDPT_MIS(psLt, psCam, i, 0);
        radiance *= mis;
        ltRasPos = film.camera2raster(psLt[i].itsc.itscVtx.position);
        if(film.isValidRasPos(ltRasPos)) {
          film.addSplat(radiance, ltRasPos);
        }
      }
    }

    for(int i=1; i<psCam.size(); i++) {
      if(psCam[i].light) { // for lightpath = 0 and campath >1
        glm::vec3 radiance = psCam[i].beta *
          psCam[i].light->evaluate(psCam[i].itsc, psCam[i].dir_o);
        float mis = BDPT_MIS(scene, psCam, i);
        L += mis*radiance;
      }
      
      if(psCam[i].bxdf && !_HasFeature(psCam[i].bxdf->getType(), DELTA)) {

        {// for lightpath =1 and campath >1
          glm::vec3 radiance = psLt[0].beta * psCam[i].beta *
            connectLightAndSurfaceItsc(scene, psLt[0], psCam[i]);
          float mis = BDPT_MIS(psLt, psCam, 0, i);
          L += mis*radiance;
        }
        for(int j=1; j<psLt.size(); j++) {// for lightpath >1 and campath >1
          if(psLt[j].bxdf && !_HasFeature(psLt[j].bxdf->getType(), DELTA)) {
            glm::vec3 radiance = psCam[i].beta * psLt[j].beta *
              connectSurfaceAndSurfaceItsc(scene, psCam[i], psLt[j]);
            float mis = BDPT_MIS(psLt, psCam, j, i);
            L += mis*radiance;
          }
        }
      }
    }
    film.addSplat(L, camRasPos);
  }
}