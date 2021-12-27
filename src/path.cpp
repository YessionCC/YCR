#include "path.hpp"
#include "medium.hpp"
#include "utility.hpp"

using BType = BXDF::BXDFNature;

void estimateDirectLightByLi(
  const Scene& scene,  const DiscreteDistribution1D& ldd1d,
  const Intersection itsc, const BXDF* bxdf, const Medium* inMedium,
  const Ray& rayo, glm::vec3& L, bool needMIS) {

  Ray rayToLight;
  Intersection itsc_lt; const Light* lt; float len;
  glm::vec3 tr(1.0f); L = glm::vec3(0.0f);

  float lpdf_A = scene.dynamicSampleALight(ldd1d, lt);
  lpdf_A *= lt->getItscOnLight(itsc_lt, itsc.itscVtx.position);
  // TODO
  glm::vec3 dirToLight = itsc_lt.itscVtx.position - 
    itsc.itscVtx.position;
  len = glm::length(dirToLight);
  dirToLight /= len;

  float lCosTheta = itsc_lt.itscVtx.cosTheta(-dirToLight);
  if(lCosTheta <= 0.0f) return;

  // REFLECT BXDF exitant radiance always on the same side with normal
  if(_IsType(bxdf->getType(), REFLECT) && 
    itsc.cosTheta(dirToLight) < 0) return; 
  // TR BXDF exitant radiance always on the opposite side with normal
  else if(_IsType(bxdf->getType(), TRANSMISSION) && 
    itsc.cosTheta(dirToLight) > 0) return; 

  rayToLight.o = itsc.itscVtx.position;
  rayToLight.d = dirToLight;

  //if(scene.occlude(rayToLight, len, itsc_lt.prim)) return; 
  if(scene.occlude(rayToLight, len, tr, inMedium, itsc_lt.prim)) 
    return; 
  //if(scene.occlude(itsc, itsc_lt, rayToLight, len)) return; 

  float invdis2 = 1.0f/(len*len);
  glm::vec3 leCosDivR2 = 
    invdis2*lCosTheta*lt->evaluate(itsc_lt, -rayToLight.d);

  glm::vec3 lastBeta = bxdf->evaluate(itsc, rayo, rayToLight);
  // debug error detect
  L = tr*leCosDivR2*lastBeta / lpdf_A;

  if(needMIS) {
    float lpdf_S = PaToPw(lpdf_A, len*len, lCosTheta);
    float pdfBxdf = bxdf->sample_pdf(itsc, rayToLight, rayo);
    L *= PowerHeuristicWeight(lpdf_S, pdfBxdf);
  }
}

void estimateDirectLightByBXDF(
  const Scene& scene, const DiscreteDistribution1D& ldd1d,
  const Intersection itsc, const BXDF* bxdf, const Medium* inMedium,
  const Ray& rayo, glm::vec3& L, bool needMIS) {

  Ray rayToLight;
  glm::vec3 tr(1.0f); L = glm::vec3(0.0f);

  glm::vec3 beta = bxdf->sample_ev(itsc, rayo, rayToLight);
  if(IsBlack(beta)) return;

  Intersection itsc_sp = scene.intersectDirectly(rayToLight, inMedium, tr);

  const Light* lt = nullptr; 
  if(itsc_sp.prim) {
    lt = itsc_sp.prim->getMesh()->material.light;
    if(lt) {
      if(itsc_sp.normalReverse) return;
      L = tr*beta*lt->evaluate(itsc_sp, -rayToLight.d);
    }
    else return;
  }
  else {
    if(scene.envLight) {// TODO: currently we only need to take envLight into account
      lt = scene.envLight;
      scene.envLight->genRayItsc(itsc_sp, rayToLight, itsc.itscVtx.position);
      L = tr*beta*scene.envLight->evaluate(itsc_sp, -rayToLight.d);
    }
    else return;
  }

  if(needMIS) {
    float bxdfPdf = bxdf->sample_pdf(itsc, rayToLight, rayo);
    float lpdf_A = scene.getLightPdf(ldd1d, lt)*lt->getItscPdf(itsc_sp, rayToLight);
    glm::vec3 dir = itsc_sp.itscVtx.position - itsc.itscVtx.position;
    float dist2 = dir.x*dir.x+dir.y*dir.y+dir.z*dir.z;
    float cosTheta = itsc_sp.itscVtx.cosTheta(-rayToLight.d);
    float lpdf_S = PaToPw(lpdf_A, dist2, cosTheta); // from area to solid angle;
    L *= PowerHeuristicWeight(bxdfPdf, lpdf_S);
  }
}

// rayToLight.o is at pre itsc.position and rayToLight.d point to light
float estimateDirectLightByBXDF(
  const Scene& scene, const DiscreteDistribution1D& ldd1d,
  const Intersection& itsc_lt, const Ray& rayToLight, 
  const Light* lt, float sample_pdfw) {

  float len2 = dist2(itsc_lt.itscVtx.position - rayToLight.o);
  float cosTheta = itsc_lt.itscVtx.cosTheta(-rayToLight.d);

  float lpdf_A = scene.getLightPdf(ldd1d, lt)*lt->getItscPdf(itsc_lt, rayToLight);
  float lpdf_S = PaToPw(lpdf_A, len2, cosTheta);
  return PowerHeuristicWeight(sample_pdfw, lpdf_S);
}


void PathIntegrator::render(
  const Scene& scene, RayGenerator& rayGen, Film& film) const {
  
  Ray startRay; glm::vec2 rasPos;
  bool hasMedium = scene.hasMediumInScene();
  //long long int bounce_cnt = 0;
  
  while(rayGen.genNextRay(startRay, rasPos)) {
    if((int)rasPos.x == 238 && (int)rasPos.y == 494) {
      int debug = 1;
    }

    glm::vec3 beta(1.0f), L(0.0f);
    Intersection itsc;
    Ray ray = startRay, sampleRay;

    int lastBType = BType::DELTA;
    DiscreteDistribution1D ldd1d;

    bool needMIS = false; 
    const BXDF* bxdf = nullptr; 

    float sample_pdfw = 0.0f; // for mis

    const Medium* inMedium = scene.getGlobalMedium();

    for(int bounce = 0; bounce<max_bounce; bounce++) {
      // bounce_cnt ++;
      // if(bounce_cnt == 4134921) {
      //   int debug = 2;
      // }

      itsc = scene.intersect(ray, nullptr);
      if(inMedium)
        beta *= inMedium->sampleNextItsc(ray, itsc);

      
      if(!itsc.prim) {
        if(scene.envLight && _HasFeature(lastBType, DELTA)) {
          L += scene.envLight->evaluate(itsc, -ray.d)*beta;
        }
        if(scene.envLight && needMIS && !hasMedium && _Connectable(lastBType)) {
          Intersection itsc_lt;
          scene.envLight->genRayItsc(itsc_lt, ray, ray.o);
          float mis = estimateDirectLightByBXDF(
            scene, ldd1d, itsc_lt, ray, scene.envLight, sample_pdfw);
          L += mis*scene.envLight->evaluate(itsc, -ray.d)*beta;
        }
        break;
      }

      const Material& mat = itsc.prim->getMesh()->material;
      if(itsc.prim->hasSurface() && itsc.cosTheta(ray.d)>0.0f) itsc.reverseNormal();

      if(mat.light) {
        if(!itsc.normalReverse && _HasFeature(lastBType, DELTA)) {
          L += mat.light->evaluate(itsc, -ray.d)*beta;
        }
        if(!itsc.normalReverse && needMIS && 
          !hasMedium && _Connectable(lastBType)) {
            
          float mis = estimateDirectLightByBXDF(
            scene, ldd1d, itsc, ray, mat.light, sample_pdfw);
          L += mis*mat.light->evaluate(itsc, -ray.d)*beta;
        }
        if(!mat.bxdfNode) break;
      }
      if(!mat.bxdfNode) {
        std::cout<<"WARNING: Detect No BXDF Material(not light)"<<std::endl;
        break;
      }

      ray.o = itsc.itscVtx.position; 
      ray.d = -ray.d;

      float bxdfWeight = mat.getBXDF(itsc, ray, bxdf); // bxdf update here

      glm::vec3 nBeta = bxdfWeight * bxdf->sample_ev(itsc, ray, sampleRay);

      if(IsBlack(nBeta)) break;
      if(!sampleRay.checkDir()) break;

      lastBType = bxdf->getType();

      if(!_IsType(lastBType, NoSurface)){
        itsc.maxErrorOffset(sampleRay.d, sampleRay.o);
        itsc.itscVtx.position = sampleRay.o;
      }

      /**********estimate direct light and useMIS*************/
      if(_Connectable(lastBType)) {
        glm::vec3 light_L;
        needMIS = useMIS && bxdf->needMIS(itsc);

        scene.getPositionLightDD1D(itsc.itscVtx.position, ldd1d);
        estimateDirectLightByLi(
          scene, ldd1d, itsc, bxdf, mat.mediumOutside, ray, light_L, needMIS);
        CheckRadiance(light_L, rasPos);
        L += beta*light_L;

        if(needMIS && hasMedium) {
          glm::vec3 BXDF_L;
          estimateDirectLightByBXDF(
            scene, ldd1d, itsc, bxdf, mat.mediumOutside, ray, BXDF_L, useMIS);
          CheckRadiance(BXDF_L, rasPos);
          L += beta*BXDF_L;
        }
        if(needMIS && !hasMedium) {//
          sample_pdfw = bxdf->sample_pdf(itsc, ray, sampleRay);
        }
      }
      /********************************************/
      
      beta *= nBeta;
      ray = sampleRay;
      if(!_IsType(lastBType, NoSurface)) 
        inMedium = itsc.isRayToInside(ray)? mat.mediumInside:mat.mediumOutside;
    }
    CheckRadiance(L, rasPos);
    film.addSplat(L, rasPos);
  }
}