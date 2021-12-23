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




void PathIntegrator::render(
  const Scene& scene, RayGenerator& rayGen, Film& film) const {
  
  Ray startRay; glm::vec2 rasPos;
  bool hasMedium = scene.hasMediumInScene();
  
  while(rayGen.genNextRay(startRay, rasPos)) {
    // if((int)rasPos.x == 245 && (int)rasPos.y == 625) {
    //   int debug = 1;
    // }

    glm::vec3 beta(1.0f), L(0.0f);
    Intersection itsc;
    Ray ray = startRay, sampleRay;

    int lastBType = BType::DELTA;
    DiscreteDistribution1D ldd1d;

    const Medium* inMedium = scene.getGlobalMedium();

    for(int bounce = 0; bounce<max_bounce; bounce++) {
      itsc = scene.intersect(ray, nullptr);
      if(inMedium)
        beta *= inMedium->sampleNextItsc(ray, itsc);
      
      if(!itsc.prim) {
        if(scene.envLight && _HasFeature(lastBType, DELTA)) {
          L += scene.envLight->evaluate(itsc, -ray.d)*beta;
        }
        break;
      }

      const Material& mat = itsc.prim->getMesh()->material;
      if(itsc.cosTheta(ray.d)>0.0f) itsc.reverseNormal();

      if(mat.light) {
        if(!itsc.normalReverse && _HasFeature(lastBType, DELTA)) {
          L += mat.light->evaluate(itsc, -ray.d)*beta;
        }
        if(!mat.bxdfNode) break;
      }
      if(!mat.bxdfNode) {
        std::cout<<"WARNING: Detect No BXDF Material(not light)"<<std::endl;
        break;
      }

      ray.o = itsc.itscVtx.position; 
      ray.d = -ray.d;

      const BXDF* bxdf;
      float bxdfWeight = mat.getBXDF(itsc, ray, bxdf);

      glm::vec3 nBeta = bxdfWeight * bxdf->sample_ev(itsc, ray, sampleRay);

      if(IsBlack(nBeta)) break;
      if(!sampleRay.checkDir()) break;

      lastBType = bxdf->getType();

      if(!_IsType(lastBType, NoSurface)){
        itsc.maxErrorOffset(sampleRay.d, sampleRay.o);
        itsc.itscVtx.position = sampleRay.o;
      }

      /**********estimate direct light*************/
      if(!_HasFeature(lastBType, DELTA)) {
        glm::vec3 light_L;
        scene.getPositionLightDD1D(itsc.itscVtx.position, ldd1d);
        estimateDirectLightByLi(
          scene, ldd1d, itsc, bxdf, mat.mediumOutside, ray, light_L, false);
        L += beta*light_L;
      }
      /********************************************/
      
      beta *= nBeta;
      ray = sampleRay;
      if(!_IsType(lastBType, NoSurface)) 
        inMedium = itsc.isRayToInside(ray)? mat.mediumInside:mat.mediumOutside;
    }
    film.addSplat(L, rasPos);
  }
}