#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "film.hpp"
#include "ray.hpp"
#include "sampler.hpp"

#include "debug/pcshow.hpp"

class Camera {
private:
  glm::mat4x4 cam2world, world2cam;

  glm::vec3 position;
  glm::mat3x3 cam2world_rot;
  glm::mat3x3 world2cam_rot;

  const Film& film;

public:
  Camera(const Film& film,
        glm::vec3 position, 
        glm:: vec3 lookdir, 
        glm::vec3 up={0,1,0}): film(film) {
    // notice camera look at -z axis
    // -z point into the screen, x to right, y to top
    world2cam = glm::lookAt(position, position+lookdir, up);
    cam2world = glm::inverse(world2cam);

    this->position = position;
    this->cam2world_rot = glm::mat3x3(cam2world);
    this->world2cam_rot = glm::mat3x3(world2cam);
  }

  void generateRay(Ray& ray, glm::vec2 rasterPos) const{
    glm::vec2 camPos = film.raster2camera(rasterPos);
    glm::vec3 camdir = glm::normalize(glm::vec3(camPos, -1.0f));
    ray.o = position;
    ray.d = cam2world_rot*camdir;
  }

  inline glm::vec2 world2raster(glm::vec3 wp) const {
    glm::vec3 camp = world2cam_rot*(wp - position);
    return film.camera2raster(camp);
  }

  inline glm::vec3 getPosition() const {return position;}

  void visualizePointCloud(PCShower& pc, float distance, int pcn) {
    Ray r1, r2, r3, r4;
    generateRay(r1, {0, 0});
    generateRay(r2, {film.getReX(), 0});
    generateRay(r3, {0, film.getReY()});
    generateRay(r4, {film.getReX(), film.getReY()});
    r1.saveSegLineAsPointCloud(pc, distance, glm::vec3(1.0f), pcn);
    r2.saveSegLineAsPointCloud(pc, distance, glm::vec3(1.0f), pcn);
    r3.saveSegLineAsPointCloud(pc, distance, glm::vec3(1.0f), pcn);
    r4.saveSegLineAsPointCloud(pc, distance, glm::vec3(1.0f), pcn);
  }

  inline int getReX() const {return film.getReX();}
  inline int getReY() const {return film.getReY();}
};

struct Block2D {
  int width, height, offsetX, offsetY;
};
class RayGenerator {
protected:
  const Camera& camera;
  Block2D curRenderBlock;
public:
  RayGenerator(const Camera& cam): camera(cam),
    curRenderBlock({cam.getReX(), cam.getReY(), 0, 0}) {}
  ~RayGenerator() {}
  virtual bool genNextRay(Ray& ray, glm::vec2& rasterPos) = 0;

  inline glm::vec3 getCamPos() const {return camera.getPosition();}
  inline glm::vec2 world2raster(glm::vec3 wp) const {return camera.world2raster(wp);}
};

class StractifiedRGen:public RayGenerator {
private:
  StratifiedSampler2D sp2d;
  int spp, cntx, cnty, cntspp;

public:
  //spp must a squre number
  StractifiedRGen(const Camera& cam, int _spp): 
    RayGenerator(cam), sp2d(glm::sqrt(_spp)), 
    spp(_spp), cntx(0), cnty(0), cntspp(0){}

  void reset(const Block2D renderBlock) {
    curRenderBlock = renderBlock;
    cntx = cnty = cntspp = 0;
    sp2d.clear();
  }

  bool genNextRay(Ray& ray, glm::vec2& rasterPos) {
    if(cntx >= curRenderBlock.width || 
       cnty >= curRenderBlock.height) 
       return false;
    glm::vec2 offset(
      cntx+curRenderBlock.offsetX,
      cnty+curRenderBlock.offsetY
    );
    rasterPos = sp2d.get2()+offset;
    
    // !! NOTICE: just like 300.0f + 0.99999f == 301.0f
    /*
    ?????????????????????????????????????????????????????????????????????????????????+1??????
    ????????????????????????????????????????????????????????????????????????-0????????????
    ??????std::nextafter?????????????????????????????????????????????????????????????????????????????????
    ??????????????????????????????????????????
    */
    if(rasterPos.x >= offset.x+1) 
      rasterPos.x=std::nextafter(rasterPos.x, rasterPos.x-1);
    if(rasterPos.y >= offset.y+1) 
      rasterPos.y=std::nextafter(rasterPos.y, rasterPos.y-1);

    camera.generateRay(ray, rasterPos);
    cntspp++;
    if(cntspp>=spp) {
      cntspp = 0;
      cntx++;
      if(cntx>=curRenderBlock.width) {
        cntx = 0;
        cnty++;
      }
    }
    return true;
  }
};


class HaltonRGen: public RayGenerator {
private:
  int cntx, cnty;
  glm::vec2 hspRasPos;
  HaltonSampler2D hsp2d;
public:
  HaltonRGen(const Camera& cam): RayGenerator(cam), cntx(0), cnty(0){}
  
  bool genNextRay(Ray& ray, glm::vec2& rasterPos) {
    if(cntx >= curRenderBlock.width || 
       cnty >= curRenderBlock.height) return false;

    int offx = cntx+curRenderBlock.offsetX;
    int offy = cnty+curRenderBlock.offsetY;
    rasterPos = hspRasPos+glm::vec2(offx, offy);
    if(rasterPos.x >= offx+1) 
      rasterPos.x=std::nextafter(rasterPos.x, rasterPos.x-1);
    if(rasterPos.y >= offy+1) 
      rasterPos.y=std::nextafter(rasterPos.y, rasterPos.y-1);

    camera.generateRay(ray, rasterPos);
    cntx++;
    if(cntx>=curRenderBlock.width) {
      cntx = 0;
      cnty++;
    }
    return true;
  }

  void reset(int index) {
    cntx = cnty = 0;
    hspRasPos = hsp2d.get2(index);
  }
};
