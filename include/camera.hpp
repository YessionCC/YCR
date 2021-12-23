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
  }

  void generateRay(Ray& ray, glm::vec2 rasterPos) const{
    glm::vec2 camPos = film.raster2camera(rasterPos);
    glm::vec3 camdir = glm::normalize(glm::vec3(camPos, -1.0f));
    ray.o = position;
    ray.d = cam2world_rot*camdir;
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
private:
  const Camera& camera;
  StratifiedSampler2D sp2d;
  Block2D curRenderBlock;
  int spp, cntx, cnty, cntspp;

public:
  //spp must a squre number
  RayGenerator(const Camera& cam, int _spp):
    camera(cam), sp2d(glm::sqrt(_spp)), 
    curRenderBlock({cam.getReX(), cam.getReY(), 0, 0}),
    spp(_spp), cntx(0), cnty(0), cntspp(0){}

  void reset(const Block2D renderBlock) {
    curRenderBlock = renderBlock;
    cntx = cnty = cntspp = 0;
    sp2d.clear();
  }

  inline glm::vec3 getCamPos() const {return camera.getPosition();}

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
    浮点数加法对阶（小阶对大阶）时右移可能发生进位（相当于+1）。
    减去一个很小的浮点数也可能因为对阶的舍入（相当于-0）没用。
    使用std::nextafter能够得到最近的大于或小于某个数的浮点数，是最好的选择。
    这一部分非常重要，用中文写明
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
