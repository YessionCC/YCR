#pragma once

#include "vertex.hpp"
#include "bb3.hpp"
#include "ray.hpp"
#include "itsc.hpp"
#include "sampler.hpp"

#include "debug/pcshow.hpp"

class Mesh;

class Primitive {
protected:
  Mesh* mesh;

public:
  virtual ~Primitive() {}
  Primitive() {}
  Primitive(Mesh* mesh): mesh(mesh){}

  virtual Primitive* copy(Mesh* mesh) const = 0;

  virtual BB3 getBB3() const = 0;

  virtual void translate(glm::vec3) = 0;
  virtual void scale(glm::vec3) = 0;
  virtual void rotate(glm::vec3, float) = 0;

  virtual inline Mesh* getMesh() const {return mesh;}

  virtual inline glm::vec3 getCenter() const = 0;

  virtual inline float getArea() const = 0;

  //virtual void transform(glm::mat4x4) = 0;

  // return pdf
  virtual float getAPointOnSurface(Intersection& itsc) = 0;

  // notice: itsc as in-out variable
  virtual void intersect(const Ray& ray, Intersection& itsc) = 0;

  // just return whether intersect or not
  virtual bool intersectTest(const Ray& ray) const = 0;

  //set normal, uv, and other for itsc
  virtual void handleItscResult(Intersection& itsc) const = 0;

  virtual void genPrimPointCloud(PCShower& pc, glm::vec3 col) const = 0;
};

class Triangle: public Primitive { //Triangle
private:
  Vertex* verts[3];

public:
  Triangle() {}
  Triangle(Vertex* v1, Vertex* v2, Vertex* v3, Mesh* mesh);

  BB3 getBB3() const;

  Primitive* copy(Mesh* mesh) const;

  inline Mesh* getMesh() const {return mesh;}

  inline glm::vec3 getCenter() const {
    return (verts[0]->position+verts[1]->position+verts[2]->position)/3.0f;
  }

  inline float getArea() const {
    glm::vec3 v01 = verts[1]->position - verts[0]->position;
    glm::vec3 v02 = verts[2]->position - verts[0]->position;
    return 0.5f*glm::length(glm::cross(v01, v02));
  }

  void translate(glm::vec3);
  void scale(glm::vec3);
  void rotate(glm::vec3, float);

  // return pdf
  float getAPointOnSurface(Intersection& itsc);

  // notice: itsc as in-able
  void intersect(const Ray& ray, Intersection& itsc);

  // just return whether intersect or not
  bool intersectTest(const Ray& ray) const;

  //set normal, uv, and other for itsc
  void handleItscResult(Intersection& itsc) const;
  
  // calc normal, tangent and others by position
  // normal point to right-hand 012
  // uv: 00, 10, 01, tangent: v01
  void autoCalcParams(bool reverseNormal = false);

  void genPrimPointCloud(PCShower& pc, glm::vec3 col) const;

};

class Sphere: public Primitive { // UV default point to up
private:
  glm::vec3 center;
  glm::vec3 up, right, forward;
  float radius;

  void calcItscInfoFromNormal(Intersection& itsc, glm::vec3 normal) const;

public:
  Sphere() {}
  Sphere(glm::vec3 center, float radius, Mesh* mesh): 
    Primitive(mesh), center(center),
    up(0,1,0), right(1,0,0), forward(0,0,1), radius(radius){}

  BB3 getBB3() const {
    return BB3(center-glm::vec3(radius), center+glm::vec3(radius));
  }

  Primitive* copy(Mesh* mesh) const;

  inline Mesh* getMesh() const {return mesh;}

  inline glm::vec3 getCenter() const {
    return center;
  }

  inline float getArea() const {
    return PI4*radius*radius;
  }

  void translate(glm::vec3);
  void scale(glm::vec3);
  void rotate(glm::vec3, float);

  // return pdf
  float getAPointOnSurface(Intersection& itsc);

  // notice: itsc as in-out variable
  void intersect(const Ray& ray, Intersection& itsc);

  // just return whether intersect or not
  bool intersectTest(const Ray& ray) const;

  //set normal, uv, and other for itsc
  void handleItscResult(Intersection& itsc) const;

  glm::vec3 uniSampleSphereDir() const;

  void genPrimPointCloud(PCShower& pc, glm::vec3 col) const;

};