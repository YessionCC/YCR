#pragma once

#include "bxdf.hpp"
#include "light.hpp"
#include <glm/glm.hpp>

class Mesh {
protected:
  enum MeshType {
    VertexMesh,
    CustomMesh
  };
  MeshType type;

public:
  BXDF* bxdf = nullptr;
  Light* light = nullptr; //if null, it is not emissive

  virtual ~Mesh() {delete bxdf; delete light;} //
  Mesh(MeshType type): type(type) {}
  MeshType getType() const {return type;}

  virtual Mesh* copy() const = 0;
  virtual void translate(glm::vec3) = 0;
  virtual void scale(glm::vec3) = 0;
  virtual void rotate(glm::vec3, float) = 0;
  virtual void toPrimitives(
    std::vector<Primitive*>& vp, 
    std::vector<Primitive*>& vl
  ) = 0;
};

class VertexMesh: public Mesh {
private:
  bool needInitVertex = false;

public:
  std::vector<Vertex*> vertices;
  std::vector<uint32_t> indices;

  ~VertexMesh();
  VertexMesh(): Mesh(Mesh::MeshType::VertexMesh){}
  Mesh* copy() const override; // copy Vertex

  void transform(glm::mat4x4 trans);

  void translate(glm::vec3) override;
  void scale(glm::vec3) override;
  void rotate(glm::vec3, float) override;

  // vp: all prims, vl: light(emission) prims
  void toPrimitives(
    std::vector<Primitive*>& vp, 
    std::vector<Primitive*>& vl
  ) override;

  static VertexMesh* CreateTriangle(glm::vec3 pos[3]);
  static VertexMesh* CreateRectangle(glm::vec3 pos[4]);
};

class CustomMesh: public Mesh {
public:
  std::vector<Primitive*> prims;

  ~CustomMesh();
  CustomMesh(): Mesh(Mesh::MeshType::CustomMesh){}
  Mesh* copy() const override; // copy Vertex

  void translate(glm::vec3) override;
  void scale(glm::vec3) override;
  void rotate(glm::vec3, float) override;

  // vp: all prims, vl: light(emission) prims
  void toPrimitives(
    std::vector<Primitive*>& vp, 
    std::vector<Primitive*>& vl
  ) override;

  static CustomMesh* CreateSphere(glm::vec3 center, float radius);
};