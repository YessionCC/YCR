#include "mesh.hpp"
#include "primitive.hpp"

#include <glm/gtc/matrix_transform.hpp>

VertexMesh::~VertexMesh() {
  for(Vertex* v : vertices) 
    delete v;
}

// will copy vertex data but will share light and bxdf
Mesh* VertexMesh::copy() const {
  VertexMesh* mesh = new VertexMesh;
  mesh->type = type;
  mesh->purpose = purpose;

  mesh->indices = indices;

  mesh->light = light;
  mesh->mediumInside = mediumInside;
  mesh->mediumOutside = mediumOutside;
  mesh->normalMap = normalMap;
  mesh->bxdf = bxdf;

  for(Vertex* v : vertices) 
    mesh->vertices.push_back(new Vertex(*v));
  return mesh;
}

void VertexMesh::transform(glm::mat4x4 trans) {
  glm::mat3x3 nTrans = glm::transpose(glm::inverse(glm::mat3x3(trans)));
  for(Vertex* vtx: vertices){
    vtx->transform(trans, nTrans);
  }
}

void VertexMesh::translate(glm::vec3 move) {
  glm::mat4x4 trans(1.0f);
  trans = glm::translate(trans, move);
  transform(trans);
};
void VertexMesh::scale(glm::vec3 rate) {
  glm::mat4x4 trans(1.0f);
  trans = glm::scale(trans, rate);
  transform(trans);
};
void VertexMesh::rotate(glm::vec3 axis, float angle) {
  glm::mat4x4 trans(1.0f);
  trans = glm::rotate(trans, glm::radians(angle), axis);
  transform(trans);
};

// need to release ptr explicitly
void VertexMesh::toPrimitives(std::vector<const Primitive*>& vp) {
  for(unsigned int i=0; i<indices.size(); i+=3) {
    int i1 = indices[i], i2 = indices[i+1], i3 = indices[i+2];
    Triangle* prim = 
      new Triangle(vertices[i1], vertices[i2], vertices[i3], this);
    if(needInitVertex) prim->autoCalcParams();
    vp.push_back(prim);
  }
}

VertexMesh* VertexMesh::CreateTriangle(glm::vec3 pos[3]) {
  VertexMesh* mesh = new VertexMesh;
  Vertex* vtx = new Vertex(pos[0]);
  vtx->uv = {0, 0};
  mesh->vertices.push_back(vtx);
  vtx = new Vertex(pos[1]);
  vtx->uv = {1, 0};
  mesh->vertices.push_back(vtx);
  vtx = new Vertex(pos[2]);
  vtx->uv = {0, 1};
  mesh->vertices.push_back(vtx);
  mesh->indices = {0,1,2};
  mesh->needInitVertex = true;
  return mesh;
}

VertexMesh* VertexMesh::CreateRectangle(glm::vec3 pos[4]) {
  VertexMesh* mesh = new VertexMesh;
  Vertex* vtx = new Vertex(pos[0]);
  vtx->uv = {0, 0};
  mesh->vertices.push_back(vtx);
  vtx = new Vertex(pos[1]);
  vtx->uv = {1, 0};
  mesh->vertices.push_back(vtx);
  vtx = new Vertex(pos[2]);
  vtx->uv = {1, 1};
  mesh->vertices.push_back(vtx);
  vtx = new Vertex(pos[3]);
  vtx->uv = {0, 1};
  mesh->vertices.push_back(vtx);
  mesh->indices = {0,1,2,0,2,3};
  mesh->needInitVertex = true;
  return mesh;
}

CustomMesh::~CustomMesh() {
  for(Primitive* prim: prims)
    delete prim;
}

// will copy vertex data but will share light and bxdf
Mesh* CustomMesh::copy() const {
  CustomMesh* mesh = new CustomMesh;
  mesh->type = type;
  mesh->purpose = purpose;

  mesh->light = light;
  mesh->mediumInside = mediumInside;
  mesh->mediumOutside = mediumOutside;
  mesh->normalMap = normalMap;
  mesh->bxdf = bxdf;
  
  for(Primitive* prim: prims)
    mesh->prims.push_back(prim->copy(mesh));
  return mesh;
}

void CustomMesh::translate(glm::vec3 tran) {
  for(Primitive* prim: prims)
    prim->translate(tran);
}
void CustomMesh::scale(glm::vec3 scl) {
  for(Primitive* prim: prims)
    prim->scale(scl);
}
void CustomMesh::rotate(glm::vec3 axis, float rot) {
  for(Primitive* prim: prims)
    prim->rotate(axis, rot);
}

// vp: all prims, vl: light(emission) prims
void CustomMesh::toPrimitives(std::vector<const Primitive*>& vp) {
  for(Primitive* prim: prims) {
    vp.push_back(prim);
  }
}

CustomMesh* CustomMesh::CreateSphere(glm::vec3 center, float radius){
  CustomMesh* mesh = new CustomMesh;
  mesh->prims.push_back(new Sphere(center, radius, mesh));
  return mesh;
}

CustomMesh* CustomMesh::CreatePoint(glm::vec3 pos) {
  CustomMesh* mesh = new CustomMesh;
  mesh->prims.push_back(new PointPrim(pos, mesh));
  return mesh;
}