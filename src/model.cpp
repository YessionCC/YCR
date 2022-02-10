#include "model.hpp"
#include "vertex.hpp"
#include "distribution.hpp"
#include "medium.hpp"
#include "texture.hpp"

#include "sceneImporter.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <iostream>

Model::Model(const char* filename) {
  isInvalidModel = SceneImporter::loadSceneFromFile(filename, meshes);
}

Model::Model(Mesh* mesh) {
  meshes.push_back(mesh);
  modelDirectory = "run time generate";
}

// copy construction will copy primitives
Model::Model(const Model& model) {
  isInvalidModel = model.isInvalidModel;
  modelDirectory = model.modelDirectory;
  for(Mesh* mesh: model.meshes) {
    meshes.push_back(mesh->copy());
  }
}

Model::~Model() {
  for(Mesh* mesh : meshes)
    delete mesh;
}

Model* Model::copy() {
  Model* model = new Model;
  for(Mesh* mesh : meshes)
    model->meshes.push_back(mesh->copy());
  return model;
}

// need to release ptr explicitly
void Model::toPrimitives(std::vector<const Primitive*>& vp) const {
  for(Mesh* mesh : meshes)
    mesh->toPrimitives(vp);
}

void Model::addMesh(Mesh* mesh) {
  meshes.push_back(mesh);
}

void Model::translate(glm::vec3 translate) {
  for(Mesh* mesh: meshes) {
    mesh->translate(translate);
  }
}

void Model::scale(glm::vec3 scale) {
  for(Mesh* mesh: meshes) {
    mesh->scale(scale);
  }
}

void Model::rotate(glm::vec3 axis, float angle) {
  for(Mesh* mesh: meshes) {
    mesh->rotate(axis, angle);
  }
}

void Model::setBxdfForAllMeshes(const BXDFNode* bxdfNode) {
  for(Mesh* mesh: meshes) mesh->material.bxdfNode = bxdfNode;
}

void Model::setBxdfForOneMesh(const BXDFNode* bxdfNode, int index) {
  meshes.at(index)->material.bxdfNode = bxdfNode;
}

void Model::setBxdfForMeshes(const BXDFNode* bxdfNode, std::vector<int> idxs) {
  for(int idx: idxs)
    meshes.at(idx)->material.bxdfNode = bxdfNode;
}

void Model::setBxdfForRangeMeshes(const BXDFNode* bxdfNode, int s, int e) {
  for(int i = s; i<=e; i++)
    meshes.at(i)->material.bxdfNode = bxdfNode;
}

void Model::setLightForAllMeshes(const Light* light) {
  for(Mesh* mesh: meshes) mesh->material.light = light;
}

// set Medeium(default inside medium)
void Model::setMediumForAllMeshes(const Medium* medium, bool isInside) {
  if(isInside) for(Mesh* mesh: meshes) mesh->material.mediumInside = medium;
  else for(Mesh* mesh: meshes) mesh->material.mediumOutside = medium;
}

void Model::setNormalMapForAllMeshes(const Texture* tex) {
  for(Mesh* mesh: meshes) mesh->material.normalMap = tex;
}

void Model::setMaterialForAllMeshes(const Material& mat) {
  for(Mesh* mesh: meshes) mesh->material = mat;
}

void Model::setMeshPurposes(Mesh::MeshPurpose purpose) {
  for(Mesh* mesh: meshes) mesh->purpose = purpose;
}

void Model::toPointClouds(PCShower& pc, int tot_pcn, glm::vec3 col) {
  std::vector<const Primitive*> vp;
  toPrimitives(vp);
  DiscreteDistribution1D dt;
  for(const Primitive* p: vp) {
    dt.addPdf(p->getArea());
  }
  dt.calcCdf();
  float pdf; int idx;
  for(int i = 0; i<tot_pcn; i++) {
    idx = dt.sample(pdf);
    vp[idx]->genPrimPointCloud(pc, col);
  }
}