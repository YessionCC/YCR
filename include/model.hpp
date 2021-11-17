#pragma once

#include <string>
#include <vector>
#include <ctype.h>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "vertex.hpp"
#include "primitive.hpp"
#include "bxdf.hpp"
#include "light.hpp"
#include "mesh.hpp"
#include "medium.hpp"

class Model {
private:
  bool isInvalidModel = true;
  std::string modelDirectory;
  std::vector<Mesh*> meshes;

  void processNode(aiNode *node, const aiScene *scene);
  Mesh* processMesh(aiMesh *mesh, const aiScene *scene);

public:
  ~Model();
  Model() {}
  Model(const char* filename);
  Model(Mesh* mesh);
  Model(const Model& model);
  Model(Model && model) = delete;
  const Model& operator=(const Model& model) = delete;

  Model* copy();
  bool loadFromFile(const char* filename);
  void toPrimitives(std::vector<Primitive*>& vp);
  void translate(glm::vec3 translate);
  void scale(glm::vec3 scale);
  void rotate(glm::vec3 axis, float angle);
  void setBxdfForAllMeshes(BXDF* bxdf);
  void setLightForAllMeshes(Light* light);
  void setMediumForAllMeshes(Medium* medium);
  void addMesh(Mesh* mesh);
  bool hasMesh(Mesh* mesh) const;

  void toPointClouds(PCShower& pc, int tot_pcn, glm::vec3 col);
};