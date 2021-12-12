#pragma once

#include <string>
#include <vector>
#include <ctype.h>

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

public:
  ~Model();
  Model() {}
  Model(const char* filename);
  Model(Mesh* mesh);
  Model(const Model& model);
  Model(Model && model) = delete;
  const Model& operator=(const Model& model) = delete;

  Model* copy();
  void toPrimitives(std::vector<const Primitive*>& vp) const;
  void translate(glm::vec3 translate);
  void scale(glm::vec3 scale);
  void rotate(glm::vec3 axis, float angle);
  void addMesh(Mesh* mesh);

  void setBxdfForAllMeshes(const BXDFNode* bxdfNode);
  void setLightForAllMeshes(const Light* light);
  void setMediumForAllMeshes(const Medium* medium, bool isInside = true);
  void setNormalMapForAllMeshes(const Texture* tex);

  void setMaterialForAllMeshes(const Material& mat);  

  void setMeshPurposes(Mesh::MeshPurpose purpose);

  void toPointClouds(PCShower& pc, int tot_pcn, glm::vec3 col);
};