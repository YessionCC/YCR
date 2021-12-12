#pragma once 

#include <map>
#include <iostream>
#include <vector>
#include <string>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

class Mesh;
class Texture;

class SceneImporter {
private:
  static std::string curDirectory;
  static void processNode(const aiScene *scene, aiNode *node, std::vector<Mesh*>& meshes);

  static Mesh* processMesh(const aiScene *scene, aiMesh *mesh);

  static void loadMaterialTextures(
    aiMaterial *mat, aiTextureType type, std::vector<const Texture*>& texs);

public:

  static bool loadSceneFromFile(const char* path, std::vector<Mesh*>& meshes);
};