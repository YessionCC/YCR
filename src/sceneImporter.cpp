#include "sceneImporter.hpp"

#include "mesh.hpp"
#include "utility.hpp"
#include "material.hpp"
#include "manager/textureManager.hpp"

#include <string>
#include <iostream>

std::string SceneImporter::curDirectory = std::string();

bool SceneImporter::loadSceneFromFile(const char* path, std::vector<Mesh*>& meshes) {
  Assimp::Importer importer;
  const aiScene* scene = importer.ReadFile(
    path, 
    aiProcess_Triangulate | 
    aiProcess_GenSmoothNormals | 
    aiProcess_CalcTangentSpace);
  if(!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode){
    std::cout << "ERROR::ASSIMP:: " << importer.GetErrorString() << std::endl;
    return false;
  }

  std::string strFilename(path);
  curDirectory = strFilename.substr(0, strFilename.find_last_of('/'));

  processNode(scene, scene->mRootNode, meshes);
  std::cout<<"Load model: "<<path<<" successfully!"<<std::endl;
  return true;
}


void SceneImporter::processNode(
  const aiScene *scene, aiNode *node, std::vector<Mesh*>& meshes){

  for(unsigned int i = 0; i < node->mNumMeshes; i++){
    aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
    meshes.push_back(processMesh(scene, mesh));
  }
  for(unsigned int i = 0; i < node->mNumChildren; i++){
      processNode(scene, node->mChildren[i], meshes);
  }
}

Mesh* SceneImporter::processMesh(const aiScene *scene, aiMesh *mesh) {
  // data to fill
  VertexMesh* modelMesh = new VertexMesh;
  Vertex* vertices = new Vertex[mesh->mNumVertices];
  modelMesh->vertices.resize(mesh->mNumVertices);

  glm::vec3 vector; glm::vec2 vec;
  // walk through each of the mesh's vertices
  for(unsigned int i = 0; i < mesh->mNumVertices; i++) {
    Vertex* vertex = &vertices[i];
    
    // positions
    vector.x = mesh->mVertices[i].x;
    vector.y = mesh->mVertices[i].y;
    vector.z = mesh->mVertices[i].z;
    vertex->position = vector;
    // normals
    if (mesh->HasNormals()) {
      vector.x = mesh->mNormals[i].x;
      vector.y = mesh->mNormals[i].y;
      vector.z = mesh->mNormals[i].z;
      vertex->normal = glm::normalize(vector);
    }
    // texture coordinates
    if(mesh->mTextureCoords[0]){

      vec.x = mesh->mTextureCoords[0][i].x; 
      vec.y = mesh->mTextureCoords[0][i].y;
      vertex->uv = vec;
      // tangent
      vector.x = mesh->mTangents[i].x;
      if(std::isnan(vector.x)) vector.x = 0.0f;
      vector.y = mesh->mTangents[i].y;
      if(std::isnan(vector.y)) vector.y = 0.0f;
      vector.z = mesh->mTangents[i].z;
      if(std::isnan(vector.z)) vector.z = 0.0f;
      if(!IsBlack(vector))
        vertex->tangent = glm::normalize(vector);
      // bitangent
      vector.x = mesh->mBitangents[i].x;
      if(std::isnan(vector.x)) vector.x = 0.0f;
      vector.y = mesh->mBitangents[i].y;
      if(std::isnan(vector.y)) vector.y = 0.0f;
      vector.z = mesh->mBitangents[i].z;
      if(std::isnan(vector.z)) vector.z = 0.0f;
      if(!IsBlack(vector))
        vertex->btangent = glm::normalize(vector);
    }
      
    modelMesh->vertices[i] = vertex;
  }
  for(unsigned int i = 0; i < mesh->mNumFaces; i++) {
    aiFace face = mesh->mFaces[i];
    for(unsigned int j = 0; j < face.mNumIndices; j++)
      modelMesh->indices.push_back(face.mIndices[j]);        
  }
  aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];    
  MaterialInfo minfo; aiColor3D IOR;
  loadMaterialTextures(material, aiTextureType_DIFFUSE, minfo.diffuse);
  loadMaterialTextures(material, aiTextureType_SPECULAR, minfo.specular);
  loadMaterialTextures(material, aiTextureType_EMISSIVE, minfo.lightMap);
  loadMaterialTextures(material, aiTextureType_SHININESS, minfo.glossy);
  loadMaterialTextures(material, aiTextureType_OPACITY, minfo.opacity);
  material->Get(AI_MATKEY_REFRACTI, IOR);
  minfo.IOR = IOR.r;
  // Height map or Normal map need to be distinguished here
  loadMaterialTextures(material, aiTextureType_HEIGHT, minfo.normal);
  modelMesh->material.handleMaterialInfo(minfo);
  return modelMesh;
}

void SceneImporter::loadMaterialTextures(
  aiMaterial *mat, aiTextureType type, std::vector<const Texture*>& texs){
  
  if(mat->GetTextureCount(type) == 0) {
    aiColor3D col; bool check = false;
    if(type == aiTextureType_DIFFUSE) {
      mat->Get(AI_MATKEY_COLOR_DIFFUSE, col), check=true;
      // we set default diffuse color white
      if(IsBlack({col.r, col.g, col.b})) col.r = col.g = col.b = 1.0f;
    }
    if(type == aiTextureType_SPECULAR) {
      mat->Get(AI_MATKEY_COLOR_SPECULAR, col), check=true;
      if(IsBlack({col.r, col.g, col.b})) return;
    }
    if(type == aiTextureType_EMISSIVE) {
      mat->Get(AI_MATKEY_COLOR_EMISSIVE, col), check=true;
      if(IsBlack({col.r, col.g, col.b})) return;
    }
    // may not correct
    //if(type == aiTextureType_OPACITY) mat->Get(AI_MATKEY_COLOR_TRANSPARENT, col), check=true;
    if(type == aiTextureType_SHININESS) {
      mat->Get(AI_MATKEY_SHININESS, col), check=true;
      if(IsBlack({col.r, col.g, col.b})) return;
    }

    if(check) texs.push_back(_TexManST({col.r, col.g, col.b}));
    return;
  }
  
  // althogh we consider multiple textures have the same type
  // we only use the first texture for simplycity
  for(unsigned int i = 0; i < mat->GetTextureCount(type); i++){
    aiString str;
    mat->GetTexture(type, i, &str);
    texs.push_back(_TexManIT(curDirectory + '/' + str.C_Str()));
  }
}