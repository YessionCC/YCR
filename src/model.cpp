#include "model.hpp"
#include "vertex.hpp"
#include "distribution.hpp"
#include "medium.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <iostream>

Model::Model(const char* filename) {
  loadFromFile(filename);
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

bool Model::loadFromFile(const char* filename){
  Assimp::Importer importer;
  const aiScene* scene = importer.ReadFile(
    filename, 
    aiProcess_Triangulate | 
    aiProcess_GenSmoothNormals | 
    aiProcess_CalcTangentSpace);
  if(!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode){
    std::cout << "ERROR::ASSIMP:: " << importer.GetErrorString() << std::endl;
    isInvalidModel = false;
    return false;
  }

  std::string strFilename(filename);
  modelDirectory = strFilename.substr(0, strFilename.find_last_of('/'));

  processNode(scene->mRootNode, scene);
  return true;
}

void Model::setBxdfForAllMeshes(const BXDF* bxdf) {
  for(Mesh* mesh: meshes) mesh->bxdf = bxdf;
}

void Model::setLightForAllMeshes(const Light* light) {
  for(Mesh* mesh: meshes) mesh->light = light;
}

// set Medeium(default inside medium)
void Model::setMediumForAllMeshes(const Medium* medium, bool isInside) {
  if(isInside) for(Mesh* mesh: meshes) mesh->mediumInside = medium;
  else for(Mesh* mesh: meshes) mesh->mediumOutside = medium;
}

void Model::processNode(aiNode *node, const aiScene *scene){
  for(unsigned int i = 0; i < node->mNumMeshes; i++){
    aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
    meshes.push_back(processMesh(mesh, scene));
  }
  for(unsigned int i = 0; i < node->mNumChildren; i++){
      processNode(node->mChildren[i], scene);
  }
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

Mesh* Model::processMesh(aiMesh *mesh, const aiScene *scene) {
  // data to fill
  VertexMesh* modelMesh = new VertexMesh;

  // walk through each of the mesh's vertices
  for(unsigned int i = 0; i < mesh->mNumVertices; i++) {
    Vertex* vertex = new Vertex;
    glm::vec3 vector; 
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
        glm::vec2 vec;
        
        vec.x = mesh->mTextureCoords[0][i].x; 
        vec.y = mesh->mTextureCoords[0][i].y;
        vertex->uv = vec;
        // tangent
        vector.x = mesh->mTangents[i].x;
        vector.y = mesh->mTangents[i].y;
        vector.z = mesh->mTangents[i].z;
        vertex->tangent = glm::normalize(vector);
        // bitangent
        vector.x = mesh->mBitangents[i].x;
        vector.y = mesh->mBitangents[i].y;
        vector.z = mesh->mBitangents[i].z;
        vertex->btangent = glm::normalize(vector);
    }
    else
        vertex->uv = glm::vec2(0.0f, 0.0f);

    modelMesh->vertices.push_back(vertex);
  }
  for(unsigned int i = 0; i < mesh->mNumFaces; i++) {
      aiFace face = mesh->mFaces[i];
      for(unsigned int j = 0; j < face.mNumIndices; j++)
          modelMesh->indices.push_back(face.mIndices[j]);        
  }
  /*aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];    

  vector<Texture> diffuseMaps = loadMaterialTextures(material, aiTextureType_DIFFUSE, "texture_diffuse");
  textures.insert(textures.end(), diffuseMaps.begin(), diffuseMaps.end());

  vector<Texture> specularMaps = loadMaterialTextures(material, aiTextureType_SPECULAR, "texture_specular");
  textures.insert(textures.end(), specularMaps.begin(), specularMaps.end());

  std::vector<Texture> normalMaps = loadMaterialTextures(material, aiTextureType_HEIGHT, "texture_normal");
  textures.insert(textures.end(), normalMaps.begin(), normalMaps.end());

  std::vector<Texture> heightMaps = loadMaterialTextures(material, aiTextureType_AMBIENT, "texture_height");
  textures.insert(textures.end(), heightMaps.begin(), heightMaps.end());*/

  return modelMesh;
}