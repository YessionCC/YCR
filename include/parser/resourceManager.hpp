#pragma once 

#include <map>
#include <string>
#include "yaml-cpp/yaml.h"

#include "model.hpp"
#include "bxdf.hpp"
#include "texture.hpp"
#include "mesh.hpp"
#include "camera.hpp"

class ResourceManager {

private:
  std::map<std::string, Model> models;
  std::map<std::string, Camera> cams;
  std::map<std::string, Mesh*> meshes;
  std::map<std::string, BXDF*> bxdfs;
  std::map<std::string, Texture*> textures;

public:
  ~ResourceManager() {
    for(auto p: bxdfs) delete p.second;
    for(auto p: textures) delete p.second;
  }

  static ResourceManager& manager() {
    static ResourceManager man;
    return man;
  }

  bool createMesh(const YAML::Node& meshNode) {
    int cnt = 0;
    for(auto mesh: meshNode) {
      cnt++;
      if(!mesh["Name"]) {
        std::cout<<"ERROR: Mesh["<<cnt<<"] no name"<<std::endl;
        return false;
      }
      if(!mesh["Type"]) {
        std::cout<<"ERROR: Mesh["<<cnt<<"] no type"<<std::endl;
        return false;
      }
      std::string type = mesh["Type"].as<std::string>();
      if(type == "Vertex") {
        if(!mesh["Path"]) {
          std::cout<<"ERROR: VertexMesh["<<cnt<<"] no path"<<std::endl;
          return false;
        }
        VertexMesh* mesh = new VertexMesh()
      }
    }
  }
};