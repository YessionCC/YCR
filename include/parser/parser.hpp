#pragma once

#include "yaml-cpp/yaml.h"
#include "resourceManager.hpp"

class Parser {
private:
  ResourceManager& manager;
  YAML::Node root;

public:
  Parser(const char* scenePath): 
    manager(ResourceManager::manager()), root(YAML::LoadFile(scenePath)){}

  bool parse() {
    if(!root["Scene"]) {
      std::cout << "ERROR: No Scene Contained!"<<std::endl;
      return false;
    }
    YAML::Node sceneNode = root["Scene"];
    if(!sceneNode["Meshes"]) {
      std::cout << "ERROR: There are must have meshes in scene!"<<std::endl;
      return false;
    }
    manager.createMesh(sceneNode["Meshes"]);
  }
};