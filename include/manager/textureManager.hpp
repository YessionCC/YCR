#pragma once

#include "texture.hpp"

#include <string>
#include <vector>
#include <map>

#define _TexMan TextureManager::manager()
#define _TexManWhite TextureManager::manager().createWhiteSolid();
#define _TexManBlack TextureManager::manager().createBlackSolid();
#define _TexManST TextureManager::manager().createSolidTexture
#define _TexManIT TextureManager::manager().createImageTexture

class TextureManager {
private:
  std::map<std::string, ImageTexture*> im_textures;
  std::vector<SolidTexture*> so_textures;
  SolidTexture *pureWhite, *pureBlack;

  TextureManager() {
    pureWhite = new SolidTexture(1.0f);
    pureBlack = new SolidTexture(0.0f);
  }

public:
  static TextureManager& manager() {
    static TextureManager man;
    return man;
  }

  Texture* createSolidTexture(glm::vec3 col) {
    so_textures.push_back(new SolidTexture(col));
    return so_textures.back();
  }
  Texture* createSolidTexture(float gray) {
    so_textures.push_back(new SolidTexture(gray));
    return so_textures.back();
  }
  Texture* createWhiteSolid() {
    return pureWhite;
  }
  Texture* createBlackSolid() {
    return pureBlack;
  }


  Texture* createImageTexture(
    std::string path, float scale = 1.0f, 
    ImageTexture::InterpolateMode mode = ImageTexture::InterpolateMode::BILINEAR) {
    
    auto res = im_textures.find(path);
    if(res != im_textures.end()) return res->second;
    ImageTexture* tex = new ImageTexture(path.c_str(), scale, mode);
    im_textures[path] = tex;
    return tex;
  }

};