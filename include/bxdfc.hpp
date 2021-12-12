#include "material.hpp"

#include "bxdf.hpp"

class PerfectGlass: public MixedBXDF {
public:
  PerfectGlass(float IOR, const Texture* refl, const Texture* refr):
    MixedBXDF( // NOTICE: memory recycle
      new PerfectSpecular(refl),  
      new PerfectTransimission(refr, IOR), 
      new DielectricFresnelBlender(IOR)
    ){}
  PerfectGlass(float IOR, const Texture* tex):
    MixedBXDF( // NOTICE: memory recycle
      new PerfectSpecular(tex),  
      new PerfectTransimission(tex, IOR), 
      new DielectricFresnelBlender(IOR)
    ){}
};

class SpecularCeramics: public MixedBXDF {
public:
  SpecularCeramics(float IOR, const Texture* albedo):
    MixedBXDF(
      new PerfectSpecular(new SolidTexture(1.0f)),
      new LambertianReflection(albedo),
      new DielectricFresnelBlender(IOR)
    ){}
};

class PrincipleBSDF1: public MixedBXDF {
public:
  PrincipleBSDF1(const Texture* diff, const Texture* spec):
    MixedBXDF(
      new PerfectSpecular(diff),
      new LambertianReflection(diff),
      new TexBlender(spec)
  ) {}
  PrincipleBSDF1(const Texture* diff, const Texture* spec, const Texture* glossy):
    MixedBXDF(
      new PrincipleBSDF1(diff, spec),
      new GGXReflection(glossy, diff),
      new TexBlender(glossy)
  ) {}
  PrincipleBSDF1(
    const Texture* diff, const Texture* spec, 
    const Texture* glossy, const Texture* opac, float IOR):
    MixedBXDF(
      new PrincipleBSDF1(diff, spec, glossy),
      new GGXTransimission(IOR, glossy, diff),
      new TexBlender(opac)
  ) {}
};

