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

class StandardSpecular: public AddBXDF {
public:
  StandardSpecular(float IOR, const Texture* base, const Texture* refl):
    AddBXDF(
      new LambertianReflection(base),
      new WeightedBXDF(
        new PerfectSpecular(refl),
        new DielectricFresnelBlender(IOR)
      )
    ){}
};

class StandardGGXRefl: public AddBXDF {
public:
  StandardGGXRefl(float IOR, const Texture* rough, const Texture* base, const Texture* refl):
    AddBXDF(
      new LambertianReflection(base),
      new WeightedBXDF(
        new GGXReflection(rough, refl),
        new DielectricFresnelBlender(IOR)
      )
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

