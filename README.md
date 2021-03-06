# YCR
Basic path tracing render\
YCR stands for **YessionCC Render**\
**NOTICE**: This project is still under development and some code is unavailable

## Environment
The program is compiled under Ubuntu 18.04， GCC， C++11

## Dependences
- [GLM](https://github.com/g-truc/glm): used for basic vector operations
- [ASSIMP](https://www.assimp.org/): used for loading 3D models
- [yaml-cpp](https://github.com/jbeder/yaml-cpp): used for parsing configuration(this function has not been implemented yet)
- OpenCV: just used for presenting render process

## Features
- Path tracing with MIS
- Bidirectional path tracing with MIS
- Multithreading acceleration
- SAH-BVH heurisitic acceleration structure
- Bump mapping
- Support homogenuous medium (volume render, both pt and bdpt are supported)
- Support HDR output
- Supports importing obj files and transform its materials to PBR materials
- Support lights:
  - point light
  - directional light
  - area light
  - environment light (support importance sampling)
- Support materials: 
  - Specular
  - Fresnel Glass
  - GGX reflection/transmission
  - Lambertain reflection
  - Mixed materials (support material tree node)
- Support filters:
  - Box
  - Gaussian
  - Mitchell
- Support shapes:
  - Polygon Mesh (from obj file)
  - Sphere

## Expected Features
- Read configuration from yaml
- Robust and really correct bidirectional path tracing (adjoint bxdf ...)
- Render hair (based on Bezier curve)

## Some Results
- 1024 spp, 205s, pt, Environment light/GGX/Specular/Lambertain
![Ball](/present/ball.jpg "Ball")

- 1024 spp, 231s, pt, SphereLight/Plastic
![Scene](/present/scene.jpg "Scene")

- 3000 spp, 2.5h, pt, Obj model
![Casa](/present/casa.jpg "Casa")

- 1024 spp, 305s, bdpt, Glass Caustic
![bdpt_glass](/present/res_bdpt1024.jpg "bdpt_glass")

- 4096 spp, 40min, bdpt
![fogbdpt](/present/fogbdpt.jpg "fogbdpt")