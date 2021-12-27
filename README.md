# YCR
Basic path tracing render\
YCR stands for **YessionCC Render**\
**NOTICE**: This project is still under development and some code is unavailable

## Environment
The program is compiled under Ubuntu 18.04， GCC， C++11

## Dependences
- [ASSIMP](https://www.assimp.org/): used for loading 3D models
- [yaml-cpp](https://github.com/jbeder/yaml-cpp): used for parsing configuration(this function has not been implemented yet)
- OpenCV: just used for presenting render process

## Features
- Path tracing with MIS
- Bidirectional path tracing with MIS
- Multithreading acceleration
- SAH-BVH heurisitic acceleration structure
- Bump mapping
- Support medium (volume render, both pt and bdpt are supported)
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

- 1024 spp, 4min, pt, Global fog
![Fog](/present/fog.jpg "Fog")

- 4096 spp, 25min, pt, Obj model
![Nano](/present/nano.jpg "Nano")

- 1024 spp, 305s, bdpt, Glass Caustic
![bdpt_glass](/present/res_bdpt1024.jpg "bdpt_glass")

- 256 spp, 442s, bdpt, Global fog(bdpt denoise)
![bdpt_global_fog](/present/res_glfogbdpt256.jpg "bdpt_global_fog")