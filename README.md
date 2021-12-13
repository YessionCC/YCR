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
- Multithreading acceleration
- SAH-BVH heurisitic acceleration structure
- Support medium (volume render)
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
- Bidirectional path tracing (Though it's very difficult, I have started out)
- Render hair (based on Bezier curve)

## Some Results
