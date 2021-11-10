#include "primitive.hpp"

#include "model.hpp"
#include "sampler.hpp"

#include <glm/gtc/matrix_transform.hpp>

#include <iostream>

/**********************Triangle******************************/

Triangle::Triangle(Vertex* v1, Vertex* v2, Vertex* v3, Mesh* mesh) {
  verts[0] = v1;
  verts[1] = v2;
  verts[2] = v3;
  this->mesh = mesh;
}

BB3 Triangle::getBB3() const{
  BB3 bb3(verts[0]->position);
  bb3.update(verts[1]->position);
  bb3.update(verts[2]->position);
  return bb3;
}

Primitive* Triangle::copy(Mesh* _mesh) const {
  std::cout<< 
  "Warning, This method: Triangle::copy, should never be called, "
  "Triangle copy should based on Vertex not Primitive, "
  "if this method called, you may put triangle to CustomMesh"
  <<std::endl;
  Primitive* prim = new Triangle(verts[0], verts[1], verts[2], _mesh);
  return prim;
}

float Triangle::getAPointOnSurface(Intersection& itsc) {
  itsc.localUV = SampleShape::sampler().uniSampleTriangle();
  itsc.prim = this;
  handleItscResult(itsc);
  return 1.0f/getArea();
}

// Triangle::intersect will set localUV, because it's easier
// to calc params from localUV
void Triangle::intersect(const Ray& ray, Intersection& itsc) { //Mollor method
  glm::vec3 v0v1 = verts[1]->position - verts[0]->position;
  glm::vec3 v0v2 = verts[2]->position - verts[0]->position;
  glm::vec3 pvec = glm::cross(ray.d, v0v2);
  float det = glm::dot(v0v1, pvec);
  if(std::abs(det)<CUSTOM_EPSILON) return; //parallel
  //if(det<0) the triangle do not face to ray
  // TODO: if the material not transmission, than return false;

  float invDet = 1.0f/det;
  glm::vec3 tvec = ray.o - verts[0]->position;
  float u = glm::dot(tvec, pvec)*invDet;
  if(u<0 || u>1) return;

  glm::vec3 qvec = glm::cross(tvec, v0v1);
  float v = glm::dot(ray.d, qvec)*invDet;
  if(v<0 || u+v>1) return;

  float t = glm::dot(v0v2, qvec)*invDet;
  if(t<CUSTOM_EPSILON) return;
  
  itsc.updateItscInfo(t, this, {u, v});
}

bool Triangle::intersectTest(const Ray& ray) const{
  glm::vec3 v0v1 = verts[1]->position - verts[0]->position;
  glm::vec3 v0v2 = verts[2]->position - verts[0]->position;
  glm::vec3 pvec = glm::cross(ray.d, v0v2);
  float det = glm::dot(v0v1, pvec);
  if(std::abs(det)<CUSTOM_EPSILON) return false; 

  float invDet = 1.0f/det;
  glm::vec3 tvec = ray.o - verts[0]->position;
  float u = glm::dot(tvec, pvec)*invDet;
  if(u<0 || u>1) return false;

  glm::vec3 qvec = glm::cross(tvec, v0v1);
  float v = glm::dot(ray.d, qvec)*invDet;
  if(v<0 || v>1) return false;

  float t = glm::dot(v0v2, qvec)*invDet;
  if(t<CUSTOM_EPSILON) return false;

  return true;
}

//set normal, uv, and other for itsc from localUV
void Triangle::handleItscResult(Intersection& itsc) const{ // for triangle
  float u = itsc.localUV[0], v = itsc.localUV[1], w = 1-u-v;
  #define BLEND(var) (w*verts[0]->var + u*verts[1]->var + v*verts[2]->var)
  
  // notice: normal/tangent interpolation need to be normalized!
  itsc.itscVtx.normal = glm::normalize(BLEND(normal));
  itsc.itscVtx.tangent = glm::normalize(BLEND(tangent));
  itsc.itscVtx.btangent = glm::cross(itsc.itscVtx.tangent, itsc.itscVtx.normal);
  itsc.itscVtx.uv = BLEND(uv); // real uv for map
  itsc.itscVtx.position = BLEND(position);
  
  #undef BLEND

  // we assume that u,v,w, normal are all exact(ignore their numerical error)
  // we just make sure no self-intersection happen
  // error = |up1|G3+|vp|2G3+|wp|3G2, NOTICE error must use abs() !!!
  itsc.itscError = _Gamma(3)*
    (w*glm::abs(verts[0]->position)+u*glm::abs(verts[1]->position))+
    _Gamma(2)*v*glm::abs(verts[0]->position);
}

void Triangle::autoCalcParams(bool reverseNormal) { 
  glm::vec3 v01 = verts[1]->position - verts[0]->position;
  glm::vec3 v02 = verts[2]->position - verts[0]->position;
  glm::vec3 normal = glm::normalize(glm::cross(v01, v02));
  if(reverseNormal) normal = -normal;
  glm::vec3 btan = glm::normalize(glm::cross(v01, normal));

  #define SET_PROP(p, v) verts[0]->p = v; verts[1]->p = v; verts[2]->p = v;

  SET_PROP(normal, normal)
  SET_PROP(tangent, v01)
  SET_PROP(btangent, btan)
  
  #undef SET_PROP
}

void Triangle::genPrimPointCloud(PCShower& pc, glm::vec3 col) const {
  glm::vec2 uv = SampleShape::sampler().uniSampleTriangle();
  glm::vec3 pos = uv.x*verts[0]->position+
    uv.y*verts[1]->position+(1.0f-uv.x-uv.y)*verts[2]->position;
  pc.addItem(pos, col);
}

void Triangle::translate(glm::vec3) {
  std::cout<< 
  "Warning, This method: Triangle::translate, should never be called, "
  "Triangle copy should based on Vertex not Primitive, "
  "if this method called, you may put triangle to CustomMesh"
  <<std::endl;
}
void Triangle::scale(glm::vec3) {
  std::cout<< 
  "Warning, This method: Triangle::scale, should never be called, "
  "Triangle copy should based on Vertex not Primitive, "
  "if this method called, you may put triangle to CustomMesh"
  <<std::endl;
}
void Triangle::rotate(glm::vec3, float) {
  std::cout<< 
  "Warning, This method: Triangle::rotate, should never be called, "
  "Triangle copy should based on Vertex not Primitive, "
  "if this method called, you may put triangle to CustomMesh"
  <<std::endl;
}

/**********************Sphere******************************/

Primitive* Sphere::copy(Mesh* _mesh) const {
  return new Sphere(this->center, this->radius, _mesh);
}

glm::vec3 Sphere::uniSampleSphereDir() const {
  glm::vec2 uv = SampleShape::sampler().uniSampleSphere();
  glm::vec3 pos;
  pos.z = uv.x;
  float sinTheta = glm::sqrt(1-uv.x*uv.x);
  pos.x = sinTheta*glm::cos(uv.y);
  pos.y = sinTheta*glm::sin(uv.y);
  return pos;
}

void Sphere::calcItscInfoFromNormal(Intersection& itsc, glm::vec3 normal) const{
  itsc.itscVtx.position = center+radius*normal;
  itsc.itscVtx.tangent = glm::normalize(glm::cross(up, normal));
  itsc.itscVtx.btangent = glm::cross(itsc.itscVtx.tangent, itsc.itscVtx.normal);
  itsc.itscVtx.uv.y = glm::acos(glm::dot(up, normal))*INV_PI;
  float phi = std::atan2(
      glm::dot(forward, normal), 
      glm::dot(right, normal));
  if(phi < 0) phi+=PI2;
  itsc.itscVtx.uv.x = phi*INV_PI2;

  // assume: normalize introduce gamma3, c+rn, introduce gamma2
  itsc.itscError = _Gamma(1)*glm::abs(center)+
    _Gamma(5)*radius*glm::abs(normal);
}

// return pdf
float Sphere::getAPointOnSurface(Intersection& itsc){
  glm::vec3 pos = uniSampleSphereDir();
  itsc.prim = this;
  itsc.itscVtx.normal = pos;
  calcItscInfoFromNormal(itsc, pos);

  return 1.0f/getArea();
}

// notice: itsc as in-out variable
// Sphere::intersect set itsc.position directly
// because its easier to calc normal and other params from pos
void Sphere::intersect(const Ray& ray, Intersection& itsc){
  glm::vec3 so = ray.o - this->center;
  // t*t+b*t+c = 0
  float b = 2.0f*glm::dot(ray.d, so);
  float c = so.x*so.x+so.y*so.y+so.z*so.z - this->radius*this->radius;
  float delta2 = b*b - 4*c;
  if(delta2 < 0) return;
  float delta = glm::sqrt(delta2);
  float t1 = 0.5f*(-b+delta), t2 = 0.5f*(-b-delta);
  if(t1 < CUSTOM_EPSILON) return;
  if(t2 < CUSTOM_EPSILON) itsc.updateItscInfo(t1, this, ray);
  else itsc.updateItscInfo(t2, this, ray);
}

// just return whether intersect or not
bool Sphere::intersectTest(const Ray& ray) const{
  glm::vec3 so = ray.o - this->center;
  float b = glm::dot(ray.d, so);
  float c = so.x*so.x+so.y*so.y+so.z*so.z - this->radius*this->radius;
  float delta2 = b*b - 4*c;
  if(delta2 < 0) return false;
  float t1 = 0.5f*(-b+glm::sqrt(delta2));;
  if(t1 < CUSTOM_EPSILON) return false;
  return true;
}

//set normal, uv, and other for itsc from itsc.position
void Sphere::handleItscResult(Intersection& itsc) const{
  glm::vec3 normal = glm::normalize(itsc.itscVtx.position - this->center);
  itsc.itscVtx.normal = normal;
  calcItscInfoFromNormal(itsc, normal);
}

void Sphere::genPrimPointCloud(PCShower& pc, glm::vec3 col) const{
  pc.addItem(uniSampleSphereDir()*radius+center, col);
}

void Sphere::translate(glm::vec3 trans){
  this->center+=trans;
}
void Sphere::scale(glm::vec3 scl){
  if(!(scl.x == scl.y && scl.x == scl.z)) {
    std::cout<<
    "Warning: Sphere::scale do not support anisotropic scale"
    <<std::endl;
  }
  radius *= scl.x;
}
// NOTICE: the origin is the world origin
void Sphere::rotate(glm::vec3 axis, float angle){
  glm::mat4x4 trans(1.0f);
  trans = glm::rotate(trans, angle, axis);
  this->center = glm::mat3x3(trans)*this->center;
  this->up = glm::mat3x3(trans)*this->up;
  this->right = glm::mat3x3(trans)*this->right;
  this->forward = glm::cross(right, up);
}