#include "Object.h"
#include <algorithm>
#include <ctime>
#include <boost/lexical_cast.hpp>

namespace ORB_SLAM2{

unsigned int Object::nextId_ = 0;
unsigned int ObjectType::nextId_ = 0;

ObjectType::ObjectType(std::string name, Shape shape, float radius=0, uint maxQuantity=1, float minSpacing=1){
  quantity_ = 0;
  radius_ = radius;
  maxQuantity_ = maxQuantity;
  minSpacing_ = minSpacing;
  shape_ = shape;
  name_ = name;
  id_ = nextId_++;
}

ObjectType::ObjectType(){
  time_t seconds;
  time(&seconds);
  std::string name = boost::lexical_cast<std::string>(seconds);
  ObjectType(name,Shape::None);
}


void ObjectType::setShape(std::string shape){
  std::transform(shape.begin(), shape.end(), shape.begin(), ::tolower);

  if(shape == "sphere")
    shape_ = Shape::Sphere;
  else if(shape == "cylinder")
    shape_ = Shape::Cylinder;
  else if(shape == "box")
    shape_ = Shape::Box;
  else if(shape == "none")
    shape_ = Shape::None;
  else{
    throw("Shape string not recongized");
  }
}

void ObjectType::setShape(Shape shape){
  shape_ = shape;
}

Object::Object():confidence_(0),type_(ObjectType()){
  id_ = nextId_++;
  ;
}

Object::Object(const ObjectType& type):confidence_(0), type_(type){
  id_ = nextId_++;
}

void Object::addPoint(MapPoint* point){
  mapPoints_.insert(point);
}

void Object::addPoints(std::set<MapPoint*> points){
  for(auto p : points){
    mapPoints_.insert(p);
  }
}
}
