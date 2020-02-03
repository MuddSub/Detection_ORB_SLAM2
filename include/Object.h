#ifndef OBJECT_H
#define OBJECT_H

#include <vector>
#include <string>
#include <opencv2/core/core.hpp>
#include <set>

#include "MapPoint.h"

namespace ORB_SLAM2{

class MapPoint;

class ObjectType{


public:

  enum class Shape{Sphere, Cylinder, Box, None};

  ObjectType();
  ObjectType(std::string name_, Shape shape, float radius, uint maxQuantity, float minSpacing);


  void setShape(std::string shape);
  void setShape(Shape shape);

  //max radius of object
  float radius_;

  //expeted number of objects in field(0 := infinity)
  uint maxQuantity_;

  //currently existing quantity
  uint quantity_;

  //what's the closest (centroid-to-centroid) they can be?
  float minSpacing_;

  //sphere, cylinder, or box
  Shape shape_;

  std::string name_;

  static unsigned int nextId_;
  unsigned int id_;

private:



};


class Object{
public:


  Object(const Object& other) = default;

  Object();
  Object(const ObjectType& type);


  //TODO Confidence of localization of the object
  float getConfidence() const;

  //TODO: Run Bees to fit a shape
  void fitShape();

  //Add points to the PC
  void addPoints(std::set<MapPoint*> points);

  //Add a single point
  void addPoint(MapPoint* point);


  //unique ID
  unsigned int id_;

  static unsigned int nextId_;

  const ObjectType& type_;

  //All the map points belonging to the object
  std::set<MapPoint*> mapPoints_;

  //SE3 centroid
  cv::Mat centroid_;

  //Vector descriptors of the fit object
  std::vector<double> objectFit_;

  float confidence_;

private:



};

}

#endif
