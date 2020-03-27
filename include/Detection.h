//
// Created by sethgi on 3/23/20.
//

#ifndef ORB_SLAM2_DETECTION_H
#define ORB_SLAM2_DETECTION_H

#include "Object.h"

namespace ORB_SLAM2 {
  class ObjectType;

  class Detection {
  public:
    Detection() = default;

    Detection(ObjectType* t, int xMin, int xMax, int yMin, int yMax);

    ObjectType* type_;
    unsigned int xMin_;
    unsigned int xMax_;
    unsigned int yMin_;
    unsigned int yMax_;

    friend std::ostream &operator<<(std::ostream &os, Detection const &d);

  };
}


#endif //ORB_SLAM2_DETECTION_H
