//
// Created by sethgi on 3/23/20.
//

#include "Detection.h"

namespace ORB_SLAM2{
Detection::Detection(ObjectType* t, int xMin, int xMax, int yMin, int yMax):type_(t), xMin_(xMin), xMax_(xMax),
                                                                           yMin_(yMin), yMax_(yMax)
{}

std::ostream &operator<<(std::ostream &os, Detection const &d){
  return os << "  Type:" << d.type_->name_ << "  xMin: " << d.xMin_ << "  xMax: " << d.xMax_ << "  yMin: " << d.yMin_
  << "  yMax" << d.yMax_;
}
}
