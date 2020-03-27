//
// Created by sethgi on 3/23/20.
//

#include "YOLODetector.h"

namespace ORB_SLAM2 {

YOLODetector::YOLODetector(std::string weightsFile, std::string cfgFile,
                           std::string classesFile, const System &sys) :
  Detector(sys), weightsFile_(weightsFile),cfgFile_(cfgFile), classesFile_(classesFile)
{

}

YOLODetector::YOLODetector(std::string weightsFile, std::string cfgFile,
                           std::string classesFile, std::vector<unsigned int> types,
                           const System &sys) :
  Detector(sys, types), weightsFile_(weightsFile), cfgFile_(cfgFile), classesFile_(classesFile)
{

}

void YOLODetector::setupDetector(){


}

}