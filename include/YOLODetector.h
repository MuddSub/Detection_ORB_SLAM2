#ifndef ORB_SLAM2_YOLODETECTOR_H
#define ORB_SLAM2_YOLODETECTOR_H

#include "Detector.h"
//#include "YOLOv3_SpringEdition/Yolov3_SpringEdition_Test/YOLOv3SE.h"


namespace ORB_SLAM2 {
class YOLODetector : public Detector{

public:

  YOLODetector(std::string weightsFile, std::string cfgFile,
               std::string classesFile, std::vector<unsigned int> types,
               const System &sys);
  YOLODetector(std::string weightsFile, std::string cfgFile,
               std::string classesFile, const System &sys);

  std::string weightsFile_;
  std::string classesFile_;
  std::string cfgFile_;

  float confThreshold_ = 0.5; // Confidence threshold
  float nmsThreshold_ = 0.4;  // Non-maximum suppression threshold
  int inpWidth_ = 752;        // Width of network's input image
  int inpHeight_ = 480;       // Height of network's input image

private:
  void setupDetector();

};
}


#endif