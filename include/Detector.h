#ifndef DETECTOR_H
#define DETECTOR_H

#include "Object.h"
#include "System.h"
#include "Tracking.h"
#include <opencv2/core/core.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <vector>
#include <map>
#include <utility>
#include <mutex>

namespace ORB_SLAM2{

class Object;
class ObjectType;
class System;
class Tracking;

namespace bg = boost::geometry;
typedef bg::model::point<double, 2, bg::cs::cartesian> point_t;
typedef bg::model::polygon<point_t> polygon_t;
typedef std::pair<ObjectType, polygon_t> detection_t;


class Detector{

public:

  //needs an associated system, can't default construct
  Detector() = delete;

  Detector(const Detector& other) = default;

  Detector(const System& sys, std::vector<unsigned int> types);

  Detector(const System& sys);


  cv::Mat getImage(bool right);

  //Vector of pairs of the type of object and bounding box for each detection
  void results(std::vector<detection_t> detections);

  // Main thread function. Draw points, keyframes, the current camera pose and the last processed
  // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
  void Run();

  void RequestFinish();

  void RequestStop();

  bool isFinished();

  bool isStopped();

  void Release();


  unsigned int id_;
  static unsigned int nextId_;

  bool detectRight = false;
  bool detectLeft = true;


  virtual void detect(cv::Mat image){
    cv::namedWindow( "Detector", cv::WINDOW_AUTOSIZE );// Create a window for display.
    cv::imshow( "Detector", image );                   // Show our image inside it.//
    sleep(1);
  }

private:

  //object type id, number found
  std::set<unsigned int> objectTypes_;

  bool overlappingDetections_ = false;

  bool finishRequested_;
  bool finished_;
  std::mutex mutexFinish_;

  bool stopped_;
  bool stopRequested_;
  std::mutex mutexStop_;

  bool CheckFinish();
  void SetFinish();
  bool Stop();

  std::vector<unsigned int> processedFrames_;

  const System& system_;



};
}




#endif
