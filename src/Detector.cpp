#include "Detector.h"

namespace ORB_SLAM2{
uint Detector::nextId_ = 0;


Detector::Detector(const System& sys, std::vector<unsigned int> types): system_(sys),finishRequested_(false),
    finished_(false),stopped_(false),stopRequested_(false){
  id_ = nextId_++;
  cout << (system_.mReady? "YES":"NO") << endl;
  for(auto t : types){
    objectTypes_.insert(t);
  }

}

Detector::Detector(const System& sys): system_(sys),finishRequested_(false),
       finished_(false),stopped_(false),stopRequested_(false){
    cout << (system_.mReady? "YES":"NO") << endl;
}

cv::Mat Detector::getImage(bool right=false){
  std::pair<cv::Mat, unsigned int> result;
  if(!system_.mReady)
      usleep(2000);
  Tracking* t = system_.mpTracker;
  if(!t->mImageReady)
      usleep(2000);
  result = t->getCurrentFrame(right);
  uint id = get<1>(result);
  processedFrames_.push_back(id);
  cout << "ID " << id << endl;
  return get<0>(result);
}



void Detector::Run(){
  finished_ = false;
  stopped_ = false;
  while(true){
      if(detectLeft){
        cv::Mat image = getImage();
        if(image.cols == 0)
            continue;
        detect(image);

//        detect(image);
    }
//    if(detectRight)
//      detect(getImage(true));

    if(Stop())
      while(isStopped())
        usleep(3000);

    if(CheckFinish()){
      cout << "Finished" << endl;
      break;
    }
  }
  cerr << "AH!" << endl;
}


void Detector::RequestFinish(){
    unique_lock<mutex> lock(mutexFinish_);
    finishRequested_ = true;
}

bool Detector::CheckFinish(){
    unique_lock<mutex> lock(mutexFinish_);
    return finishRequested_;
}

void Detector::SetFinish(){
    unique_lock<mutex> lock(mutexFinish_);
    finished_ = true;
}

bool Detector::isFinished(){
    unique_lock<mutex> lock(mutexFinish_);
    return finished_;
}

void Detector::RequestStop(){
    unique_lock<mutex> lock(mutexStop_);
    if(!stopped_)
        stopRequested_ = true;
}

bool Detector::isStopped(){
    unique_lock<mutex> lock(mutexStop_);
    return stopped_;
}

bool Detector::Stop(){
    unique_lock<mutex> lock(mutexStop_);
    unique_lock<mutex> lock2(mutexFinish_);

    if(finishRequested_)
        return false;
    else if(stopRequested_)
    {
        stopped_ = true;
        stopRequested_ = false;
        return true;
    }

    return false;

}

void Detector::Release(){
    unique_lock<mutex> lock(mutexStop_);
    stopped_ = false;
}
}
