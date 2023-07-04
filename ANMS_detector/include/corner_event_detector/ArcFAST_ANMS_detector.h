#pragma once

#include <deque>

#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>

#include <eigen3/Eigen/Dense>

#include "corner_event_detector/detector.h"

namespace corner_event_detector
{

class ArcFASTANMSDetector : public Detector
{
public:
  ArcFASTANMSDetector(bool connect = true);
  virtual ~ArcFASTANMSDetector();

  bool isFeature(const dvs_msgs::Event &e, int & F1, int & F2);

private:
  // SAE and SAE*
  Eigen::MatrixXd sae_[2];
  constexpr static const double filter_threshold_ = 0.050;
  Eigen::MatrixXd sae_latest_[2];
  
  //code for ANMS
  Eigen::MatrixXd ssae_[2];
  int nms_size_=2;
  int tau_size_=2;
  

  // pixels on circle
  int circle3_[16][2];
  int circle4_[20][2];

  // parameters
  static const int sensor_width_ = 640;
  static const int sensor_height_ = 480;
};


} // namespace
