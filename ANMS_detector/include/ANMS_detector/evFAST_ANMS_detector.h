#pragma once

#include <deque>

#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>

#include <eigen3/Eigen/Dense>

#include "ANMS_detector/detector.h"

namespace ANMS_detector
{

class evFASTANMSDetector : public Detector
{
public:
  evFASTANMSDetector(bool connect = true);
  virtual ~evFASTANMSDetector();

  bool isFeature(const dvs_msgs::Event &e, int & F1, int & F2);

private:
  // SAE
  Eigen::MatrixXd sae_[2];
  
  //code for ANMS
  constexpr static const double filter_threshold_ = 0.050;
  Eigen::MatrixXd sae_latest_[2];
  Eigen::MatrixXd ssae_[2];
  int nms_size_=3;
  int tau_size_=2;
  

  // pixels on circle
  int circle3_[16][2];
  int circle4_[20][2];

  // parameters
  static const int sensor_width_ = 240;
  static const int sensor_height_ = 180;
};


} // namespace
