#pragma once

#include <deque>

#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>

#include "ANMS_detector/detector.h"

#include "ANMS_detector/local_event_queues.h"
#include "ANMS_detector/distinct_queue.h"

#include <eigen3/Eigen/Dense>


namespace ANMS_detector
{

class evHarrisANMSDetector : public Detector
{
public:
  evHarrisANMSDetector(bool connect = true);
  virtual ~evHarrisANMSDetector();

  bool isFeature(const dvs_msgs::Event &e, int & F1, int & F2);
  double getLastScore() const {
    return last_score_;
  }

private:
  // methods
  void updateQueue(const int x, const int y, const dvs_msgs::Event &e);
  double getHarrisScore(int x, int y, bool polarity);

  // queues
  LocalEventQueues* queues_;

  // parameters
  int queue_size_;
  int window_size_;
  int kernel_size_;
  static const int sensor_width_ = 240;
  static const int sensor_height_ = 180;
  double harris_threshold_;

  double last_score_;

  // kernels
  Eigen::MatrixXd Gx_, h_;
  int factorial(int n) const;
  int pasc(int k, int n) const;
  
  // Start : Parameters For ANMS
  constexpr static const double filter_threshold_ = 0.050;
  Eigen::MatrixXd sae_[2];
  Eigen::MatrixXd sae_latest_[2];
  Eigen::MatrixXd ssae_[2];
  int nms_size_=2;
  int tau_size_=2;
  // End : Parameters For ANMS
};


} // namespace
