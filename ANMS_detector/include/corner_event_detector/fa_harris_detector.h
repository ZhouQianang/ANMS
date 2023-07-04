/****************************************************************************************
*  FA-Harris corner detection. 
*
*  This method will save corner events to `/corner/scene_/fa_harris.txt` 
*  with (x, y, t, p). Modify `corner.launch` file to choose scene before `launch`.
*
*  Author: Ruoxiang Li
*  Date: 2019.1.20
*
****************************************************************************************/

#pragma once

#include <deque>

#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>

#include <eigen3/Eigen/Dense>

#include "corner_event_detector/detector.h"



namespace corner_event_detector
{

class FAHarrisANMSDetector : public Detector
{
public:
    FAHarrisANMSDetector(bool connect = true);
    virtual ~FAHarrisANMSDetector();

    bool isFeature(const dvs_msgs::Event &e, int & F1, int & F2);
    bool isCornerWithoutFilter(const dvs_msgs::Event &e);
    bool addNewEventToTimeSurface(const dvs_msgs::Event &e);
    bool isCornerCandidate(const dvs_msgs::Event &e);
    ////  
    double isCornerCandidateRefined(const dvs_msgs::Event &e);
    double getHarrisScore(int img_x, int img_y, bool polarity);
    bool checkPatch(const dvs_msgs::Event &e);
    Eigen::MatrixXi getPatch();

    bool isFiltered(const dvs_msgs::Event &e);
  
private:
    ///////////////////
    // Filter Parameters
    constexpr static const double filter_threshold_ = 0.050; //50 ms
    Eigen::MatrixXd sae_latest_[2];

    // Surface of Active Events
    Eigen::MatrixXd sae_[2];
    
    // Score of SAE
    Eigen::MatrixXd ssae_[2];
    int nms_size_=2;
    int tau_size_=2;

    ///////////////////
    // Circular Breshenham Masks
    const int circle3_[16][2];
    const int circle4_[20][2];    

    ///////////////////
    Eigen::MatrixXi patch_;

    // pixels on local window 9*9
    int latest_event_local_;

    int window_size_;
    int kernel_size_;
    static const int sensor_width_ = 640;
    static const int sensor_height_ = 480;
    double harris_threshold_;

    // kernels
    Eigen::MatrixXd Gx_, h_;
    int factorial(int n) const;
    int pasc(int k, int n) const;
};

}
