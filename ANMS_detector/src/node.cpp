#include <ros/ros.h>

#include "ANMS_detector/detector.h"
#include "ANMS_detector/evHarris_ANMS_detector.h"
#include "ANMS_detector/evFAST_ANMS_detector.h"
#include "ANMS_detector/ArcFAST_ANMS_detector.h"
#include "ANMS_detector/fa_harris_detector.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ANMS_detector");

  // load parameter
  std::string feature_type;
  ros::param::param<std::string>("~feature_type", feature_type, "evHarris_ANMS");

  // create feature detecotr
  ANMS_detector::Detector* detector;
  if (feature_type == "evHarris_ANMS")
  {
    ROS_INFO("Using evHarris detector with ANMS.");
    detector = new ANMS_detector::evHarrisANMSDetector;
  }
  else if (feature_type == "evFAST_ANMS")
  {
    ROS_INFO("Using evFAST detector with ANMS.");
    detector = new ANMS_detector::evFASTANMSDetector;
  }
  else if (feature_type == "ArcFAST_ANMS")
  {
    ROS_INFO("Using ArcFAST detector with ANMS.");
    detector = new ANMS_detector::ArcFASTANMSDetector;
  }
  else if (feature_type == "FAHarris")
  {
    ROS_INFO("Using FAHarris detector with ANMS.");
    detector = new ANMS_detector::FAHarrisANMSDetector;
  }
  else
  {
    ROS_ERROR("Feature type '%s' is unknown.", feature_type.c_str());
    return 1;
  }

  // run
  ros::spin();

  delete detector;

  return 0;
}
