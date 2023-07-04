#include <ros/ros.h>

#include "corner_event_detector/detector.h"
#include "corner_event_detector/evHarris_ANMS_detector.h"
#include "corner_event_detector/evFAST_ANMS_detector.h"
#include "corner_event_detector/ArcFAST_ANMS_detector.h"
#include "corner_event_detector/fa_harris_detector.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "corner_event_detector");

  // load parameter
  std::string feature_type;
  ros::param::param<std::string>("~feature_type", feature_type, "evHarris_ANMS");

  // create feature detecotr
  corner_event_detector::Detector* detector;
  if (feature_type == "evHarris_ANMS")
  {
    ROS_INFO("Using evHarris detector with ANMS.");
    detector = new corner_event_detector::evHarrisANMSDetector;
  }
  else if (feature_type == "evFAST_ANMS")
  {
    ROS_INFO("Using evFAST detector with ANMS.");
    detector = new corner_event_detector::evFASTANMSDetector;
  }
  else if (feature_type == "ArcFAST_ANMS")
  {
    ROS_INFO("Using ArcFAST detector with ANMS.");
    detector = new corner_event_detector::ArcFASTANMSDetector;
  }
  else if (feature_type == "FAHarris")
  {
    ROS_INFO("Using FAHarris detector with ANMS.");
    detector = new corner_event_detector::FAHarrisANMSDetector;
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
