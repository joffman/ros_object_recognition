/** \file
 * \brief main function for the feature_detector node.
 */

// std.
#include <iostream>
#include <exception>

// ROS.
#include <ros/ros.h>

// Object-detection.
#include <object_detection_2d/ros_detector_wrapper.h>
#include <feature_detector/feature_detector.h>
#include <feature_detector/reconfigure_parameter_manager.h>

using object_detection_2d::ROSDetectorWrapper;
using feature_detector::FeatureDetector;
using feature_detector::ReconfigureParameterManager;
using namespace std;


void setupAndRun(int argc, char** argv)
{
  ros::init(argc, argv, "feature_detector");

  ReconfigureParameterManager pm;
  FeatureDetector detector {&pm};
  ROSDetectorWrapper wrapper {&detector};

  ros::spin();
}


int main(int argc, char** argv)
try {
  setupAndRun(argc, argv);
  return 0;
}
catch (const exception& e) {
  cerr << "std::exception: " << e.what() << '\n';
  return 1;
}
catch (...) {
  cerr << "Unknown exception. Terminating program.\n";
  return 1;
}
