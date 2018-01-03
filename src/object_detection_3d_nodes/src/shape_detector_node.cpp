/** \file
 * \brief main function for the shape_detector node.
 */

// std.
#include <iostream>
#include <exception>

// Our headers.
#include <object_detection_3d/ros_detector_wrapper.h>
#include <shape_detector/shape_detector.h>
#include <shape_detector/reconfigure_parameter_manager.h>

using object_detection_3d::ROSDetectorWrapper;
using shape_detector::ShapeDetector;
using shape_detector::ReconfigureParameterManager;
using namespace std;


void setupAndRun(int argc, char** argv)
{
  ros::init(argc, argv, "shape_detector");

  ReconfigureParameterManager pm;
  ShapeDetector detector {&pm};
  ROSDetectorWrapper wrapper {&detector};

  ros::spin();
}


int main(int argc, char** argv)
try {
  setupAndRun(argc, argv);
  return 0;
}
catch (const std::exception& e) {
  cerr << "std::exception: " << e.what() << '\n';
  return 1;
}
catch (...) {
  cerr << "Unknown exception. Terminating program.\n";
  return 1;
}
