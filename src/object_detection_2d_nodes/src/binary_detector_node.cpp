/** \file
 * \brief main function for the binary_detector node.
 */

// std.
#include <iostream>
#include <exception>

// ROS.
#include <ros/ros.h>

// Headers of this project.
#include <object_detection_2d/ros_detector_wrapper.h>
#include <binary_detector/binary_detector.h>
#include <binary_detector/reconfigure_parameter_manager.h>

using object_detection_2d::ROSDetectorWrapper;
using binary_detector::BinaryDetector;
using binary_detector::ReconfigureParameterManager;
using namespace std;


void setupAndRun(int argc, char** argv)
{
  ros::init(argc, argv, "binary_detector");

  ReconfigureParameterManager pm;
  BinaryDetector detector {&pm};
  ROSDetectorWrapper wrapper {&detector};

  ros::spin();
}


int main(int argc, char** argv)
try
{
  setupAndRun(argc, argv);
  return 0;
}
catch (const exception& e)
{
  cerr << "std::exception: " << e.what() << '\n';
  return 1;
}
catch (...)
{
  cerr << "Unknown exception. Terminating program.\n";
  return 1;
}
