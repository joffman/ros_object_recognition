/** \file
 * \brief main function for the morphology_filter node.
 */

// std.
#include <iostream>
#include <exception>

// ROS.
#include <ros/ros.h>

// object-detection.
#include <object_detection_2d/ros_filter_wrapper.h>
#include <morphology_filter/morphology_filter.h>
#include <morphology_filter/reconfigure_parameter_manager.h>

using object_detection_2d::ROSFilterWrapper;
using morphology_filter::MorphologyFilter;
using morphology_filter::ReconfigureParameterManager;
using namespace std;


void setupAndRun(int argc, char** argv)
{
  ros::init(argc, argv, "morphology_filter");

  ReconfigureParameterManager pm;
  MorphologyFilter filter {&pm};
  ROSFilterWrapper wrapper {&filter};

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
