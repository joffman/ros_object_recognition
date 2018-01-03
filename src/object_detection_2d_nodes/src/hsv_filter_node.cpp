/** \file
 * \brief main function for the hsv_filter node.
 */

// std.
#include <iostream>
#include <exception>

// ROS.
#include <ros/ros.h>

// Headers of this project.
#include <object_detection_2d/ros_filter_wrapper.h>
#include <hsv_filter/hsv_filter.h>
#include <hsv_filter/reconfigure_parameter_manager.h>

using object_detection_2d::ROSFilterWrapper;
using hsv_filter::HSVFilter;
using hsv_filter::ReconfigureParameterManager;
using namespace std;


void setupAndRun(int argc, char** argv)
{
  ros::init(argc, argv, "hsv_filter");

  ReconfigureParameterManager pm;
  HSVFilter filter {&pm};
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
