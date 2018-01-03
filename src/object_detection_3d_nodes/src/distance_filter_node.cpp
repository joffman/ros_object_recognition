/** \file
 * \brief main function for the distance_filter node.
 */

// std.
#include <iostream>
#include <exception>

// ROS.
#include <ros/ros.h>

// Our headers.
#include <object_detection_3d/ros_filter_wrapper.h>
#include <distance_filter/distance_filter.h>
#include <distance_filter/reconfigure_parameter_manager.h>

using object_detection_3d::ROSFilterWrapper;
using distance_filter::DistanceFilter;
using distance_filter::ReconfigureParameterManager;
using namespace std;


void setupAndRun(int argc, char** argv)
{
  ros::init(argc, argv, "distance_filter");

  ReconfigureParameterManager pm;
  DistanceFilter filter {&pm};
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
