/** \file
 * \brief main-function for creating an ObjectsToMarkers object and waiting
 *  for ROS events.
 */

// std.
#include <iostream>
#include <exception>

// ROS & Co.
#include <ros/ros.h>
#include <objects_to_markers/objects_to_markers.h>

using objects_to_markers::ObjectsToMarkers;
using namespace std;


void setupAndRun(int argc, char** argv)
{
  ros::init(argc, argv, "objects_to_markers");
  ObjectsToMarkers otm;

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
