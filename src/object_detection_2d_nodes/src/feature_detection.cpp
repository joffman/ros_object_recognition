/** \file
 * \brief Node for detecting objects with the feature-detection system,
 *  based on a given parameter-file.
 */

// std.
#include <iostream>
#include <exception>
#include <string>

// ROS.
#include <ros/ros.h>

// Headers of our project.
#include <object_detection_2d/parameter_storage.h>
#include <object_detection_2d/detection_director.h>
#include <feature_detector/feature_detector.h>
#include <feature_detector/observed_parameter_manager.h>

using object_detection_2d::ParameterStorage;
using object_detection_2d::DetectionDirector;
using feature_detector::FeatureDetector;
using feature_detector::ObservedParameterManager;

using namespace std;


void printUsage(const string& program_name)
{
  cerr << "Usage: " << program_name << " <parameter-file>\n";
}


int main(int argc, char** argv)
try {
  // Init ROS.
  ros::init(argc, argv, "feature_detection");

  if (argc != 2) {
    printUsage(argv[0]);
    return 0;
  }

  // Create components of the system.
  ObservedParameterManager pm;  // TODO: Should we use a special derived class for this?
  pm.setObjectName("dummy"); // TODO: Derive name from filename.
  FeatureDetector detector {&pm};

  ParameterStorage storage;
  storage.addParametrizable(&pm);
  if (!storage.loadParameters(argv[1])) {
    cerr << "Cannot load parameters from file '" << argv[1] << "'\n";
    return 1;
  }

  DetectionDirector director;
  director.setDetector(&detector);

  // Event loop.
  ros::spin();
  return 0;
}
catch (const exception& e) {
  cerr << "std::exception: " << e.what() << '\n';
  return 1;
}
catch (...) {
  cerr << "Unknown exception caught.\n";
  return 1;
}
