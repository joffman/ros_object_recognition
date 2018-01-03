/** \file
 * \brief Node for detecting objects with the hsv-detection system,
 *  based on a given parameter-file.
 */

// std.
#include <iostream>
#include <exception>
#include <string>

// ROS.
#include <ros/ros.h>

// Headers of this project.
#include <object_detection_2d/parameter_storage.h>
#include <object_detection_2d/detection_director.h>
#include <hsv_filter/hsv_filter.h>
#include <hsv_filter/observed_parameter_manager.h>
#include <morphology_filter/morphology_filter.h>
#include <morphology_filter/observed_parameter_manager.h>
#include <binary_detector/binary_detector.h>
#include <binary_detector/observed_parameter_manager.h>

using object_detection_2d::ParameterStorage;
using object_detection_2d::DetectionDirector;
using hsv_filter::HSVFilter;
using morphology_filter::MorphologyFilter;
using binary_detector::BinaryDetector;

using namespace std;


void printUsage(const string& program_name)
{
  cerr << "Usage: " << program_name << " <parameter-file>\n";
}


int main(int argc, char** argv)
try {
  // Init ROS.
  ros::init(argc, argv, "hsv_detection");

  if (argc != 2) {
    printUsage(argv[0]);
    return 0;
  }

  // Create components of the system.
  hsv_filter::ObservedParameterManager hsv_pm;
  HSVFilter hsv_filter {&hsv_pm};

  morphology_filter::ObservedParameterManager morph_pm;
  MorphologyFilter morph_filter {&morph_pm};

  binary_detector::ObservedParameterManager binary_pm;
  binary_pm.setObjectName("dummy"); // TODO
  BinaryDetector detector {&binary_pm};

  ParameterStorage storage;
  storage.addParametrizable(&hsv_pm);
  storage.addParametrizable(&morph_pm);
  storage.addParametrizable(&binary_pm);
  if (!storage.loadParameters(argv[1])) {
    cerr << "Cannot load parameters from file '" << argv[1] << "'\n";
    return 1;
  }

  DetectionDirector director;
  director.addFilter(&hsv_filter);
  director.addFilter(&morph_filter);
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
