/** \file
 * \brief Node for detecting objects with the shape-detection system,
 *  based on a given parameter-file.
 */

// std.
#include <iostream>
#include <exception>

// Our headers.
#include <object_detection_3d/detection_director.h>
#include <object_detection_3d/parameter_storage.h>
#include <distance_filter/distance_filter.h>
#include <distance_filter/groupbox.h>
#include <distance_filter/observed_parameter_manager.h>
#include <shape_detector/shape_detector.h>
#include <shape_detector/groupbox.h>
#include <shape_detector/observed_parameter_manager.h>

using object_detection_3d::DetectionDirector;
using object_detection_3d::ParameterStorage;
using distance_filter::DistanceFilter;
using shape_detector::ShapeDetector;


void printUsage(const std::string& program_name)
{
  std::cerr << "Usage: " << program_name << " <object-file>\n";
}


int main(int argc, char** argv)
try {
  // Initialize ROS.
  ros::init(argc, argv, "object_detection");

  // Check cmd-line args.
  if (argc != 2) {
    printUsage(argv[0]);
    return 0;
  }

  // Create distance-filter.
  distance_filter::ObservedParameterManager filter_pm;
  DistanceFilter filter {&filter_pm};

  // Create shape-detector.
  shape_detector::ObservedParameterManager detector_pm;
  ShapeDetector detector {&detector_pm};
  detector_pm.setObjectName("Garbage-Can");
    // TODO: The object-name should be contained in the file.

  // Create parameter-storage.
  ParameterStorage storage;
  storage.addParametrizable(&filter_pm);
  storage.addParametrizable(&detector_pm);
  if (!storage.loadParameters(argv[1])) {
    std::cerr << "Cannot load parameters from file " << argv[1] << '\n';
    return 1;
  }

  // Create detection-director.
  DetectionDirector director;
  director.addFilter(&filter);
  director.setDetector(&detector);

  // Event loop.
  ros::spin();
  return 0;
}
catch (const std::exception& e) {
  std::cerr << "\nstd::exception: " << e.what() << "\n\n";
  return 1;
}
catch (...) {
  std::cerr << "\nUnknown exception caught. Terminating program.\n\n";
  return 1;
}
