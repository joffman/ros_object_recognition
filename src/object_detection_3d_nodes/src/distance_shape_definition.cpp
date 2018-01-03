/** \file
 * \brief Node for creating parameter-files for the object recognition system
 *  that comprises the DistanceFilter and the ShapeDetector.
 */

// std.
#include <iostream>
#include <exception>

// Qt.
#include <QApplication>

// Our headers.
#include <object_detection_3d/gui.h>
#include <object_detection_3d/definition_director.h>
#include <object_detection_3d/parameter_storage.h>
#include <distance_filter/distance_filter.h>
#include <distance_filter/groupbox.h>
#include <distance_filter/observed_parameter_manager.h>
#include <shape_detector/shape_detector.h>
#include <shape_detector/groupbox.h>
#include <shape_detector/observed_parameter_manager.h>

using object_detection_3d::GUI;
using object_detection_3d::DefinitionDirector;
using distance_filter::DistanceFilter;
using shape_detector::ShapeDetector;
using object_detection_3d::ParameterStorage;


int main(int argc, char** argv)
try {
  // Initialize Qt and ROS.
  QApplication q_app {argc, argv};
  ros::init(argc, argv, "distance_shape_definition");

  // Create distance-filter with groupbox.
  distance_filter::GroupBox* filter_groupbox {new distance_filter::GroupBox};
  distance_filter::ObservedParameterManager filter_pm;
  filter_pm.addObserver(filter_groupbox);
  filter_groupbox->setParameterManager(&filter_pm);
  DistanceFilter filter {&filter_pm};

  // Create shape-detector with groupbox.
  shape_detector::GroupBox* detector_groupbox {new shape_detector::GroupBox};
  shape_detector::ObservedParameterManager detector_pm;
  detector_pm.addObserver(detector_groupbox);
  detector_groupbox->setParameterManager(&detector_pm);
  ShapeDetector detector {&detector_pm};

  // Create ParameterStorage.
  ParameterStorage storage;
  storage.addParametrizable(&filter_pm);
  storage.addParametrizable(&detector_pm);

  // Create gui.
  GUI gui;
  gui.setWindowTitle("Shape Detection - Object Definition");
  gui.addRightWidget(filter_groupbox);
  gui.addRightWidget(detector_groupbox);
  gui.setParameterStorage(&storage);
  gui.show();

  // Create DefinitionDirector.
  DefinitionDirector director;
  director.addFilter(&filter);
  director.setDetector(&detector);
  director.setGUI(&gui);

  // Event loop.
  ros::Rate r {10};
  while (ros::ok()) {
    ros::spinOnce();
    q_app.processEvents();

    // Shutdown ROS node if user closes the gui.
    if (!gui.isVisible())
      ros::shutdown();

    r.sleep();
  }

  // Quit Qt application if we exited the event loop because of shutting down
  // the ROS-node.
  if (!ros::ok())
    q_app.quit();

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
