/** \file
 * \brief Node for creating parameter-files for the object recognition system
 *  that comprises the HSVFilter, the MorphologyFilter, and the BinaryDetector.
 */

// std.
#include <iostream>
#include <exception>

// ROS.
#include <ros/ros.h>

// Qt.
#include <QApplication>

// Headers of this project.
#include <object_detection_2d/gui.h>
#include <object_detection_2d/parameter_storage.h>
#include <object_detection_2d/definition_director.h>
#include <hsv_filter/hsv_filter.h>
#include <hsv_filter/groupbox.h>
#include <hsv_filter/observed_parameter_manager.h>
#include <morphology_filter/morphology_filter.h>
#include <morphology_filter/groupbox.h>
#include <morphology_filter/observed_parameter_manager.h>
#include <binary_detector/binary_detector.h>
#include <binary_detector/groupbox.h>
#include <binary_detector/observed_parameter_manager.h>

using object_detection_2d::GUI;
using object_detection_2d::ParameterStorage;
using object_detection_2d::DefinitionDirector;
using hsv_filter::HSVFilter;
using morphology_filter::MorphologyFilter;
using binary_detector::BinaryDetector;
using namespace std;


int main(int argc, char** argv)
try {
  // Initialization.
  ros::init(argc, argv, "hsv_system_definition");
  QApplication qapp {argc, argv};

  //
  // Set up the system.
  //

  // Create hsv-filter with groupbox.
  hsv_filter::ObservedParameterManager hsv_pm;
  HSVFilter hsv_filter {&hsv_pm};

  hsv_filter::GroupBox* hsv_groupbox {new hsv_filter::GroupBox};
  hsv_groupbox->setParameterManager(&hsv_pm);
  hsv_pm.addObserver(hsv_groupbox);

  // Create morphology-filter with groupbox.
  morphology_filter::ObservedParameterManager morphology_pm;
  MorphologyFilter morphology_filter {&morphology_pm};

  morphology_filter::GroupBox* morphology_groupbox
  {new morphology_filter::GroupBox};
  morphology_groupbox->setParameterManager(&morphology_pm);
  morphology_pm.addObserver(morphology_groupbox);

  // Create binary-detector with groupbox.
  binary_detector::ObservedParameterManager binary_pm;
  BinaryDetector binary_detector {&binary_pm};

  binary_detector::GroupBox* binary_groupbox {new binary_detector::GroupBox};
  binary_groupbox->setParameterManager(&binary_pm);
  binary_pm.addObserver(binary_groupbox);

  // Create parameter-storage.
  ParameterStorage ps;
  ps.addParametrizable(&hsv_pm);
  ps.addParametrizable(&morphology_pm);
  ps.addParametrizable(&binary_pm);

  // Create GUI.
  GUI gui;
  gui.setWindowTitle("HSV System Definition");
  gui.addRightWidget(hsv_groupbox);
  gui.addRightWidget(morphology_groupbox);
  gui.addRightWidget(binary_groupbox);
  gui.setParameterStorage(&ps);
  gui.show();

  // Create Director.
  DefinitionDirector director;
  director.addFilter(&hsv_filter);
  director.addFilter(&morphology_filter);
  director.setDetector(&binary_detector);
  director.setGUI(&gui);

  // Event loop.
  ros::Rate rate {10};
  while (ros::ok()) {
    ros::spinOnce();
    qapp.processEvents();

    if (!gui.isVisible())
      ros::shutdown();
    
    rate.sleep();
  }

  // Clean up.
  if (!ros::ok())
    qapp.quit();

  return 0;
}
catch (const exception& e) {
  cerr << "std::exception: " << e.what() << '\n';
  return 1;
}
catch (...) {
  cerr << "Unknown exception; terminating program.\n";
  return 1;
}
