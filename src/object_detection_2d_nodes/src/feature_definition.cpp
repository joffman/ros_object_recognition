/** \file
 * \brief Node for creating parameter-files for the feature-detection system.
 */

// std.
#include <iostream>
#include <exception>
#include <functional>   // bind()

// ROS.
#include <ros/ros.h>

// Qt.
#include <QApplication>

// Headers of this project.
#include <object_detection_2d/gui.h>
#include <object_detection_2d/parameter_storage.h>
#include <object_detection_2d/definition_director.h>
#include <feature_detector/feature_detector.h>
#include <feature_detector/groupbox.h>
#include <feature_detector/observed_parameter_manager.h>

using object_detection_2d::GUI;
using object_detection_2d::ParameterStorage;
using object_detection_2d::DefinitionDirector;
using feature_detector::FeatureDetector;
using feature_detector::GroupBox;
using feature_detector::ObservedParameterManager;
using AreaSelectionCallback = GUI::AreaSelectionCallback;
using namespace std;


int main(int argc, char** argv)
try {
  // Initialization.
  ros::init(argc, argv, "feature_definition");
  QApplication app {argc, argv};

  // Create detector with groupbox.
  ObservedParameterManager pm;
  FeatureDetector detector {&pm};
  GroupBox* groupbox {new GroupBox};
  groupbox->setParameterManager(&pm);
  pm.addObserver(groupbox);

  // Create parameter-storage.
  ParameterStorage ps;
  ps.addParametrizable(&pm);

  // Create gui.
  GUI gui;
  gui.setWindowTitle("Feature Definition");
  gui.addRightWidget(groupbox);
  gui.setParameterStorage(&ps);
  gui.registerAreaSelectionCallback(
      AreaSelectionCallback{
      bind(&FeatureDetector::processAreaSelection, &detector, placeholders::_1)
      });
  gui.show();

  // Create director.
  DefinitionDirector director;
  director.setDetector(&detector);
  director.setGUI(&gui);

  // Event loop.
  ros::Rate rate {10};
  while (ros::ok()) {
    ros::spinOnce();
    app.processEvents();

    if (!gui.isVisible())
      ros::shutdown();

    rate.sleep();
  }

  // Clean up.
  if (!ros::ok())
    app.quit();

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
