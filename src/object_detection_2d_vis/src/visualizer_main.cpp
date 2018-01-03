/** \file
 * \brief main function for running the Visualizer.
 */

// std.
#include <iostream>
#include <exception>

// Qt.
#include <QApplication>

// ROS.
#include <ros/ros.h>

// object_detection_*.
#include <object_detection_2d_vis/visualizer.h>

using object_detection_2d_vis::Visualizer;
using namespace std;


/** \brief Representation of the visualizer-application. */
class Application {
public:
  /** \brief Default constructor.
   *
   * Initializes the QApplication and Visualizer members.
   */
  Application(int argc, char** argv)
      : q_app_ {argc, argv} {}

  /** \brief Processes events until the node is shut down. */
  void run()
  {
    ros::Rate r {10};
    while (ros::ok()) {
      processEvents();
      r.sleep();
    }
  }

  /** \brief Destructor.
   *
   * Closes the gui if necessary.
   */
  ~Application()
  {
    q_app_.quit();
  }

private:
  /** \brief Processes ROS and Qt events. */
  void processEvents()
  {
    ros::spinOnce();
    q_app_.processEvents();

    // Shutdown ros when gui is closed.
    if (!vis_.isVisible())
      ros::shutdown();
  }

  // Data members.
  QApplication q_app_;
  Visualizer vis_;
};


/** \brief main function.
 *
 * Creates an Application object and lets it run.
 */
int main(int argc, char** argv)
try {
  ros::init(argc, argv, "visualizer");

  Application app {argc, argv};
  app.run();

  return 0;
}
catch (const std::exception& e) {
  cerr << "std::exception: " << e.what() << '\n';
  return 1;
}
catch (...) {
  cerr << "Unknown exception caught. Terminating program.\n";
  return 1;
}
