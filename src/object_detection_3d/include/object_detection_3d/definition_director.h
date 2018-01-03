/** \file
 * \brief Definition of the DefinitionDirector class.
 */

#ifndef OBJECT_DETECTION_3D_DEFINITION_DIRECTOR_H
#define OBJECT_DETECTION_3D_DEFINITION_DIRECTOR_H

// std.
#include <vector>

// ROS.
#include <ros/ros.h>

// Our headers.
#include "pcl_types.h"


namespace object_detection_3d {

class Filter;
class Detector;
class GUI;


/** \brief Director for coordinating the components that are involved in the
 *  creation of parameter-files (definition files) for a 3D object recognition
 *  system.
 *
 * An object of this class keeps references to the components of the system.
 * It receives images over the ROS network in one of its member functions
 * and passes data between the components.
 */
class DefinitionDirector {
public:
  /** \brief Sets up the ROS communication. */
  DefinitionDirector();

  /** \brief Sets the GUI component. */
  void setGUI(GUI*);
  /** \brief Adds a Filter component. */
  void addFilter(Filter*);
  /** \brief Sets the Detector component. */
  void setDetector(Detector*);

private:
  void cloudCallback(const PointCloudT::ConstPtr& cloud) const;

  // Data members.
  ros::NodeHandle node_handle_;
  ros::Subscriber subscriber_;
  GUI* gui_;
  std::vector<Filter*> filters_;
  Detector* detector_;
};

} // object_detection_3d

#endif  // OBJECT_DETECTION_3D_DEFINITION_DIRECTOR_H
