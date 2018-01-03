/** \file
 * \brief Definition of the DefinitionDirector class.
 */

#ifndef OBJECT_DETECTION_2D_DEFINITION_DIRECTOR_H
#define OBJECT_DETECTION_2D_DEFINITION_DIRECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>


namespace object_detection_2d {

class GUI;
class Filter;
class Detector;

/** \brief Director for coordinating the components that are involved in the
 *  creation of parameter-files (definition files) for an object recognition
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

  // TODO: gui_ & detector_ should be public. (?!)
  /** \brief Sets the GUI component. */
  void setGUI(GUI*);
  /** \brief Adds a Filter component. */
  void addFilter(Filter*);
  /** \brief Sets the Detector component. */
  void setDetector(Detector*);

private:
  void imageCallback(const sensor_msgs::Image::ConstPtr& image_msg) const;

  // Data members.
  ros::NodeHandle node_handle_;
  ros::Subscriber image_sub_;

  GUI* gui_ {nullptr};
  std::vector<Filter*> filters_;
  Detector* detector_ {nullptr};
};

} // object_detection_2d

#endif  // OBJECT_DETECTION_2D_DEFINITION_DIRECTOR_H
