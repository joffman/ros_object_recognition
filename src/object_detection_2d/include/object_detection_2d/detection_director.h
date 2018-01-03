/** \file
 * \brief Definition of the DetectionDirector class.
 */

#ifndef OBJECT_DETECTION_2D_DETECTION_DIRECTOR_H
#define OBJECT_DETECTION_2D_DETECTION_DIRECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>


namespace object_detection_2d {

class Filter;
class Detector;


/** \brief Director for coordinating the components of a 2D object recognition
 *  system that are involved in the detection of objects based on a given
 *  parameter-file.
 *
 * An object of this class keeps references to the components of the system.
 * It receives images over the ROS network in one of its member functions,
 * passes data between the components, and publishes the detected objects.
 */
class DetectionDirector {
public:
  /** \brief Sets up the ROS communication. */
  DetectionDirector();

  /** \brief Adds a Filter component. */
  void addFilter(Filter*);
  /** \brief Sets the Detector component. */
  void setDetector(Detector*);

private:
  void imageCallback(const sensor_msgs::Image::ConstPtr& image_msg) const;

  // Data members.
  ros::NodeHandle node_handle_;
  ros::Subscriber image_sub_;
  ros::Publisher obj_pub_;

  std::vector<Filter*> filters_;
  Detector* detector_ {nullptr};
};

} // object_detection_2d

#endif  // OBJECT_DETECTION_2D_DETECTION_DIRECTOR_H
