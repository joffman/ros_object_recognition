/** \file
 * \brief Definition of the DetectionDirector class.
 */

#ifndef OBJECT_DETECTION_3D_DETECTION_DIRECTOR_H
#define OBJECT_DETECTION_3D_DETECTION_DIRECTOR_H

// std.
#include <vector>

// ROS.
#include <ros/ros.h>

// Our headers.
#include "pcl_types.h"


namespace object_detection_3d {

class Filter;
class Detector;


/** \brief Director for coordinating the components of a 3D object recognition
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
  void addFilter(Filter* filter);
  /** \brief Sets the Detector component. */
  void setDetector(Detector* detector);

private:
  void cloudCallback(const PointCloudT::ConstPtr& cloud) const;

  // Data members.
  ros::NodeHandle node_handle_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
  std::vector<Filter*> filters_;
  Detector* detector_;
};

} // object_detection_3d

#endif  // OBJECT_DETECTION_3D_DETECTION_DIRECTOR_H
