/** \file
 * \brief Definition of the ObjectsToMarkers class.
 */

#ifndef OBJECTS_TO_MARKERS_OBJECTS_TO_MARKERS_H
#define OBJECTS_TO_MARKERS_OBJECTS_TO_MARKERS_H

// std.
#include <ros/ros.h>

// Our headers.
#include <object_detection_3d_msgs/DetectedObject3DArray.h>

#include "parameter_manager.h"


namespace objects_to_markers {

/** \brief Class for publishing object markers.
 *
 * ObjectsToMarkers objects subscribe to 3-dimensional detected objects
 * and publish Markers representing those objects.
 *
 * The markers are typically used for visualization in rviz.
 */

class ObjectsToMarkers {
public:
  /** \brief Default constructor.
   *
   * Initializes the ROS communication.
   */
  ObjectsToMarkers();

private:
  /** \brief Callback for incoming detected-objects-message.
   *
   * Receives incoming 3-dimensional objects and publishes markers representing
   * those objects.
   */
  void objectsCallback(
      const object_detection_3d_msgs::DetectedObject3DArray::ConstPtr& objects)
      const;

  // Data members.
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  ParameterManager param_manager_;
};

} // objects_to_markers

#endif  // OBJECTS_TO_MARKERS_OBJECTS_TO_MARKERS_H
