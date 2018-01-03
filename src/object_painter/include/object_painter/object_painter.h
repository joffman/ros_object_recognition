/** \file
 * \brief Definition of the ObjectPainter class.
 */

#ifndef OBJECT_PAINTER_OBJECT_PAINTER_H
#define OBJECT_PAINTER_OBJECT_PAINTER_H

// std.
#include <vector>

// ROS.
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

// Our packages.
#include <object_detection_2d_msgs/DetectedObject2DArray.h>
#include "parameter_manager.h"


namespace object_painter {

/** \brief The ObjectPainter draws 2D objects into images.
 *
 * The ObjectPainter subscribes to images and 2D detected objects.
 * When a new image is received, it draws the polygons representing the objects
 * into it and publishes the resulting image.
 *
 * Only objects up to an adjustable maximum age are retained and drawn into
 * images.
 */
class ObjectPainter {
public:
  /** \brief Initializes the ROS communication. */
  ObjectPainter();

private:
  void initROSCommunication();

  void imageCallback(const sensor_msgs::Image::ConstPtr& image_msg);
  void processImageMsg(const sensor_msgs::Image::ConstPtr& image_msg);

  void objectsCallback(
      const object_detection_2d_msgs::DetectedObject2DArray::ConstPtr& objects);
  
  /** \brief Modify objects_ so that it only contains objects that are not older than
   *  the max-age parameter.
   */
  void updateObjects();

  std::vector<object_detection_2d_msgs::DetectedObject2DArray::ConstPtr>::iterator
      findBeginningOfYoungObjects();
  
  // Data members.
  ros::NodeHandle nh_;
  ros::Subscriber image_sub_;
  ros::Subscriber objects_sub_;
  ros::Publisher image_pub_;

  ParameterManager param_manager_;
  std::vector<object_detection_2d_msgs::DetectedObject2DArray::ConstPtr>
      objects_;
    // objects_ stores objects with descending age.
    // We assume that the time-stamp given in the header of each incoming
    // object-array is approximately equal to the moment of the message-arrival.
};

} // object_painter

#endif  // OBJECT_PAINTER_OBJECT_PAINTER_H
