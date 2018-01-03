/** \file
 * \brief Definition of the ROSDetectorWrapper class.
 */

#ifndef OBJECT_DETECTION_2D_ROS_DETECTOR_WRAPPER_H
#define OBJECT_DETECTION_2D_ROS_DETECTOR_WRAPPER_H

// ROS.
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

// object_detection_*.
#include <object_detection_2d/Rect2D.h>


namespace object_detection_2d {

class Detector;

/** \brief ROS wrapper for objects of the Detector class.
 *
 * A ROSDetectorWrapper object integrates Detector objects into the ROS
 * framework by providing the necessary subscribers, publishers, and event
 * handlers.
 */
class ROSDetectorWrapper {
public:
  /** \brief Sets the internal Detector-reference and initializes
   *  the ROS communication.
   *
   * \param[in] detector Valid pointer to the Detector object that is integrated
   *  into the ROS framework. The ROSDetectorWrapper does not own this object.
   */
  explicit ROSDetectorWrapper(Detector* detector);

private:
  void initRosCommunication();

  void imageCallback(const sensor_msgs::Image::ConstPtr& image_msg) const;
  void processImageMsg(const sensor_msgs::Image::ConstPtr& image_msg) const;

  void areaSelectionCallback(
      const object_detection_2d::Rect2D::ConstPtr& rect) const;

  // Data members.
  ros::NodeHandle nh_;
  ros::Subscriber image_sub_;
  ros::Subscriber selection_sub_;
  ros::Publisher objects_pub_;

  Detector* detector_;
};

} // object_detection_2d

#endif  // OBJECT_DETECTION_2D_ROS_DETECTOR_WRAPPER_H
