/** \file
 * \brief Definition of the ROSDetectorWrapper class.
 */

#ifndef OBJECT_DETECTION_3D_ROS_DETECTOR_WRAPPER_H
#define OBJECT_DETECTION_3D_ROS_DETECTOR_WRAPPER_H

// ROS.
#include <ros/ros.h>

// Our headers.
#include "pcl_types.h"


namespace object_detection_3d {

class Detector;


/** \brief ROS wrapper for objects of the Detector class.
 *
 * A ROSDetectorWrapper object integrates Detector objects into the ROS
 * framework by providing the necessary subscribers, publishers and event
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
  explicit ROSDetectorWrapper(Detector*);

private:
  void cloudCallback(const PointCloudT::ConstPtr& cloud);

  // Data members.
  ros::NodeHandle nh_;
  ros::Subscriber cloud_sub_;
  ros::Publisher objects_pub_;

  Detector* detector_;
};

} // object_detection_3d

#endif  // OBJECT_DETECTION_3D_ROS_DETECTOR_WRAPPER_H
