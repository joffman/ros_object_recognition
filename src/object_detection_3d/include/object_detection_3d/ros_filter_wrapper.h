/** \file
 * \brief Definition of the ROSFilterWrapper class.
 */

#ifndef OBJECT_DETECTION_3D_ROS_FILTER_WRAPPER_H
#define OBJECT_DETECTION_3D_ROS_FILTER_WRAPPER_H

// ROS.
#include <ros/ros.h>

// Our headers.
#include "pcl_types.h"


namespace object_detection_3d {

class Filter;


/** \brief ROS wrapper for objects of the Filter class.
 *
 * A ROSFilterWrapper object integrates Filter objects into the ROS
 * framework by providing the necessary subscribers, publishers and event
 * handlers.
 */
class ROSFilterWrapper {
public:
  /** \brief Sets the internal Filter-reference and initializes
   *  the ROS communication.
   *
   * \param[in] filter Pointer to the Filter object that is integrated
   *  into the ROS framework. The ROSFilterWrapper does not own this object.
   */
   explicit ROSFilterWrapper(Filter* filter);

private:
  void cloudCallback(const PointCloudT::ConstPtr& cloud);

  // Data members.
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  Filter* filter_;
};

} // object_detection_3d

#endif  // OBJECT_DETECTION_3D_ROS_FILTER_WRAPPER_H
