/** \file
 * \brief Implementation of the ROSDetectorWrapper class.
 */

// std.
#include <stdexcept>  // runtime_error

// Our headers.
#include <object_detection_3d/ros_detector_wrapper.h>
#include <object_detection_3d/detector.h>
#include <object_detection_3d_msgs/DetectedObject3DArray.h>


namespace object_detection_3d {

ROSDetectorWrapper::ROSDetectorWrapper(Detector* det)
    : detector_ {det}
{
  if (!det)
    throw std::runtime_error {"ROSDetectorWrapper: nullptr passed to constructor"};

  cloud_sub_ =
      nh_.subscribe("input_points", 1,
                    &ROSDetectorWrapper::cloudCallback, this);
  objects_pub_ =
      nh_.advertise<object_detection_3d_msgs::DetectedObject3DArray>(
          "detected_objects_3d", 10);
}


void ROSDetectorWrapper::cloudCallback(const PointCloudT::ConstPtr& cloud)
{
  object_detection_3d_msgs::DetectedObject3DArray objects {
    detector_->detect(cloud)};
  if (!objects.objects.empty())
    objects_pub_.publish(objects);
}

} // object_detection_3d
