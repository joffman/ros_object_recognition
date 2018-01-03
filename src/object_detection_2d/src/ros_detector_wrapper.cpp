/** \file
 * \brief Implementation of the ROSDetectorWrapper class.
 */

// std.
#include <stdexcept>  // runtime_error

// ROS & OpenCV.
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// Object-detection.
#include <object_detection_2d_msgs/DetectedObject2DArray.h>
#include <object_detection_2d/ros_detector_wrapper.h>
#include <object_detection_2d/detector.h>
#include <object_detection_2d/Rect2D.h>

using object_detection_2d_msgs::DetectedObject2DArray;
using object_detection_2d::Rect2D;


namespace object_detection_2d {

ROSDetectorWrapper::ROSDetectorWrapper(Detector* detector)
  : detector_ {detector}
{
  if (!detector)
    throw std::runtime_error {"ROSDetectorWrapper: nullptr passed to constructor"};

  initRosCommunication();
}


void ROSDetectorWrapper::initRosCommunication()
{
  image_sub_ =
      nh_.subscribe("input_image", 1, &ROSDetectorWrapper::imageCallback, this);
  selection_sub_ =
        nh_.subscribe("selection_area", 10,
                      &ROSDetectorWrapper::areaSelectionCallback, this);
  objects_pub_ =
      nh_.advertise<DetectedObject2DArray>("detected_objects_2d", 10);
}


void ROSDetectorWrapper::imageCallback(
    const sensor_msgs::Image::ConstPtr& image_msg) const
{
  try {
    processImageMsg(image_msg);
  }
  catch (const cv_bridge::Exception& e) {
    ROS_ERROR_STREAM("cv_bridge::Exception when converting image: "
                     << e.what());
    return;
  }
}


void ROSDetectorWrapper::processImageMsg(
    const sensor_msgs::Image::ConstPtr& image_msg) const
{
  cv_bridge::CvImageConstPtr cv_image =
      cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);

  DetectedObject2DArray objects = detector_->detect(cv_image->image);
  if (!objects.objects.empty())
    objects_pub_.publish(objects);
}


void ROSDetectorWrapper::areaSelectionCallback(const Rect2D::ConstPtr& rect)
  const
{
  cv::Rect cv_rect {rect->upper_left.x, rect->upper_left.y,
    rect->width, rect->height};
  detector_->processAreaSelection(cv_rect);
}

} // object_detection_2d
