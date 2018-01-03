/** \file
 * \brief Implementation of the ROSFilterWrapper class.
 */

// std.
#include <stdexcept>  // runtime_error

// ROS & OpenCV.
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// Object-detection_*.
#include <object_detection_2d/ros_filter_wrapper.h>
#include <object_detection_2d/filter.h>


namespace object_detection_2d {

ROSFilterWrapper::ROSFilterWrapper(Filter* filter)
  : filter_ {filter}
{
  if (!filter)
    throw std::runtime_error {"ROSFilterWrapper: nullptr passed to constructor"};

  initRosCommunication();
}


void ROSFilterWrapper::initRosCommunication()
{
  sub_ = nh_.subscribe("input_image", 1,
                       &ROSFilterWrapper::imageCallback, this);
  pub_ = nh_.advertise<sensor_msgs::Image>("output_image", 10);
}


void ROSFilterWrapper::imageCallback(
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


/*
 * Convert message to cv-image, filter the image, convert it back to a message,
 * and publish it.
 */
void ROSFilterWrapper::processImageMsg(const sensor_msgs::Image::ConstPtr& image_msg) const
{
  cv_bridge::CvImagePtr cv_image =
      cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
  cv_image->image = filter_->filter(cv_image->image);
  pub_.publish(cv_image->toImageMsg());
}

} // object_detection_2d
