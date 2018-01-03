/** \file
 * \brief Implementation of the DetectionDirector class.
 */

// OpenCV.
#include <cv_bridge/cv_bridge.h>

// ROS.
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// Headers of this project.
#include <object_detection_2d/detection_director.h>
#include <object_detection_2d/filter.h>
#include <object_detection_2d/detector.h>
#include <object_detection_2d_msgs/DetectedObject2DArray.h>

using object_detection_2d_msgs::DetectedObject2DArray;
using namespace std;


namespace object_detection_2d {

DetectionDirector::DetectionDirector()
{
  image_sub_ = node_handle_.subscribe("input_image", 1,
                                      &DetectionDirector::imageCallback, this);
  obj_pub_ =
      node_handle_.advertise<DetectedObject2DArray>("detected_objects_2d", 10);
}


void DetectionDirector::imageCallback(
    const sensor_msgs::Image::ConstPtr& image_msg) const
{
  if (!detector_)
    return;

  // Convert image.
  cv_bridge::CvImageConstPtr image_ptr;
  try {
    image_ptr =
        cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (const cv_bridge::Exception& e) {
    ROS_ERROR_STREAM("cv_bridge::Exception when converting image-msg: "
                     << e.what());
    return;
  }

  // Filter image.
  cv::Mat filtered_img {image_ptr->image.clone()};
  for (auto f : filters_)
    filtered_img = f->filter(filtered_img);

  // Detect objects.
  DetectedObject2DArray objects {detector_->detect(filtered_img)};
  if (!objects.objects.empty())
    obj_pub_.publish(objects);
}


void DetectionDirector::addFilter(Filter* f)
{
  if (f)
    filters_.push_back(f);
}


void DetectionDirector::setDetector(Detector* det)
{
  detector_ = det;
}

} // object_detection_2d
