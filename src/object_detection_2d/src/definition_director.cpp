/** \file
 * \brief Implementation of the DefinitionDirector class.
 */

// std.
#include <vector>

// OpenCV.
#include <cv_bridge/cv_bridge.h>

// ROS.
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// object_detection_*.
#include <object_detection_2d/definition_director.h>
#include <object_detection_2d/gui.h>
#include <object_detection_2d/detector.h>
#include <object_detection_2d/filter.h>
#include <object_detection_2d_msgs/DetectedObject2DArray.h>

using object_detection_2d_msgs::DetectedObject2DArray;
using namespace std;


namespace object_detection_2d {

DefinitionDirector::DefinitionDirector()
{
  image_sub_ = node_handle_.subscribe("input_image", 1,
                                      &DefinitionDirector::imageCallback,
                                      this);
}


void DefinitionDirector::imageCallback(
    const sensor_msgs::Image::ConstPtr& image_msg) const
{
  if (!gui_)
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
  if (detector_) {
    DetectedObject2DArray objects {detector_->detect(filtered_img)};

    // Draw object-polygons.
    vector<vector<cv::Point>> object_polygons;
    for (auto& obj : objects.objects) {
      vector<cv::Point> polygon;
      for (auto pt : obj.polygon.points)
        polygon.push_back(
            cv::Point{static_cast<int>(pt.x), static_cast<int>(pt.y)});
      object_polygons.push_back(polygon);
    }
    cv::polylines(filtered_img, object_polygons, true /* is_closed */,
                  CV_RGB(0x00, 0xff, 0x00), 3 /* thickness */);
      // TODO: Why don't we use cv::drawContours()? What's the difference?
  }
  // Show image( with objects).
  gui_->showImage(filtered_img);
}


void DefinitionDirector::setGUI(GUI* gui)
{
  gui_ = gui;
}


void DefinitionDirector::addFilter(Filter* f)
{
  if (f)
    filters_.push_back(f);
}


void DefinitionDirector::setDetector(Detector* det)
{
  detector_ = det;
}

} // object_detection_2d
