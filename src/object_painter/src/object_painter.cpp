/** \file
 * \brief Implementation of the ObjectPainter class.
 */

/*
 * TODO: Speed this up.
 *  This node became really slow after refactoring.
 *  One reason could be that there are more member functions now.
 *  Inlining those could help.
 *  Another reason could be the double-update of the objects, once in
 *  imageCallback and once in objectsCallback; previously the update was only
 *  done in the imageCallback.
 */

// std.
#include <vector>
#include <algorithm>  // find_if(), rotate()

// OpenCV.
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>        // Mat, Point
#include <opencv2/imgproc/imgproc.hpp>  // polylines()

// ROS.
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point32.h>

// Our headers.
#include <object_detection_2d_msgs/DetectedObject2D.h>
#include <object_detection_2d_msgs/DetectedObject2DArray.h>
#include <object_painter/object_painter.h>

using object_detection_2d_msgs::DetectedObject2D;
using object_detection_2d_msgs::DetectedObject2DArray;
using namespace std;


namespace {

inline cv::Point convertGeomPointToCvPoint(const geometry_msgs::Point32& obj_pt)
{
  return cv::Point{static_cast<int>(obj_pt.x), static_cast<int>(obj_pt.y)};
}


inline vector<cv::Point> createObjectPolygon(const DetectedObject2D& obj)
{
  vector<cv::Point> polygon;
  for (auto pt : obj.polygon.points)
    polygon.push_back(convertGeomPointToCvPoint(pt));
  return polygon;
}


inline vector<vector<cv::Point>> createObjectPolygons(
    const vector<DetectedObject2DArray::ConstPtr>& obj_arrays)
{
  vector<vector<cv::Point>> obj_polygons;
  for (const auto& arr : obj_arrays)
    for (const auto& obj : arr->objects)
      obj_polygons.push_back(createObjectPolygon(obj));
  return obj_polygons;
}


inline void drawObjectsIntoImage(
    const vector<DetectedObject2DArray::ConstPtr>& objects, cv::Mat& image)
{
  vector<vector<cv::Point>> object_polygons {createObjectPolygons(objects)};
  cv::polylines(image, object_polygons, true /* is_closed */,
                CV_RGB(0x00, 0xff, 0x00), 3 /* thickness */);
}


template <typename Vec>
inline void shiftVectorElementsToNewBeginning(
    Vec& vec, typename Vec::iterator new_beginning)
{
  auto new_sz = vec.end() - new_beginning;
  rotate(vec.begin(), new_beginning, vec.end());
  vec.resize(new_sz);
}

} // anonymous ns


namespace object_painter {

ObjectPainter::ObjectPainter()
{
  initROSCommunication();
}
 

void ObjectPainter::initROSCommunication()
{
  image_sub_ =
      nh_.subscribe("input_image", 1, &ObjectPainter::imageCallback, this);
  objects_sub_ =
      nh_.subscribe("input_objects", 10, &ObjectPainter::objectsCallback, this);
  image_pub_ = nh_.advertise<sensor_msgs::Image>("output_image", 10);
}


void ObjectPainter::imageCallback(const sensor_msgs::Image::ConstPtr& image_msg)
try {
  processImageMsg(image_msg);
}
catch (const cv_bridge::Exception& e) {
  ROS_ERROR_STREAM("cv_bridge::Exception when converting image-msg: "
                   << e.what());
}


void ObjectPainter::processImageMsg(const sensor_msgs::Image::ConstPtr& image_msg)
{
  cv_bridge::CvImage::Ptr cv_image {
    cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)};

  updateObjects();
  drawObjectsIntoImage(objects_, cv_image->image);

  image_pub_.publish(cv_image->toImageMsg());
}


void ObjectPainter::objectsCallback(
    const DetectedObject2DArray::ConstPtr& new_objects)
{
  objects_.push_back(new_objects);

  // Update our objects so that they do not accumulate when no images arrive.
  updateObjects();

  /* This would be faster:
   * if (100 < objects_.size)
   *    updateObjects();
   */
}


void ObjectPainter::updateObjects()
{
  auto young_enough_objects_beginning = findBeginningOfYoungObjects();
  shiftVectorElementsToNewBeginning(objects_, young_enough_objects_beginning);
}


vector<DetectedObject2DArray::ConstPtr>::iterator
ObjectPainter::findBeginningOfYoungObjects()
{
  auto current_time = ros::Time::now();

  auto youngEnough =
      [&current_time, this](const DetectedObject2DArray::ConstPtr& arr)
      { auto age = current_time - arr->header.stamp;
        return age <= param_manager_.maxAge(); };
  auto young_enough_objects_beginning =
      find_if(objects_.begin(), objects_.end(), youngEnough);

  return young_enough_objects_beginning;
}

} // object_painter
