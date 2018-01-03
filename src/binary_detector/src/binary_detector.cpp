/** \file
 * \brief Implementation of the BinaryDetector.
 */

// std.
#include <vector>
#include <string>
#include <stdexcept>  // runtime_error

// ROS & OpenCV.
#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <opencv2/core/core.hpp>  // Mat, Point, Size
#include <opencv2/imgproc/imgproc.hpp>    // cvtColor(), COLOR_*,
        // findContours(), convexHull()

// object_detection_*.
#include <object_detection_2d_msgs/DetectedObject2D.h>
#include <object_detection_2d_msgs/DetectedObject2DArray.h>

// binary_detector.
#include <binary_detector/binary_detector.h>
#include <binary_detector/parameter_manager.h>

using object_detection_2d_msgs::DetectedObject2D;
using object_detection_2d_msgs::DetectedObject2DArray;
using namespace std;


namespace binary_detector {

namespace {

geometry_msgs::Point32 convertCvPointToGeometryPoint(const cv::Point& cv_pt);


class ObjectBuilder {
public:
  void setName(const string& name) { obj_.name = name; }

  void setContour(const Contour& con)
  {
    for (const auto& con_pt : con) {
      geometry_msgs::Point32 obj_pt {convertCvPointToGeometryPoint(con_pt)};
      obj_.polygon.points.push_back(obj_pt);
    }
  }

  DetectedObject2D build() const { return obj_; }

private:
  DetectedObject2D obj_;
};


inline geometry_msgs::Point32 convertCvPointToGeometryPoint(const cv::Point& cv_pt)
{
  geometry_msgs::Point32 geom_pt;

  geom_pt.x = cv_pt.x;
  geom_pt.y = cv_pt.y;
  geom_pt.z = 0;

  return geom_pt;
}


class ObjectArrayBuilder {
public:
  void setObjectName(const string& name) { obj_name_ = name; }

  void buildObjectFromContour(const Contour& con)
  {
    ObjectBuilder builder;
    builder.setName(obj_name_);
    builder.setContour(con);
    objects_.objects.push_back(builder.build());
  }

  DetectedObject2DArray build()
  {
    fillHeader();
    return objects_;
  }

private:
  void fillHeader()
  {
    objects_.header.stamp = ros::Time::now();
    objects_.header.frame_id = "camera_rgb_frame";
  }

  // Data members.
  DetectedObject2DArray objects_;
  string obj_name_;
};

} // anonymous namespace


BinaryDetector::BinaryDetector(ParameterManager* pm)
  : param_manager_ {pm}
{
  if (!pm)
    throw runtime_error {"BinaryDetector: nullptr passed to constructor"};
}


namespace {
cv::Mat convertBgrToSingleChannel(const cv::Mat& bgr_image);
Contour approximateContour(const Contour& con);
}


DetectedObject2DArray BinaryDetector::detect(const cv::Mat& image)
  // TODO: This could be cleaner.
{
  cv::Mat single_channel_image {convertBgrToSingleChannel(image)};

  vector<Contour> contours;
  cv::findContours(single_channel_image, contours,
                   cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  ContourChecker checker {createContourCheckerFromImageSize(image.size())};

  ObjectArrayBuilder builder;
  builder.setObjectName(param_manager_->objectName());

  for (const auto& con : contours)
    if (checker.check(con)) {
      Contour approx {approximateContour(con)};
      builder.buildObjectFromContour(approx);
    }

  return builder.build();
}


namespace {

inline cv::Mat convertBgrToSingleChannel(const cv::Mat& bgr_image)
{
  cv::Mat single_channel_image;
  cv::cvtColor(bgr_image, single_channel_image,
               cv::COLOR_BGR2GRAY, 1 /*destination-channels*/);
  return single_channel_image;
}

} // anonymous namespace


ContourChecker BinaryDetector::createContourCheckerFromImageSize(
    const cv::Size& sz) const
{
  int image_length {sz.width + sz.height};

  ContourChecker checker;
  checker.setMinLength(param_manager_->minLengthFraction() * image_length);
  checker.setMaxLength(param_manager_->maxLengthFraction() * image_length);

  return checker;
}


namespace {

inline Contour approximateContour(const Contour& con)
{
  Contour hull;
  cv::convexHull(con, hull);
  return hull;
}

} // anonymous namespace

} // binary_detector
