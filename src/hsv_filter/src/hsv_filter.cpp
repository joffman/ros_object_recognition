/** \file
 * \brief Implementation of the HSVFilter class.
 */
// std.
#include <stdexcept>    // runtime_error

// ROS & OpenCV.
#include <opencv2/core/core.hpp>      // Mat, Scalar, inRange()
#include <opencv2/imgproc/imgproc.hpp>  // COLOR_*, cvtColor(),

// Our headers.
#include <hsv_filter/hsv_filter.h>
#include <hsv_filter/parameter_manager.h>

using namespace std;


namespace {

cv::Mat bgr2hsv(const cv::Mat& bgr_image)
{
  cv::Mat hsv_image;
  cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);
  return hsv_image;
}


cv::Mat gray2bgr(const cv::Mat& gray_image)
{
  cv::Mat bgr_image;
  cv::cvtColor(gray_image, bgr_image, cv::COLOR_GRAY2BGR);
  return bgr_image;
}

} // anonymous namespace


namespace hsv_filter {

HSVFilter::HSVFilter(ParameterManager* pm)
    : param_manager_{pm}
{
  if (!pm)
    throw runtime_error {"HSVFilter: nullptr given to constructor"};
}


cv::Mat HSVFilter::filter(const cv::Mat& bgr_image)
{
  cv::Mat hsv_image {bgr2hsv(bgr_image)};
  cv::Mat thresholded_image {threshold(hsv_image)};
  return gray2bgr(thresholded_image);
}


cv::Mat HSVFilter::threshold(const cv::Mat& hsv_image) const
{
  auto& pm {param_manager_};  // shorthand for convenience

  cv::Mat thresholded_image;
  cv::inRange(hsv_image,
              cv::Scalar(pm->hMin(), pm->sMin(), pm->vMin()),
              cv::Scalar(pm->hMax(), pm->sMax(), pm->vMax()),
              thresholded_image);
  return thresholded_image;
}

} // hsv_filter
