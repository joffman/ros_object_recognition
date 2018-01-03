/** \file
 * \brief Implementation of the MorphologyFilter class.
 */

// std
#include <stdexcept>  // runtime_error

// ROS & OpenCV.
#include <opencv2/core/core.hpp>  // cv::Mat
#include <opencv2/imgproc/imgproc.hpp>  // morphologyEx()

// Our headers.
#include <morphology_filter/morphology_filter.h>
#include <morphology_filter/parameter_manager.h>

using namespace std;


namespace morphology_filter {

MorphologyFilter::MorphologyFilter(ParameterManager* pm)
    : param_manager_ {pm}
{
  if (!pm)
    throw runtime_error {"MorphologyFilter: nullptr given to constructor"};
}


cv::Mat MorphologyFilter::filter(const cv::Mat& image)
{
  cv::Mat filtered_image;
  cv::morphologyEx(image, filtered_image,
                   static_cast<int>(param_manager_->operation()),
                   cv::Mat{},         // default (3x3) kernel
                   cv::Point{-1, -1}, // default (centered) anchor
                   param_manager_->numIterations());
  return filtered_image;
}

} // morphology_filter
