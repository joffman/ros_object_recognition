/** \file
 * \brief Definition of the HSVFilter class.
 */

#ifndef HSV_FILTER_HSV_FILTER_H
#define HSV_FILTER_HSV_FILTER_H

// ROS & OpenCV headers.
#include <opencv2/core/core.hpp>  // cv::Mat

// object-detection headers.
#include <object_detection_2d/filter.h>


namespace hsv_filter {

class ParameterManager;

/** \brief Filter for filtering images according to their HSV pixel-values. */
class HSVFilter : public object_detection_2d::Filter {
public:
  /** \brief Creates an HSVFilter and initializes its parameter-manager.
   *
   * \param[in] pm Pointer to a valid ParameterManager.
   */
  explicit HSVFilter(ParameterManager* pm);

  /** \brief Filters the given image.
   *
   * Only pixels whose Hue, Saturation and Value values lie inside
   * adjustable ranges pass through.
   *
   * \param[in] bgr_image Image that is filtered.
   * \return New image, containing the filtered image.
   */
  cv::Mat filter(const cv::Mat& bgr_image) override;

private:
  cv::Mat threshold(const cv::Mat& hsv_image) const;

  // Data members.
  ParameterManager* param_manager_;
};

} // hsv_filter

#endif  // HSV_FILTER_HSV_FILTER_H
