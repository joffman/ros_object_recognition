/** \file
 * \brief Definition of the Filter class.
 */

#ifndef OBJECT_DETECTION_2D_FILTER_H
#define OBJECT_DETECTION_2D_FILTER_H

#include <opencv2/core/core.hpp>  // Mat


namespace object_detection_2d {

/** \brief Abstract filter for filtering an image. */
class Filter {
public:
  /** \brief Filters the given image.
   * \param[in] image Image that is filtered.
   * \return New image containing the filtered image.
   */
  virtual cv::Mat filter(const cv::Mat& image) = 0;
};

} // object_detection_2d

#endif  // OBJECT_DETECTION_2D_FILTER_H
