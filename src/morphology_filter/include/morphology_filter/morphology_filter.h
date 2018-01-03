/** \file
 * \brief Definition of the MorphologyFilter class.
 */

#ifndef MORPHOLOGY_FILTER_MORPHOLOGY_FILTER_H
#define MORPHOLOGY_FILTER_MORPHOLOGY_FILTER_H

// object-detection headers.
#include <object_detection_2d/filter.h>


namespace morphology_filter {

class ParameterManager;

/** \brief Filter for applying morphological transformations on images. */
class MorphologyFilter : public object_detection_2d::Filter {
public:
  /** \brief Creates a MorphologyFilter and initializes its parameter-manager.
   *
   * \param[in] pm Pointer to a valid ParameterManager.
   */
  explicit MorphologyFilter(ParameterManager* pm);

  /** \brief Applies a morphological transformation on an image.
   *
   * \param[in] image Image on which the morphological transformation is
   *  applied.
   * \return New image containing the transformed input image.
   */
  cv::Mat filter(const cv::Mat& image) override;

private:
  ParameterManager* param_manager_;
};

} // morphology_filter

#endif  // MORPHOLOGY_FILTER_MORPHOLOGY_FILTER_H
