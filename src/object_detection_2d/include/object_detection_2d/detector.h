/** \file
 * \brief Definition of the Detector class.
 */

#ifndef OBJECT_DETECTION_2D_DETECTOR_H
#define OBJECT_DETECTION_2D_DETECTOR_H

#include <cv_bridge/cv_bridge.h>  // Mat, Rect
#include <object_detection_2d_msgs/DetectedObject2DArray.h>


namespace object_detection_2d {

/** \brief Abstract detector class for detecting objects in 2D images. */
class Detector {
public:
  /** \brief Detects objects in 2D image.
   *
   * \param[in] image Image in which the detector looks for objects.
   * \return Array containing the objects found.
   */
  virtual object_detection_2d_msgs::DetectedObject2DArray detect(
      const cv::Mat& image) = 0;

  /** \brief Callback function for processing an area of the image that was
   *  selected by the user.
   */
  virtual void processAreaSelection(const cv::Rect& rect) {}

  // TODO: Do we need a virtual destructor?
};

} // object_detection_2d

#endif  // OBJECT_DETECTION_2D_DETECTOR_H
