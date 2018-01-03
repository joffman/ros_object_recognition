/** \file
 * \brief Definition of the BinaryDetector class.
 */

#ifndef BINARY_DETECTOR_BINARY_DETECTOR_H
#define BINARY_DETECTOR_BINARY_DETECTOR_H

// std.
#include <vector>

// OpenCV.
#include <opencv2/core/core.hpp>    // Size, Point, Mat
#include <opencv2/imgproc/imgproc.hpp>  // arcLength()

// object_detection_*.
#include <object_detection_2d/detector.h>
#include <object_detection_2d_msgs/DetectedObject2DArray.h>


namespace binary_detector {

using Contour = std::vector<cv::Point>; // using in header??!

class ParameterManager;


/** \brief Class for checking if a contour is valid, i.e. represents
 *  an object.
 */
// Should this class be defined in its own file?!
class ContourChecker {
public:
  /** \brief Checks if the given contour is valid.
   *
   * Currently the check only examines the length of the contour.
   * The contour is considered valid, if its length lies within
   * a specific range.
   *
   * \param[in] con Contour that is checked.
   * \return true if con is valid; false otherwise.
   */ 
  bool check(const Contour& con) const;


  /** \brief Sets the lower limit for the length of valid contours. */
  void setMinLength(double val) { min_length_ = val; }

  /** \brief Sets the upper limit for the length of valid contours. */
  void setMaxLength(double val) { max_length_ = val; }

private:
  double min_length_;
  double max_length_;
};


inline bool ContourChecker::check(const Contour& con) const
{
  const double con_length {cv::arcLength(con, true /* closed */)};
  return min_length_<=con_length && con_length<=max_length_;
}


/** \brief Detector for detecting objects in binary images. */
class BinaryDetector : public object_detection_2d::Detector {
public:
  /** \brief Creates a BinaryDetector and initializes its parameter-manager.
   *
   * \param[in] pm Pointer to a valid ParameterManager.
   */
  explicit BinaryDetector(ParameterManager* pm);

  /** \brief Detects objects in given binary image.
   *
   * The input image is interpreted as a binary image by converting it to
   * grayscale and then only distinguishing between pixels with value zero
   * and pixels with non-zero values.
   * Objects are approximated by the contours of the remaining (i.e. non-zero)
   * pixels. To be considered as an object, a contour has to pass the check
   * of the ContourChecker.
   *
   * \param[in] image Image in which the detector searches for objects.
   * \return Array of detected objects.
   */
  object_detection_2d_msgs::DetectedObject2DArray detect(const cv::Mat& image)
      override;
 
private:
  ContourChecker createContourCheckerFromImageSize(const cv::Size& sz) const;

  // Data members.
  ParameterManager* param_manager_;
};

} // binary_detector

#endif  // BINARY_DETECTOR_BINARY_DETECTOR_H
