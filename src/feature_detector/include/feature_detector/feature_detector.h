/** \file
 * \brief Definition of the FeatureDetector class.
 */

#ifndef FEATURE_DETECTOR_FEATURE_DETECTOR_H
#define FEATURE_DETECTOR_FEATURE_DETECTOR_H

// std.
#include <vector>
#include <utility>  // pair

// OpenCV.
#include <opencv2/core/core.hpp>    // Mat, Rect KeyPoint, DMatch

// object_detection_*.
#include <object_detection_2d_msgs/DetectedObject2DArray.h>
#include <object_detection_2d_msgs/DetectedObject2D.h>
#include <object_detection_2d/detector.h>

// Headers of this package.
#include "parameter_manager.h"


namespace feature_detector {

/** \brief Struct for storing corresponding points that have been matched.
 *
 * MatchedPoints stores target-points and query-points that have been matched.
 * This means that there is a vector of matches, so that matches[i] represents
 * the match between target_pts[i] and query_pts[i].
 */
/*
 * TODO: It would probably be nicer to define the struct as
 *  struct MatchedPoints {cv::Point2f target_pt; cv::Point2f query_pt;};
 *  and then use a vector<MatchedPoints> in the code.
 */
struct MatchedPoints {
  std::vector<cv::Point2f> target_pts;
  std::vector<cv::Point2f> query_pts;
};


/** \brief Detector that uses features to detect objects in 2-dimensional
 * images.
 *
 * \note Currently only the usage of SURF features is available.
 */
class FeatureDetector : public object_detection_2d::Detector {
public:
  /** \brief Creates a FeatureDetector and initializes its parameter-manager.
   *
   * \param[in] pm Pointer to a valid ParameterManager.
   */
  explicit FeatureDetector(ParameterManager* pm);

  /** \brief Detects objects in 2D images, using features.
   *
   * The detector computes features (e.g. SURF) in the given image; let's call
   * them query-features. The query-features are than compared (matched) to 
   * a stored set of 'target-features' that were computed from a target.
   * If enough matches are found, it is assumed that the target is present in
   * the given image. In this case its location in the image is determined
   *  by computing the homography that transforms the points belonging to the
   * target-features into the points belonging to the query-features.
   *
   * \param[in] image Image in which we try to detect the object.
   * \return Array containing exactly one object if an object was detected.
   *  The array contains zero objects otherwise.
   */
  object_detection_2d_msgs::DetectedObject2DArray detect(const cv::Mat& image)
      override;
 
  /** \brief Stores target informations from the selected image area.
   *
   * The target that the detector tries to detect is selected by giving
   * the detector an area of the current image that contains the target.
   * This callback function processes such an area selection and stores
   * informations about the target. These informations can later be used
   * to detect the target in other images.
   *
   * \param[in] area Rectangular area in the current image that contains the
   *  target. It's best to select an area that is rich in features.
   */
  void processAreaSelection(const cv::Rect& area) override;

private:
  bool lookingForTarget() const
  { return !param_manager_->targetKeypoints().empty(); }
    // TODO: Give param_manager_ an appropriate function for that
    //  (Law of Demeter).
    // TODO: Check if target_keypoints are valid/sufficient.

  std::pair<object_detection_2d_msgs::DetectedObject2D, bool>
      tryDetectObject(const cv::Mat& image) const;

  object_detection_2d_msgs::DetectedObject2D detectObject(
    const std::vector<cv::DMatch>& matches,
    const std::vector<cv::KeyPoint>& query_keypoints) const;

  void fillObjectsHeader(
      object_detection_2d_msgs::DetectedObject2DArray& objects) const;

  // Data members.
  ParameterManager* param_manager_;
};

} // feature_detector

#endif  // FEATURE_DETECTOR_FEATURE_DETECTOR_H
