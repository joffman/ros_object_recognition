/** \file
 * \brief Definition of the ParameterManager class.
 */

/* TODO: setSelectionImage and setSelectionArea were only created to
 *  enable the ObservedParameterStorage object to store them in its
 *  setParametersFromStorage method.
 *  In general, the seperation of concerns between ParameterManager and
 *  its children is not clear.
 */

#ifndef FEATURE_DETECTOR_PARAMETER_MANAGER_H
#define FEATURE_DETECTOR_PARAMETER_MANAGER_H

// std.
#include <string>
#include <vector>
#include <memory>   // unique_ptr

// OpenCV.
#include <opencv2/core/core.hpp>    // Mat, Rect, KeyPoint, Ptr
#include <opencv2/features2d/features2d.hpp>  // DescriptorMatcher
#include <opencv2/xfeatures2d.hpp>


namespace feature_detector {

/** \brief Parameter-manager for FeatureDetector objects.
 *
 * A ParameterManager object manages the parameters of
 * a FeatureDetector object and provides getter- and setter-methods for
 * accessing those parameters.
 */
class ParameterManager {
public:
  /** \brief Enumeration for values of the norm-type parameter. */
  enum class NormType {
    l1 = cv::NORM_L1,
    l2 = cv::NORM_L2
  };

  //
  // Static consts.
  //

  /** \brief Lower limit for the hessian-threshold parameter. */
  static constexpr double min_hessian_thresh {2.};
  /** \brief Upper limit for the hessian-threshold parameter. */
  static constexpr double max_hessian_thresh {10000.};

  /** \brief Lower limit for the number-octaves parameter. */
  static constexpr int min_num_octaves {1};
  /** \brief Upper limit for the number-octaves parameter. */
  static constexpr int max_num_octaves {20};

  /** \brief Lower limit for the number-octave-layers parameter. */
  static constexpr int min_num_octave_layers {1};
  /** \brief Upper limit for the number-octave-layers parameter. */
  static constexpr int max_num_octave_layers {20};

  /** \brief Lower limit for the match-distance-threshold parameter. */
  static constexpr double min_match_distance_thresh {0.1};
  /** \brief Upper limit for the match-distance-threshold parameter. */
  static constexpr double max_match_distance_thresh {100.};

  /** \brief Lower limit for the ransac-reprojection-threshold parameter. */
  static constexpr double min_ransac_reproj_thresh {0.5};
  /** \brief Upper limit for the ransac-reprojection-threshold parameter. */
  static constexpr double max_ransac_reproj_thresh {100.};


  /** \brief Processes the given area in the current image to compute
   *  and store informations about the target.
   *
   * \param[in] area Rectangular area in the last stored image that represents
   *  the target.
   *
   * \sa FeatureDetector::processAreaSelection
   * \todo It would be nice if the selection, which is passed in as a parameter,
   *  would also contain the relevant part of the image itself.
   *  This would avoid the need for storing a copy of the most recent image.
   */
  void processAreaSelection(const cv::Rect& area);

  /** \brief Stores the given image.
   *
   * The last image stored is used to compute target features when a
   * selection-area is received.
   *
   * \sa ParameterManager::processAreaSelection
   */
  void storeImage(const cv::Mat& image) { last_image_ = image.clone(); }


  //
  // Setters & getters. TODO: Maybe make setters protected.
  //
  std::string objectName() const { return object_name_; }
  void setObjectName(const std::string&);

  double hessianThreshold() const { return surf_->getHessianThreshold(); }
  void setHessianThreshold(double);

  int numOctaves() const { return surf_->getNOctaves(); }
  void setNumOctaves(int);

  int numOctaveLayers() const { return surf_->getNOctaveLayers(); }
  void setNumOctaveLayers(int);

  bool extended() const { return surf_->getExtended(); }
  void setExtended(bool);

  bool upright() const { return surf_->getUpright(); }
  void setUpright(bool);

  NormType normType() const { return norm_type_; }
  void setNormType(NormType);

  bool crossCheck() const { return cross_check_; }
  void setCrossCheck(bool);

  double matchDistanceThreshold() const { return match_distance_thresh_; }
  void setMatchDistanceThreshold(double);

  double ransacReprojThreshold() const { return ransac_reproj_thresh_; }
  void setRansacReprojThreshold(double);

  cv::xfeatures2d::SURF* surf() const { return surf_.get(); }

  cv::DescriptorMatcher* matcher() const { return matcher_.get(); }

  cv::Rect selectionArea() const { return selection_area_; }
  // void setSelectionArea(const cv::Rect&);

  cv::Mat selectionImage() const { return selection_image_; }
  // void setSelectionImage(const cv::Mat&);

  void setSelection(const cv::Mat& selection_image,
                    const cv::Rect& selection_rect);

  std::vector<cv::KeyPoint> targetKeypoints() const
  { return target_keypoints_; } // TODO: is a pointer to the vector sufficient?

  cv::Mat targetDescriptors() const { return target_descriptors_; }

private:
  void saveSelection(const std::string& filename) const;
  void loadSelection(const std::string& filename);

  void updateFeaturesAndMatcher();
  void computeTargetFeatures();
  void trainMatcher() const;

  bool selectionIsValid() const { return !selection_image_.empty(); }

  //
  // Data members.
  //

  // Name given to detected objects.
  std::string object_name_;

  // Matcher parameters.
  NormType norm_type_ {NormType::l2};
  bool cross_check_ {true};

  // Custom thresholds.
  double match_distance_thresh_ {0.25};
  double ransac_reproj_thresh_ {3.0};

  // Feature-detector & descriptor-matcher.
  const cv::Ptr<cv::xfeatures2d::SURF> surf_ {
    cv::xfeatures2d::SURF::create(500, 4, 3, false, false)};
  std::unique_ptr<cv::DescriptorMatcher> matcher_ {
    new cv::BFMatcher{static_cast<int>(norm_type_), cross_check_} };
    // TODO: BFMatcher's constructor is obsolete. Use static create method instead.

  // Area/Target selection.
  cv::Rect selection_area_;
  cv::Mat selection_image_;
  std::vector<cv::KeyPoint> target_keypoints_;
  cv::Mat target_descriptors_;

  // We store the last image for computing target features on user selection.
  cv::Mat last_image_;
};

} // feature_detector

#endif  // FEATURE_DETECTOR_PARAMETER_MANAGER_H
