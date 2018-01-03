/** \file
 * \brief Implementation of the ParameterManager class.
 */

/*
 * TODO: I think the logic behind using lookingForTarget() and
 *  selectionIsValid() is very obscured. The only reason that
 *  selectionIsValid() exists is to control when lookingForTarget()
 *  becomes true.
 */

// std.
#include <string>
#include <algorithm>  // copy_if(), min(), max()
#include <memory>     // unique_ptr
#include <functional> // bind

// OpenCV.
#include <opencv2/core/core.hpp>    // Mat, Size, Scalar, FileStorage
#include <opencv2/features2d/features2d.hpp>  // BFMatcher

// Headers of this package.
#include <feature_detector/parameter_manager.h>

using namespace std;


namespace feature_detector {

namespace {

template<typename T>
inline T truncateToRange(T val, T min, T max)
{
  return std::min(std::max(val, min), max);
}


/*
 * Returns mask with given size, filled with ones only in the given rectangular
 * area.
 */
inline cv::Mat createMask(
    const cv::Size& sz,    // overall size of mask
    const cv::Rect& rect)  // region that is filled with ones
{
  cv::Mat mask = cv::Mat::zeros(sz.height, sz.width, CV_8UC1);
  cv::Mat selected_region = mask(rect);
  selected_region = cv::Scalar::all(1);

  return mask;
}


// TODO: There is still a bug hiding in here.
//  The program crashes when you select an area from top-right to bottom-left,
//  with bottom-left far out of the image. (OpenCV Error '-215'.)
cv::Rect computeTruncatedROIForImage(const cv::Rect& roi, const cv::Mat& image)
{
  cv::Rect truncated_roi;

  truncated_roi.x = max(roi.x, 0);
  truncated_roi.y = max(roi.y, 0);

  truncated_roi.width =
      min(roi.width, image.size().width - roi.x);
  truncated_roi.height =
      min(roi.height, image.size().height - roi.y);

  return truncated_roi;
}
 
} // anonymous ns


void ParameterManager::processAreaSelection(const cv::Rect& area)
{
  setSelection(last_image_.clone(), area);
}


// TODO: Maybe remove updateFeaturesAndMatcher;
//  remove the call to trainMatcher in computeTargetFeatures and only call this
//  method from updateFeaturesAndMatcher.
void ParameterManager::updateFeaturesAndMatcher()
{
  computeTargetFeatures();
  trainMatcher(); // train with the newly computed features.
}


void ParameterManager::computeTargetFeatures()
{
  if (selectionIsValid()) {
    cv::Mat mask = createMask(selection_image_.size(), selection_area_);
    surf_->detectAndCompute(selection_image_, mask,
                            target_keypoints_, target_descriptors_);
    // TODO: Check number & quality of keypoints.
    //  If not sufficient, print warning and clear target_keypoints_
    //  (so that we don't look for a target).

    trainMatcher(); // target_descriptors_ changed, so retrain matcher_.
      // TODO: Is this redundant?
  }
}


void ParameterManager::trainMatcher() const
{
  matcher_->clear();
  matcher_->add(target_descriptors_);
  matcher_->train();
}


//
// Setters.
//

void ParameterManager::setObjectName(const std::string& name)
{
  object_name_ = name;
}


void ParameterManager::setHessianThreshold(double val)
{
  surf_->setHessianThreshold(truncateToRange(val, min_hessian_thresh,
                                             max_hessian_thresh));
  computeTargetFeatures();
}


void ParameterManager::setNumOctaves(int val)
{
  surf_->setNOctaves(
      truncateToRange(val, min_num_octaves, max_num_octaves));
  computeTargetFeatures();
}


void ParameterManager::setNumOctaveLayers(int val)
{
  surf_->setNOctaveLayers(truncateToRange(val, min_num_octave_layers,
                                          max_num_octave_layers));
  computeTargetFeatures();
}


void ParameterManager::setExtended(bool val)
{
  surf_->setExtended(val);
  computeTargetFeatures();
}


void ParameterManager::setUpright(bool val)
{
  surf_->setUpright(val);
  computeTargetFeatures();
}


void ParameterManager::setNormType(const NormType type)
{
  norm_type_ = type;
  matcher_.reset(new cv::BFMatcher{static_cast<int>(norm_type_), cross_check_});
  trainMatcher();
}


void ParameterManager::setCrossCheck(const bool val)
{
  cross_check_ = val;
  matcher_.reset(new cv::BFMatcher{static_cast<int>(norm_type_), cross_check_});
  trainMatcher();
}


void ParameterManager::setMatchDistanceThreshold(double val)
{
  match_distance_thresh_ = truncateToRange(val, min_match_distance_thresh,
                                           max_match_distance_thresh);
}


void ParameterManager::setRansacReprojThreshold(double val)
{
  ransac_reproj_thresh_ = truncateToRange(val, min_ransac_reproj_thresh,
                                          max_ransac_reproj_thresh);
}


void ParameterManager::setSelection(const cv::Mat& selection_image,
                                    const cv::Rect& selection_area)
{
  selection_image_ = selection_image;
  selection_area_ =
      computeTruncatedROIForImage(selection_area, selection_image_);

  updateFeaturesAndMatcher();
}

} // feature_detector
