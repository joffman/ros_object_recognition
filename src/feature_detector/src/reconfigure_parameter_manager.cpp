/** \file
 * \brief Implementation of the ReconfigureParameterManager class.
 */

// std.
#include <string>
#include <functional> // bind

// OpenCV.
#include <opencv2/core/core.hpp>    // Mat, FileStorage

// Headers of this package.
#include <feature_detector/ParametersConfig.h>
#include <feature_detector/reconfigure_parameter_manager.h>

using namespace std;


namespace feature_detector {

ReconfigureParameterManager::ReconfigureParameterManager()
{
  reconfigure_callback_ = bind(&ReconfigureParameterManager::reconfigureCallback, this,
                               placeholders::_1, placeholders::_2);
  reconfigure_server_.setCallback(reconfigure_callback_);
}


// TODO: Improve this. Features are recomputed a lot of times.
void ReconfigureParameterManager::reconfigureCallback(
    feature_detector::ParametersConfig& config, uint32_t)
{
  /*
  object_name_ = config.object_name;

  // surf_->setHessianThreshold(config.groups.surf.hessian_threshold);
  // groups are not yet fully implemented
  // (see answers.ros.org/question/101506)
  surf_->setHessianThreshold(config.hessian_threshold);
  surf_->setNOctaves(config.num_octaves);
  surf_->setNOctaveLayers(config.num_octave_layers);
  surf_->setExtended(config.extended);
  surf_->setUpright(config.upright);

  matcher_.reset(new cv::BFMatcher {config.norm_type, config.cross_check});

  match_distance_thresh_ = config.match_distance_threshold;
  ransac_reproj_thresh_ = config.ransac_reproj_threshold;
  */

  setObjectName(config.object_name);
  setHessianThreshold(config.hessian_threshold);
  setNumOctaves(config.num_octaves);
  setNumOctaveLayers(config.num_octave_layers);
  setExtended(config.extended);
  setUpright(config.upright);
  setNormType(static_cast<NormType>(config.norm_type));
  setCrossCheck(config.cross_check);
  setMatchDistanceThreshold(config.match_distance_threshold);
  setRansacReprojThreshold(config.ransac_reproj_threshold);

  if (config.save_selection) {
    config.save_selection = false;
    saveSelection(config.filename);
  }
  if (config.load_selection) {
    config.load_selection = false;
    loadSelection(config.filename);
  }

  // updateFeaturesAndMatcher();
}


void ReconfigureParameterManager::saveSelection(const string& filename) const
{
  cv::FileStorage fs {filename, cv::FileStorage::WRITE};
  if (fs.isOpened()) {
    fs << "selection_area" << selectionArea() // selection_area_
        << "selection_image" << selectionImage();  //selection_image_;
  }
}


void ReconfigureParameterManager::loadSelection(const string& filename)
{
  cv::FileStorage fs {filename, cv::FileStorage::READ};
  if (fs.isOpened()) {
    auto fn_image = fs["selection_image"];
    auto fn_area = fs["selection_area"];
    if (!fn_image.empty() && !fn_area.empty()) {
      cv::Mat image;
      fn_image >> image;

      cv::Rect area;
      fn_area >> area;

      setSelection(image, area);
    }
  }
}

} // feature_detector
