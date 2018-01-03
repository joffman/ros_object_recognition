/** \file
 * \brief Implementation of the ObservedParameterManager class.
 */

// std.
#include <string>

// OpenCV.
#include <opencv2/core/core.hpp>    // KeyPoint, DMatch, FileStorage
#include <cv_bridge/cv_bridge.h>  // necessary?

// Our headers.
#include <feature_detector/observed_parameter_manager.h>


namespace {

std::string fullParamName(const std::string& param_name)
{
  return "FeatureDetector-" + param_name;
}

} // anonymous namespace
    

namespace feature_detector {

void ObservedParameterManager::writeParametersToStorage(cv::FileStorage& fs) const
{
  auto fpn = fullParamName; // just for convenience
  fs << fpn("hessian_thresh") << hessianThreshold()
      << fpn("num_octaves") << numOctaves()
      << fpn("num_octave_layers") << numOctaveLayers()
      << fpn("extended") << extended()
      << fpn("upright") << upright()
      << fpn("norm_type") << static_cast<int>(normType())
      << fpn("cross_check") << crossCheck()
      //<< fpn("kpt_response_thresh") << keypointResponseThreshold()
      << fpn("match_distance_thresh") << matchDistanceThreshold()
      << fpn("ransac_reproj_thresh") << ransacReprojThreshold()
      << fpn("selection_area") << selectionArea()
      << fpn("selection_image") << selectionImage();
}


// TODO: Refactor this.
// Problems:
//  I think the matcher is retrained about 10 times.
//  Features are recomputed about 6 times.
//  Repetition.
//  The same names are used as in writeParametersToStorage.
void ObservedParameterManager::setParametersFromStorage(cv::FileStorage& fs)
{
  auto fpn = fullParamName;
  {
    auto fn = fs[fpn("hessian_thresh")];  // file-node
    if (!fn.empty())
      setHessianThreshold((double)fn);
  }
  {
    auto fn = fs[fpn("num_octaves")];
    if (!fn.empty())
      setNumOctaves((int)fn);
  }
  {
    auto fn = fs[fpn("num_octave_layers")];
    if (!fn.empty())
      setNumOctaveLayers((int)fn);
  }
  {
    auto fn = fs[fpn("extended")];
    if (!fn.empty())
      setExtended(static_cast<bool>((int)fn));
  }
  {
    auto fn = fs[fpn("upright")];
    if (!fn.empty())
      setUpright(static_cast<bool>((int)fn));
  }
  {
    auto fn = fs[fpn("norm_type")];
    if (!fn.empty())
      setNormType(static_cast<NormType>((int)fn));
  }
  {
    auto fn = fs[fpn("cross_check")];
    if (!fn.empty())
      setCrossCheck(static_cast<bool>((int)fn));
  }
  /*
  {
    auto fn = fs[fpn("kpt_response_thresh")];
    if (!fn.empty())
      setKeypointResponseThreshold((double)fn);
  }
  */
  {
    auto fn = fs[fpn("match_distance_thresh")];
    if (!fn.empty())
      setMatchDistanceThreshold((double)fn);
  }
  {
    auto fn = fs[fpn("ransac_reproj_thresh")];
    if (!fn.empty())
      setRansacReprojThreshold((double)fn);
  }
  {
    auto fn_image = fs[fpn("selection_image")];
    auto fn_area = fs[fpn("selection_area")];
    if (!fn_image.empty() && !fn_area.empty()) {
      cv::Mat m;
      fn_image >> m;

      cv::Rect r;
      fn_area >> r;

      setSelection(m, r);
    }
  }
}


//
// Setters.
//

void ObservedParameterManager::setObjectName(const std::string& name)
{
  ParameterManager::setObjectName(name);
  notify();
}

void ObservedParameterManager::setHessianThreshold(double val)
{
  ParameterManager::setHessianThreshold(val);
  notify();
}

void ObservedParameterManager::setNumOctaves(int val)
{
  ParameterManager::setNumOctaves(val);
  notify();
}

void ObservedParameterManager::setNumOctaveLayers(int val)
{
  ParameterManager::setNumOctaveLayers(val);
  notify();
}

void ObservedParameterManager::setExtended(bool val)
{
  ParameterManager::setExtended(val);
  notify();
}

void ObservedParameterManager::setUpright(bool val)
{
  ParameterManager::setUpright(val);
  notify();
}

void ObservedParameterManager::setNormType(const NormType type)
{
  ParameterManager::setNormType(type);
  notify();
}

void ObservedParameterManager::setCrossCheck(const bool val)
{
  ParameterManager::setCrossCheck(val);
  notify();
}

/*
void ObservedParameterManager::setKeypointResponseThreshold(const double val)
{
  ParameterManager::setKeypointResponseThreshold(val);
  notify();
}
*/

void ObservedParameterManager::setMatchDistanceThreshold(double val)
{
  ParameterManager::setMatchDistanceThreshold(val);
  notify();
}

void ObservedParameterManager::setRansacReprojThreshold(double val)
{
  ParameterManager::setRansacReprojThreshold(val);
  notify();
}

} // feature_detector
