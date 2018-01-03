/** \file
 * \brief Implementation of the ObservedParameterManager class.
 */

// std.
#include <string>

// OpenCV.
#include <cv_bridge/cv_bridge.h>  // FileStorage

// Headers of this package.
#include <binary_detector/observed_parameter_manager.h>

using namespace std;


namespace {

string fullParamName(const string& param_name)
{
  return "BinaryDetector-" + param_name;
}

} // anonymous namespace
    
    
namespace binary_detector {

void ObservedParameterManager::writeParametersToStorage(cv::FileStorage& fs)
    const
{
  auto& fpn = fullParamName;
  fs << fpn("min_length_frac") << minLengthFraction()
      << fpn("max_length_frac") << maxLengthFraction();
}


void ObservedParameterManager::setParametersFromStorage(cv::FileStorage& fs)
{
  auto& fpn = fullParamName;

  {
    auto fn = fs[fullParamName("min_length_frac")];
    if (!fn.empty())
      setMinLengthFraction((double)fn);
  }

  {
    auto fn = fs[fullParamName("max_length_frac")];
    if (!fn.empty())
      setMaxLengthFraction((double)fn);
  }
}


void ObservedParameterManager::setObjectName(const string& name)
{
  ParameterManager::setObjectName(name);
  notify();
}


void ObservedParameterManager::setMinLengthFraction(double val)
{
  ParameterManager::setMinLengthFraction(val);
  notify();
}


void ObservedParameterManager::setMaxLengthFraction(double val)
{
  ParameterManager::setMaxLengthFraction(val);
  notify();
}

} // binary_detector
