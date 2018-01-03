/** \file
 * \brief Implementation of the ObservedParameterManager class.
 */

// std.
#include <string>

// OpenCV.
#include <cv_bridge/cv_bridge.h>  // FileStorage

// Headers of this package.
#include <hsv_filter/observed_parameter_manager.h>

using namespace std;

    
namespace hsv_filter {

namespace {

string fullParamName(const string& param_name)
{
  return "HSVFilter-" + param_name;
}

} // anonymous namespace



void ObservedParameterManager::writeParametersToStorage(cv::FileStorage& fs)
  const 
{
  auto& fpn = fullParamName;  // convenience shortcut
  fs << fpn("h_min") << hMin()
      << fpn("h_max") << hMax()
      << fpn("s_min") << sMin()
      << fpn("s_max") << sMax()
      << fpn("v_min") << vMin()
      << fpn("v_max") << vMax();
}


void ObservedParameterManager::setParametersFromStorage(cv::FileStorage& fs) 
{
  auto& fpn = fullParamName;

  {
    auto fn = fs[fpn("h_min")]; // file-node
    if (!fn.empty())
      setHMin((int)fn);
  }
  {
    auto fn = fs[fpn("h_max")];
    if (!fn.empty())
      setHMax((int)fn);
  }

  {
    auto fn = fs[fpn("s_min")];
    if (!fn.empty())
      setSMin((int)fn);
  }
  {
    auto fn = fs[fpn("s_max")];
    if (!fn.empty())
      setSMax((int)fn);
  }

  {
    auto fn = fs[fpn("v_min")];
    if (!fn.empty())
      setVMin((int)fn);
  }
  {
    auto fn = fs[fpn("v_max")];
    if (!fn.empty())
      setVMax((int)fn);
  }
}


////////////////////////////////////////////////////////////////////////////////
//
// Setters.
//

void ObservedParameterManager::setHMin(int val)
{
  ParameterManager::setHMin(val);
  notify();
}


void ObservedParameterManager::setHMax(int val)
{
  ParameterManager::setHMax(val);
  notify();
}


void ObservedParameterManager::setSMin(int val)
{
  ParameterManager::setSMin(val);
  notify();
}


void ObservedParameterManager::setSMax(int val)
{
  ParameterManager::setSMax(val);
  notify();
}


void ObservedParameterManager::setVMin(int val)
{
  ParameterManager::setVMin(val);
  notify();
}


void ObservedParameterManager::setVMax(int val)
{
  ParameterManager::setVMax(val);
  notify();
}

} // hsv_filter
