/** \file
 * \brief Implementation of the ObservedParameterManager class.
 */

// std.
#include <string>

// OpenCV.
#include <cv_bridge/cv_bridge.h>  // FileStorage

// Headers of this package.
#include <morphology_filter/observed_parameter_manager.h>

using namespace std;

    
namespace morphology_filter {

namespace {

string fullParamName(const string& param_name)
{
  return "MorphologyFilter-" + param_name;
}

} // anonymous namespace


////////////////////////////////////////////////////////////////////////////////
//
// Parametrizable interface.
//

void ObservedParameterManager::writeParametersToStorage(cv::FileStorage& fs)
  const 
{
  auto& fpn = fullParamName;  // convenience shortcut
  fs << fpn("operation") << static_cast<int>(operation())
      << fpn("num_iterations") << numIterations();
}


void ObservedParameterManager::setParametersFromStorage(cv::FileStorage& fs) 
{
  auto& fpn = fullParamName;

  {
    auto fn = fs[fpn("operation")]; // file-node
    if (!fn.empty())
      setOperation(static_cast<Operation>((int)fn));
  }

  {
    auto fn = fs[fpn("num_iterations")];
    if (!fn.empty())
      setNumIterations((int)fn);
  }
}


////////////////////////////////////////////////////////////////////////////////
//
// Setters.
//

void ObservedParameterManager::setOperation(Operation op)
{
  ParameterManager::setOperation(op);
  notify();
}


void ObservedParameterManager::setNumIterations(int val)
{
  ParameterManager::setNumIterations(val);
  notify();
}

} // morphology_filter
