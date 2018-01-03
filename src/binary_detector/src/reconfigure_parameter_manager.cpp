/** \file
 * \brief Implementation of the ReconfigureParameterManager.
 */

// std.
#include <functional> // bind(), placeholders

// binary_detector.
#include <binary_detector/ParametersConfig.h>
#include <binary_detector/reconfigure_parameter_manager.h>

using namespace std;


namespace binary_detector {

ReconfigureParameterManager::ReconfigureParameterManager()
{
  reconfigure_callback_ = bind(&ReconfigureParameterManager::reconfigureCallback, this,
                         placeholders::_1, placeholders::_2);
  reconfigure_server_.setCallback(reconfigure_callback_);
}


void ReconfigureParameterManager::reconfigureCallback(ParametersConfig& config, uint32_t)
{
  setObjectName(config.object_name);

  setMinLengthFraction(config.min_length_fraction);
  setMaxLengthFraction(config.max_length_fraction);
}

} // binary_detector
