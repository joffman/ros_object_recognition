/** \file
 * \brief Implementation of the ReconfigureParameterManager class.
 */

// std.
#include <algorithm>  // max()
#include <functional> // bind(), placeholders

// Our headers.
#include <distance_filter/reconfigure_parameter_manager.h>

using namespace std;


namespace distance_filter {

ReconfigureParameterManager::ReconfigureParameterManager()
{
  reconfigure_callback_ = bind(
      &ReconfigureParameterManager::reconfigureCallback, this,
      placeholders::_1, placeholders::_2);
  reconfigure_server_.setCallback(reconfigure_callback_);
}


void ReconfigureParameterManager::reconfigureCallback(ParametersConfig& config,
                                                      uint32_t)
{
  config.max_distance = max(config.min_distance, config.max_distance);

  ParameterManager::setMinDistance(config.min_distance);
  ParameterManager::setMaxDistance(config.max_distance);
}

} // distance_filter
