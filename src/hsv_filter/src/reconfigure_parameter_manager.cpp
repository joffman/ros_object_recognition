/** \file
 * \brief Implementation of the ReconfigureParameterManager class.
 */

// std.
#include <functional> // bind(), placeholders

// Headers of this project.
#include <hsv_filter/reconfigure_parameter_manager.h>
#include <hsv_filter/ParametersConfig.h>

using namespace std;


namespace hsv_filter {

ReconfigureParameterManager::ReconfigureParameterManager()
  : reconfigure_callback_{bind(
      &ReconfigureParameterManager::reconfigureCallback, this,
      placeholders::_1, placeholders::_2)}
{
    reconfigure_server_.setCallback(reconfigure_callback_);
}


void ReconfigureParameterManager::reconfigureCallback(
    hsv_filter::ParametersConfig& config, uint32_t)
{
  setHMin(config.h_min);
  setHMax(config.h_max);

  setSMin(config.s_min);
  setSMax(config.s_max);

  setVMin(config.v_min);
  setVMax(config.v_max);
}

} // hsv_filter
