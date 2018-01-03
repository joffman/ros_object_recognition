/** \file
 * \brief Implementation of the ReconfigureParameterManager class.
 */

// std.
#include <functional> // bind(), placeholders

// object-detection.
#include <morphology_filter/reconfigure_parameter_manager.h>
#include <morphology_filter/ParametersConfig.h>

using namespace std;


namespace morphology_filter {

ReconfigureParameterManager::ReconfigureParameterManager()
  : reconfigure_callback_{bind(
      &ReconfigureParameterManager::reconfigureCallback, this,
      placeholders::_1, placeholders::_2)}
{
    reconfigure_server_.setCallback(reconfigure_callback_);
}


void ReconfigureParameterManager::reconfigureCallback(
    morphology_filter::ParametersConfig& config, uint32_t)
{
  setOperation(static_cast<Operation>(config.operation));
  setNumIterations(config.num_iterations);
}

} // morphology_filter
