/** \file
 * \brief Definition of the ReconfigureParameterManager class.
 */

#ifndef DISTANCE_FILTER_RECONFIGURE_PARAMETER_MANAGER_H
#define DISTANCE_FILTER_RECONFIGURE_PARAMETER_MANAGER_H

// dynamic_reconfigure.
#include <dynamic_reconfigure/server.h>
#include <distance_filter/ParametersConfig.h>

// Our headers.
#include "parameter_manager.h"


namespace distance_filter {

/** \brief dynamic_reconfigure parameter-manager for DistanceFilter objects.
 *
 * A ReconfigureParameterManager object allows to manage the parameters
 * of a DistanceFilter object via a corresponding set of dynamic_reconfigure
 * parameters.
 */
class ReconfigureParameterManager : public ParameterManager {
public:
  /** \brief Initializes the dynamic_reconfigure server and registers a callback
   * for this server.
   */
  ReconfigureParameterManager();

private:
  void reconfigureCallback(ParametersConfig& config, uint32_t);

  // dynamic_reconfigure server & callback.
  dynamic_reconfigure::Server<ParametersConfig> reconfigure_server_;
  dynamic_reconfigure::Server<ParametersConfig>::CallbackType
      reconfigure_callback_;
};

} // distance_filter

#endif  // DISTANCE_FILTER_RECONFIGURE_PARAMETER_MANAGER_H
