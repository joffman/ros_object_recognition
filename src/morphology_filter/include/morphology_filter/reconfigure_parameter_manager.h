/** \file
 * \brief Definition of the ReconfigureParameterManager class.
 */

#ifndef MORPHOLOGY_FILTER_RECONFIGURE_PARAMETER_MANAGER_H
#define MORPHOLOGY_FILTER_RECONFIGURE_PARAMETER_MANAGER_H

// std.
#include <cstdint>  // uint32_t

// dynamic_reconfigure.
#include <dynamic_reconfigure/server.h>
#include <morphology_filter/ParametersConfig.h>

// Headers of this package.
#include <morphology_filter/parameter_manager.h>


namespace morphology_filter {

/** \brief dynamic_reconfigure parameter-manager for MorphologyFilter objects.
 *
 * A ReconfigureParameterManager object allows to manage the parameters
 * of a MorphologyFilter object via a corresponding set of dynamic_reconfigure
 * parameters.
 */
class ReconfigureParameterManager : public ParameterManager {
public:
  /** \brief Initializes the dynamic_reconfigure server and registers a callback
   * for this server.
   */
  ReconfigureParameterManager();

private:
  void reconfigureCallback(morphology_filter::ParametersConfig& config,
                           uint32_t);

  // Dynamic reconfigure server & callback.
  dynamic_reconfigure::Server<morphology_filter::ParametersConfig>::CallbackType
      reconfigure_callback_;
  dynamic_reconfigure::Server<morphology_filter::ParametersConfig>
      reconfigure_server_;
};

} // morphology_filter

#endif  // MORPHOLOGY_FILTER_RECONFIGURE_PARAMETER_MANAGER_H
