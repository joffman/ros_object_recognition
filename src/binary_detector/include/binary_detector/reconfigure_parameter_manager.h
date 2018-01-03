/** \file
 * \brief Definition of the ReconfigureParameterManager class.
 */

#ifndef BINARY_DETECTOR_RECONFIGURE_PARAMETER_MANAGER_H
#define BINARY_DETECTOR_RECONFIGURE_PARAMETER_MANAGER_H

// std.
#include <string>

// dynamic_reconfigure.
#include <dynamic_reconfigure/server.h>
#include <binary_detector/ParametersConfig.h>

// Headers of this project.
#include "parameter_manager.h"


namespace binary_detector {

/** \brief dynamic_reconfigure parameter-manager for BinaryDetector objects.
 *
 * A ReconfigureParameterManager object allows to manage the parameters
 * of a BinaryDetector object via a corresponding set of dynamic_reconfigure
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

} // binary_detector

#endif  // BINARY_DETECTOR_RECONFIGURE_PARAMETER_MANAGER_H
