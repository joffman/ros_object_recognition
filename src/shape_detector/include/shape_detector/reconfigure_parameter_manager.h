/** \file
 * \brief Definition of the ReconfigureParameterManager class.
 */

#ifndef SHAPE_DETECTOR_RECONFIGURE_PARAMETER_MANAGER_H
#define SHAPE_DETECTOR_RECONFIGURE_PARAMETER_MANAGER_H

// dynamic_reconfigure.
#include <dynamic_reconfigure/server.h>
#include <shape_detector/ParametersConfig.h>

// Headers of this package.
#include "shape_enum.h"
#include "parameter_manager.h"


namespace shape_detector {

/** \brief dynamic_reconfigure parameter-manager for ShapeDetector objects.
 *
 * A ReconfigureParameterManager object allows to manage the parameters
 * of a ShapeDetector object via a corresponding set of dynamic_reconfigure
 * parameters.
 */
class ReconfigureParameterManager : public ParameterManager {
public:
 /** \brief Initializes the dynamic_reconfigure server and registers a callback
  * for this server.
  */
  ReconfigureParameterManager();

private:
  void reconfigureCallback(shape_detector::ParametersConfig& config,
                     uint32_t);

  // dynamic_reconfigure server & callback.
  dynamic_reconfigure::Server<ParametersConfig> reconfigure_server_;
  dynamic_reconfigure::Server<ParametersConfig>::CallbackType
      reconfigure_callback_;
};

} // shape_detector

#endif  // SHAPE_DETECTOR_RECONFIGURE_PARAMETER_MANAGER_H
