/** \file
 * \brief Definition of the ReconfigureParameterManager class.
 */

#ifndef HSV_FILTER_RECONFIGURE_PARAMETER_MANAGER_H
#define HSV_FILTER_RECONFIGURE_PARAMETER_MANAGER_H

// std.
#include <cstdint>  // uint32_t

// dynamic_reconfigure.
#include <dynamic_reconfigure/server.h>

// Headers of this package.
#include <hsv_filter/ParametersConfig.h>
#include <hsv_filter/parameter_manager.h>


namespace hsv_filter {

/** \brief dynamic_reconfigure parameter-manager for HSVFilter objects.
 *
 * A ReconfigureParameterManager object allows to manage the parameters
 * of a HSVFilter object via a corresponding set of dynamic_reconfigure
 * parameters.
 */
class ReconfigureParameterManager : public ParameterManager {
public:
  /** \brief Initializes the dynamic_reconfigure server and registers a callback
   * for this server.
   */
   ReconfigureParameterManager();

private:
  void reconfigureCallback(hsv_filter::ParametersConfig& config,
                           uint32_t);

  // Dynamic reconfigure server & callback.
  dynamic_reconfigure::Server<hsv_filter::ParametersConfig>::CallbackType
      reconfigure_callback_;
  dynamic_reconfigure::Server<hsv_filter::ParametersConfig>
      reconfigure_server_;
};

} // hsv_filter

#endif  // HSV_FILTER_RECONFIGURE_PARAMETER_MANAGER_H
