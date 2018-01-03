/** \file
 * \brief Definition of the ReconfigureParameterManager class.
 */

#ifndef FEATURE_DETECTOR_RECONFIGURE_PARAMETER_MANAGER_H
#define FEATURE_DETECTOR_RECONFIGURE_PARAMETER_MANAGER_H

// std.
#include <string>

// dynamic_reconfigure.
#include <dynamic_reconfigure/server.h>

// Headers of this package.
#include <feature_detector/ParametersConfig.h>
#include "parameter_manager.h"


namespace feature_detector {

/** \brief dynamic_reconfigure parameter-server for FeatureDetector objects.
 *
 * A ReconfigureParameterManager object allows to manage the parameters
 * of a FeatureDetector object via a corresponding set of dynamic_reconfigure
 * parameters.
 */
class ReconfigureParameterManager : public ParameterManager {
public:
  /** \brief Initializes the dynamic_reconfigure server and registers a callback
   * for this server.
   */
  ReconfigureParameterManager();

private:
  void reconfigureCallback(feature_detector::ParametersConfig& config,
                           uint32_t);

  void saveSelection(const std::string& filename) const;
  void loadSelection(const std::string& filename);
    // or are those part of ParameterManager?

  // dynamic_reconfigure server & callback.
  dynamic_reconfigure::Server<feature_detector::ParametersConfig>
      reconfigure_server_;
  dynamic_reconfigure::Server<feature_detector::ParametersConfig>::
      CallbackType reconfigure_callback_;
};

} // feature_detector

#endif  // FEATURE_DETECTOR_RECONFIGURE_PARAMETER_MANAGER_H
