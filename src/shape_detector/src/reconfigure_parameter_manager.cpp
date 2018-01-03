/** \file
 * \brief Implementation of the ReconfigureParameterManager class.
 */

// std.
#include <functional> // bind(), placeholders
#include <algorithm>  // max()

// Headers of this package.
#include <shape_detector/reconfigure_parameter_manager.h>
#include <shape_detector/shape_enum.h>


namespace shape_detector {

ReconfigureParameterManager::ReconfigureParameterManager()
{
  reconfigure_callback_ = std::bind(&ReconfigureParameterManager::reconfigureCallback,
                                    this,
                                    std::placeholders::_1, std::placeholders::_2);
  reconfigure_server_.setCallback(reconfigure_callback_);
}

void ReconfigureParameterManager::reconfigureCallback(
    shape_detector::ParametersConfig& config, uint32_t)
{
  setObjectName(config.object_name);
  setShape(static_cast<Shape>(config.shape));

  // Groups do not work yet.
  // That's why we cannot write 'config.groups.Radii.min_radius'.
  config.max_radius = std::max(config.min_radius, config.max_radius);
  setMinRadius(config.min_radius);
  setMaxRadius(config.max_radius);

  setNormalDistanceWeight(config.normal_distance_weight);
  setDistanceThreshold(config.distance_threshold);
  setNumNearestNeighbors(config.num_nearest_neighbors);
  setMaxIterations(config.max_iterations);
  setOptimizeCoefficients(config.optimize_coefficients);
}

} // shape_detector
