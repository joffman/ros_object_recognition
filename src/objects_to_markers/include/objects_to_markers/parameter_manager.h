/** \file
 * \brief Definition of the ParameterManager class.
 */

#ifndef OBJECTS_TO_MARKERS_PARAMETER_MANAGER_H
#define OBJECTS_TO_MARKERS_PARAMETER_MANAGER_H

// std.
#include <functional> // bind(), placeholders

// ROS.
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

// Our headers.
#include <objects_to_markers/ParametersConfig.h>


namespace objects_to_markers {

/** \brief dynamic_reconfigure parameter-manager for ObjectsToMarkers objects.
 *
 * ParameterManager objects manage the dynamic_reconfigure parameters of
 * ObjectsToMarkers objects and provide getter-methods for accessing those
 * parameters.
 */
class ParameterManager {
public:
  /** \brief Default constructor.
   *
   * Initializes the dynamic_reconfigure server and registers a callback
   * for this server.
   */
  ParameterManager()
  {
    reconfigure_callback_ = std::bind(
        &ParameterManager::reconfigureCallback, this,
        std::placeholders::_1, std::placeholders::_2);
    reconfigure_server_.setCallback(reconfigure_callback_);
  }

  /** \brief Getter function for the 'lifetime' parameter.
    */
  ros::Duration lifetime() const { return lifetime_; }

private:
  void reconfigureCallback(ParametersConfig& config, uint32_t)
  {
    lifetime_ = ros::Duration{config.lifetime_s};
  }

  // Data members.
  dynamic_reconfigure::Server<ParametersConfig> reconfigure_server_;
  dynamic_reconfigure::Server<ParametersConfig>::CallbackType reconfigure_callback_;

  ros::Duration lifetime_;
};

} // objects_to_markers

#endif  // OBJECTS_TO_MARKERS_PARAMETER_MANAGER_H
