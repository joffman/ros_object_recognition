/** \file
 * \brief Definition of the ParameterManager class.
 */

#ifndef OBJECT_PAINTER_PARAMETER_MANAGER_H
#define OBJECT_PAINTER_PARAMETER_MANAGER_H

// std.
#include <functional> // bind(), placeholders

// ROS.
#include <ros/ros.h>  // Duration
#include <dynamic_reconfigure/server.h>

// object_painter.
#include <object_painter/ObjectPainterConfig.h>


namespace object_painter {

/** \brief dynamic_reconfigure parameter-manager for ObjectPainter objects.
 *
 * A ParameterManager object manages the dynamic_reconfigure parameters of
 * an ObjectPainter object and provides getter-methods for accessing those
 * parameters.
 */
class ParameterManager {
public:
  /** \brief Initializes the dynamic_reconfigure server and registers a callback
   * for this server.
   */
  ParameterManager()
  {
    reconfigure_callback_ = std::bind(&ParameterManager::reconfigureCallback, this,
                           std::placeholders::_1, std::placeholders::_2);
    reconfigure_server_.setCallback(reconfigure_callback_);
  }


  /** \brief Returns the value of the max-age parameter. */
  ros::Duration maxAge() const { return max_age_; }

private:
  void reconfigureCallback(object_painter::ObjectPainterConfig& config,
                     uint32_t)
  {
    max_age_ = ros::Duration {config.max_age_s};
  }

  // Data members.
  dynamic_reconfigure::Server<object_painter::ObjectPainterConfig>
      reconfigure_server_;
  dynamic_reconfigure::Server<object_painter::ObjectPainterConfig>::
      CallbackType reconfigure_callback_;

  ros::Duration max_age_;
};

} // object_painter

#endif  // OBJECT_PAINTER_PARAMETER_MANAGER_H
