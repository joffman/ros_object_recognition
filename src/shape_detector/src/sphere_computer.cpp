/** \file
 * \brief Implementation of the SphereComputer class.
 */

// ROS.
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

// object_detection_*.
#include <object_detection_3d_msgs/DetectedObject3D.h>

// Headers of this package.
#include <shape_detector/object_computer.h>

using object_detection_3d_msgs::DetectedObject3D;
using namespace std;


namespace shape_detector {

DetectedObject3D SphereComputer::computeObject()
{
  DetectedObject3D obj;

  obj.box.pose.position = computePosition();
  obj.box.pose.orientation = computeOrientation();
  obj.box.width = obj.box.height = obj.box.depth = computeDiameter();

  return obj;
}


geometry_msgs::Point SphereComputer::computePosition() const
{
  geometry_msgs::Point position;

  position.x = model_coefficients_->values[0];
  position.y = model_coefficients_->values[1];
  position.z = model_coefficients_->values[2];

  return position;
}


geometry_msgs::Quaternion SphereComputer::computeOrientation() const
{
  geometry_msgs::Quaternion q;

  q.x = 0;
  q.y = 0;
  q.z = 0;
  q.w = 1;

  return q;
}


double SphereComputer::computeDiameter() const
{
  return 2 * model_coefficients_->values[3];
}

} // shape_detector
