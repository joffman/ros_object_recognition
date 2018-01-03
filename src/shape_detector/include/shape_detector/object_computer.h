/** \file
 * \brief Definition of the ObjectComputer class hierarchy.
 */

#ifndef SHAPE_DETECTOR_OBJECT_COMPUTER_H
#define SHAPE_DETECTOR_OBJECT_COMPUTER_H

// std.
#include <memory>   // unique_ptr

// Eigen.
#include <Eigen/Dense>      // Vector3d

// PCL.
#include <pcl/PointIndices.h>
#include <pcl_msgs/ModelCoefficients.h>

// ROS.
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

// Our headers.
#include <object_detection_3d/pcl_types.h>
#include <object_detection_3d_msgs/DetectedObject3D.h>
#include "shape_enum.h"


namespace shape_detector {

/** \brief Abstract class for computing objects that represent shapes.
 */
class ObjectComputer {
public:
  /** \brief Computes a 3-dimensional object, representing a shape.
   *
   * Abstract function for computing an object, representing a specific shape.
   * Which shape is presumed is determined by the concrete implementation used.
   * The object is computed from the inputs passed in with the setter methods.
   */
  virtual object_detection_3d_msgs::DetectedObject3D computeObject() = 0;

  /** \brief Sets the point cloud that contains the computed object.
   */
  void setCloud(const object_detection_3d::PointCloudT::ConstPtr& cloud)
  { cloud_ = cloud; }

  /** \brief Sets the indices of inliers.
   *
   * The inlier-indices are the indices of those points in the given
   * point cloud that are part of the shape.
   *
   * \sa ObjectComputer::setCloud
   */
  void setInlierIndices(const pcl::PointIndices::ConstPtr& inlier_indices)
  { inlier_indices_ = inlier_indices; }


  /** \brief Sets the model coefficients of the shape.
   *
   * The coefficients define the geometry of the specific shape in detail.
   *
   * Example: The coefficients of a cylinder desribe the pose of its axis
   *  and its radius.
   */
  void setModelCoefficients(
      const pcl::ModelCoefficients::ConstPtr& model_coefficients)
  { model_coefficients_ = model_coefficients; }

protected:
  object_detection_3d::PointCloudT::ConstPtr cloud_;
  pcl::PointIndices::ConstPtr inlier_indices_;
  pcl::ModelCoefficients::ConstPtr model_coefficients_;
};


/** \brief Class for computing objects representing cylinders.
 */
class CylinderComputer : public ObjectComputer {
public:
  /** \brief Computes a 3-dimensional object representing a given cylinder.
   *
   * \note The cylinder is passed in with the setter-methods before calling this
   *  function.
   */
  object_detection_3d_msgs::DetectedObject3D computeObject() override;

private:
  geometry_msgs::Point computeCentroid() const;
  geometry_msgs::Quaternion computeOrientation() const;
  double computeDiameter() const;
  double computeHeight() const;

  static const Eigen::Vector3d reference_direction;

  object_detection_3d::PointCloudT::Ptr inlier_points_;
};


/** \brief Class for computing objects representing spheres.
 */
class SphereComputer : public ObjectComputer {
public:
  /** \brief Computes a 3-dimensional object representing a given sphere.
   *
   * \note The sphere is passed in with the setter-methods before calling this
   *  function.
   */
  object_detection_3d_msgs::DetectedObject3D computeObject() override;
  
private:
  geometry_msgs::Point computePosition() const;
  geometry_msgs::Quaternion computeOrientation() const;
  double computeDiameter() const;
};


/** \brief Creates an ObjectComputer that is capable of computing objects
 *  from instances of the given shape.
 */
std::unique_ptr<ObjectComputer> createObjectComputerForShape(Shape shape);

} // shape_detector

#endif  // SHAPE_DETECTOR_OBJECT_COMPUTER_H
