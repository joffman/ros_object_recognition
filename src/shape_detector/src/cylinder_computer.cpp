/** \file
 * \brief Implementation of the CylinderComputer class.
 */

// std.
#include <algorithm>  // min(), max()
#include <exception>

// Eigen.
#include <Eigen/Dense>      // Vector3d
#include <Eigen/Geometry>   // Quaternion
#include <eigen_conversions/eigen_msg.h>    // tf::quaternionEigenToMsg()

// PCL.
#include <pcl/common/centroid.h>  // computeCentroid()
#include <pcl/filters/project_inliers.h>  // ProjectInliers
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <pcl/sample_consensus/model_types.h>

// ROS.
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

// object_detection_*.
#include <object_detection_3d_msgs/DetectedObject3D.h>
#include <object_detection_3d/pcl_types.h>

// Headers of this package.
#include <shape_detector/object_computer.h>

using object_detection_3d_msgs::DetectedObject3D;
using object_detection_3d::PointT;
using object_detection_3d::PointCloudT;
using namespace std;


namespace {

/** \brief Extract points with given indices from given cloud.
 * \param[in] cloud Cloud from which points are extracted.
 * \param[in] indices Indices of the points that are extracted from the
 *  input cloud.
 * \return Cloud containing the extracted points.
 */
inline PointCloudT::Ptr extractIndexedPoints(
    const PointCloudT::ConstPtr& cloud,
    const pcl::PointIndices::ConstPtr& indices)
{
    pcl::ExtractIndices<PointT> extractor;
    extractor.setInputCloud(cloud);
    extractor.setIndices(indices);
    extractor.setNegative(false);

    PointCloudT::Ptr indexed_points {new PointCloudT};
    extractor.filter(*indexed_points);

    return indexed_points;
}


inline PointCloudT::Ptr createPointCloudFromPoint(const PointT& pt)
{
  PointCloudT::Ptr cloud {new PointCloudT};
  cloud->points.push_back(pt);
  return cloud;
}


inline PointCloudT::Ptr projectPointCloudOnLine(
    const PointCloudT::ConstPtr& cloud,
    const pcl::ModelCoefficients::ConstPtr& line_model)
{
  pcl::ProjectInliers<PointT> projection;
  projection.setInputCloud(cloud);
  projection.setModelType(pcl::SACMODEL_LINE);
  projection.setModelCoefficients(line_model);

  PointCloudT::Ptr projected_cloud {new PointCloudT};
  projection.filter(*projected_cloud);
  return projected_cloud;
}


inline pcl::ModelCoefficients::Ptr computeCylinderAxisModel(
    const pcl::ModelCoefficients::ConstPtr& cylinder_model)
{
  constexpr int num_line_coefficients {6};

  pcl::ModelCoefficients::Ptr axis_model {new pcl::ModelCoefficients};
  axis_model->values.resize(num_line_coefficients);
  for (int i {0}; i < num_line_coefficients; ++i)
    axis_model->values[i] = cylinder_model->values.at(i);

  return axis_model;
}


inline PointT projectPointOnLine(
    const PointT& pt, const pcl::ModelCoefficients::ConstPtr& line_model)
{
  PointCloudT::Ptr cloud_containing_pt {createPointCloudFromPoint(pt)};
  PointCloudT::Ptr projected_cloud_containing_pt {projectPointCloudOnLine(
          cloud_containing_pt, line_model)};
  return projected_cloud_containing_pt->points[0];
}


inline Eigen::Vector3d computeCylinderDirection(
    const pcl::ModelCoefficients::ConstPtr& cylinder_model)
{
  return Eigen::Vector3d {
    cylinder_model->values.at(3),
    cylinder_model->values.at(4),
    cylinder_model->values.at(5)};
}


inline Eigen::Vector3d convertPCLPointToEigenVector(const PointT& point)
{
  return Eigen::Vector3d {point.x, point.y, point.z};
}


inline geometry_msgs::Point convertPCLPointToGeometryPoint(
    const PointT& pcl_pt)
{
  geometry_msgs::Point geom_pt;

  geom_pt.x = pcl_pt.x;
  geom_pt.y = pcl_pt.y;
  geom_pt.z = pcl_pt.z;

  return geom_pt;
}


inline Eigen::Vector3d normalizeVector(const Eigen::Vector3d& vec)
{
  return vec / vec.squaredNorm();
}


/* Compute the distance from pt1 to pt2, projected onto the given direction
 * vector. The distance is positive, if the distance vector (pt2-pt1) points
 * into the same direction as 'direction' does.
 * The returned length is the real length, multiplied by the length of
 * the 'direction' vector.
 */
inline double computeSignedProjectedDistance(const PointT& pt1, const PointT& pt2,
                                      const Eigen::Vector3d& direction)
{
  Eigen::Vector3d vec1 {convertPCLPointToEigenVector(pt1)};
  Eigen::Vector3d vec2 {convertPCLPointToEigenVector(pt2)};

  Eigen::Vector3d distance_vec {vec2 - vec1};

  return distance_vec.dot(direction);
}

} // anonymous ns


namespace shape_detector {

const Eigen::Vector3d CylinderComputer::reference_direction {0., 0., 1.};


DetectedObject3D CylinderComputer::computeObject()
{
  // Extract points that are part of the cylinder.
  inlier_points_ = extractIndexedPoints(cloud_, inlier_indices_);

  // Compute cylinder/object.
  DetectedObject3D obj;

  obj.box.pose.position = computeCentroid();
  obj.box.pose.orientation = computeOrientation();

  obj.box.width = obj.box.depth = computeDiameter();
  obj.box.height = computeHeight();

  return obj;
}


geometry_msgs::Point CylinderComputer::computeCentroid() const
{
  PointT cloud_centroid;
  if (pcl::computeCentroid(*inlier_points_, cloud_centroid) == 0)
    throw runtime_error {"CylinderComputer::computeCentroid: no inlier points"};

  pcl::ModelCoefficients::Ptr axis_model {
    computeCylinderAxisModel(model_coefficients_)};
  PointT cylinder_centroid {projectPointOnLine(cloud_centroid, axis_model)};
  return convertPCLPointToGeometryPoint(cylinder_centroid);
}


geometry_msgs::Quaternion CylinderComputer::computeOrientation() const
{
  const Eigen::Vector3d direction {computeCylinderDirection(model_coefficients_)};

  Eigen::Quaterniond eigen_quaternion {Eigen::Quaterniond::FromTwoVectors(
      reference_direction,
      direction)};

  geometry_msgs::Quaternion geom_quaternion;
  tf::quaternionEigenToMsg(eigen_quaternion, geom_quaternion);
  return geom_quaternion;
}


double CylinderComputer::computeDiameter() const
{
  return 2 * model_coefficients_->values.at(6);
}


double CylinderComputer::computeHeight() const
{
  Eigen::Vector3d direction {computeCylinderDirection(model_coefficients_)};
  direction = normalizeVector(direction);

  PointT reference_point {inlier_points_->points.at(0)};

  double min_distance {0.};
  double max_distance {0.};
  for (const auto& pt : inlier_points_->points) {
    double distance {computeSignedProjectedDistance(reference_point, pt, direction)};
    min_distance = min(min_distance, distance);
    max_distance = max(max_distance, distance);
  }

  return max_distance - min_distance;
}

} // shape_detector
