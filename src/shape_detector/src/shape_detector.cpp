/** \file
 * \brief Implementation of the ShapeDetector class.
 */

// std.
#include <memory>   // unique_ptr
#include <tuple>    // tie()
#include <utility>  // pair
#include <stdexcept>  // runtime_error

// PCL.
#include <pcl/PointIndices.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

// Eigen.
#include <Eigen/Dense>      // Vector3d
#include <Eigen/Geometry>   // Quaternion
#include <eigen_conversions/eigen_msg.h>    // tf::quaternionEigenToMsg()

// ROS.
#include <ros/ros.h>

// object_detection_*.
#include <object_detection_3d_msgs/DetectedObject3D.h>
#include <object_detection_3d_msgs/DetectedObject3DArray.h>
#include <object_detection_3d/detector.h>
#include <object_detection_3d/pcl_types.h>

// Headers of this package.
#include <shape_detector/ParametersConfig.h>
#include <shape_detector/shape_detector.h>
#include <shape_detector/shape_enum.h>
#include <shape_detector/object_computer.h>
#include <shape_detector/parameter_manager.h>

using object_detection_3d_msgs::DetectedObject3D;
using object_detection_3d_msgs::DetectedObject3DArray;
using namespace std;


namespace shape_detector {

namespace {

/** \brief Compute normals for point cloud.
 * \param[in] cloud Pointer to input cloud.
 * \param[in] num_nearest_neighbors Number of nearest neighbors, used for
 *  computation of normals.
 * \return Pointer to cloud of normals.
 */
NormalCloudT::Ptr computeNormals(
    const PointCloudT::ConstPtr cloud,
    const int num_nearest_neighbors)
{
  // Create search tree for the normal estimation.
  pcl::search::KdTree<PointT>::Ptr tree_ptr {new pcl::search::KdTree<PointT>};

  // Create NormalEstimation object and set its parameters.
  pcl::NormalEstimation<PointT, NormalT> normal_estimator;
  normal_estimator.setSearchMethod(tree_ptr);
  normal_estimator.setInputCloud(cloud);
  normal_estimator.setKSearch(num_nearest_neighbors);

  // Compute normals and return pointer to them.
  NormalCloudT::Ptr normals_ptr {new NormalCloudT};
  normal_estimator.compute(*normals_ptr);

  return normals_ptr;
}

} // anonymous ns


ShapeDetector::ShapeDetector(ParameterManager* pm)
  : param_manager_ {pm}
{
  if (!pm)
    throw runtime_error {"ShapeDetector: nullptr passed to constructor"};
}


DetectedObject3DArray ShapeDetector::detect(const PointCloudT::ConstPtr& cloud)
{
  DetectedObject3DArray objects;

  if (!cloud->points.empty()) {
    // C++17: auto [object, object_found] = tryDetectObject(cloud);
    bool object_found;
    DetectedObject3D object;
    tie(object, object_found) = tryDetectObject(cloud);
    if (object_found)
      objects.objects.push_back(object);
  }
  
  // fillObjectsHeader(objects);
  objects.header.stamp = ros::Time::now();
  objects.header.frame_id = cloud->header.frame_id;

  return objects;
}


pair<DetectedObject3D, bool> ShapeDetector::tryDetectObject(
    const PointCloudT::ConstPtr& cloud)
{
  NormalCloudT::Ptr normals {
    computeNormals(cloud, param_manager_->numNearestNeighbors())
  };

  // TODO: Try to use unique_ptr's instead.
  // C++17: auto [inlier_indices, model_coefficients] = segmentShapeFromCloud(cloud, normals);
  pcl::PointIndices::Ptr inlier_indices {new pcl::PointIndices};
  pcl::ModelCoefficients::Ptr model_coefficients {
    new pcl::ModelCoefficients
  };
  tie(inlier_indices, model_coefficients) =
      segmentShapeFromCloud(cloud, normals);

  bool object_found {false};
  DetectedObject3D object;
  if (!inlier_indices->indices.empty()) { // object found
    unique_ptr<ObjectComputer> object_computer {
      createObjectComputerForShape(param_manager_->shape())};

    // TODO: object_computer->setName(object_name_);
    object_computer->setCloud(cloud);
    object_computer->setInlierIndices(inlier_indices);
    object_computer->setModelCoefficients(model_coefficients);

    object = object_computer->computeObject();
    object_found = true;
  }
  return make_pair(object, object_found);
}


pair<pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr>
ShapeDetector::segmentShapeFromCloud(const PointCloudT::ConstPtr& cloud,
                                     const NormalCloudT::ConstPtr& normals)
  // TODO: Can we use unique_ptrs instead?
{
    pcl::SACSegmentationFromNormals<PointT, NormalT> sac_segmentation {
      createSACSegmentation()
    };
    sac_segmentation.setInputCloud(cloud);
    sac_segmentation.setInputNormals(normals);

    pcl::PointIndices::Ptr inlier_indices {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr model_coefficients {
      new pcl::ModelCoefficients
    };
    sac_segmentation.segment(
        *inlier_indices,
        *model_coefficients
        );
    
    return make_pair(inlier_indices, model_coefficients);
}


pcl::SACSegmentationFromNormals<PointT,NormalT>
ShapeDetector::createSACSegmentation() const
// TODO: Maybe make this one a member of param_manager_.
{
  pcl::SACSegmentationFromNormals<PointT, NormalT> sac_segmentation;

  sac_segmentation.setOptimizeCoefficients(param_manager_->optimizeCoefficients());
  sac_segmentation.setModelType(static_cast<int>(param_manager_->shape()));
  sac_segmentation.setMethodType(pcl::SAC_RANSAC);
  sac_segmentation.setNormalDistanceWeight(param_manager_->normalDistanceWeight());
  sac_segmentation.setMaxIterations(param_manager_->maxIterations());
  sac_segmentation.setDistanceThreshold(param_manager_->distanceThreshold());
  sac_segmentation.setRadiusLimits(param_manager_->minRadius(), param_manager_->maxRadius());

  return sac_segmentation;
}

} // shape_detector
