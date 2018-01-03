/** \file
 * \brief Common typedefs used for processing point clouds.
 */

#ifndef OBJECT_DETECTION_3D_PCL_TYPES_H
#define OBJECT_DETECTION_3D_PCL_TYPES_H

#include <pcl/point_types.h>      // PointXYZ, Normal
#include <pcl_ros/point_cloud.h>  // PointCloud<PointT>


namespace object_detection_3d {

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

using NormalT = pcl::Normal;
using NormalCloudT = pcl::PointCloud<NormalT>;

} // object_detection_3d

#endif  // OBJECT_DETECTION_3D_PCL_TYPES_H
