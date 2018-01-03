/** \file
 * \brief Definition of the ShapeDetector class.
 */

#ifndef SHAPE_DETECTOR_SHAPE_DETECTOR_H
#define SHAPE_DETECTOR_SHAPE_DETECTOR_H

// std.
#include <utility>    // pair

// PCL.
#include <pcl/PointIndices.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

// object_detection_*.
#include <object_detection_3d_msgs/DetectedObject3D.h>
#include <object_detection_3d_msgs/DetectedObject3DArray.h>
#include <object_detection_3d/detector.h>
#include <object_detection_3d/pcl_types.h>

// Headers of this package.
#include "shape_enum.h"


namespace shape_detector {

class ParameterManager;


// TODO: This is probably not the right place for those using's.
using PointT = object_detection_3d::PointT;
using PointCloudT = object_detection_3d::PointCloudT;

using NormalT = object_detection_3d::NormalT;
using NormalCloudT = object_detection_3d::NormalCloudT;


/** \brief Detects 3-dimensional shapes.
 */
class ShapeDetector : public object_detection_3d::Detector {
public:
  /** \brief Creates a ShapeDetector and initializes its parameter-manager.
   *
   * \param[in] Pointer to a valid ParameterManager.
   */
  explicit ShapeDetector(ParameterManager* pm);

  /** \brief Tries to detect a 3-dimensional shape.
   *
   * \note The parameters (including the shape) of this algorithm are managed
   *  by the ParameterManager data member.
   *
   * \return An array of objects. This array contains one object if the shape
   *  was found; it contains zero objects otherwise.
   *
   * \note Even though an array of 3-dimensional objects is returned, this
   *  array can contain at most one object. This is done to adhere to the
   *  signature used by all Detectors.
   */
  object_detection_3d_msgs::DetectedObject3DArray detect(
      const PointCloudT::ConstPtr& cloud) override;

private:
  std::pair<object_detection_3d_msgs::DetectedObject3D, bool> tryDetectObject(
      const PointCloudT::ConstPtr& cloud);

  std::pair<pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr>
      segmentShapeFromCloud(const PointCloudT::ConstPtr& cloud,
                            const NormalCloudT::ConstPtr& normals);

  pcl::SACSegmentationFromNormals<PointT, NormalT>
      createSACSegmentation() const;

  // Data members.
  ParameterManager* param_manager_;
};

} // shape_detector

#endif  // SHAPE_DETECTOR_SHAPE_DETECTOR_H
