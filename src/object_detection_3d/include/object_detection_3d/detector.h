/** \file
 * \brief Definition of the Detector class.
 */

// TODO: Do we need a virtual destructor?

#ifndef OBJECT_DETECTION_3D_DETECTOR_H
#define OBJECT_DETECTION_3D_DETECTOR_H

// Our headers.
#include <object_detection_3d_msgs/DetectedObject3DArray.h>
#include "pcl_types.h"


namespace object_detection_3d {

/** \brief Abstract detector class for detecting 3D objects in point clouds. */
class Detector {
public:
  /** \brief Detects objects in a point cloud.
   *
   * \param[in] cloud Point cloud in which the detector looks for objects.
   * \return Array containing the objects found.
   */
  virtual object_detection_3d_msgs::DetectedObject3DArray detect(
      const PointCloudT::ConstPtr& cloud) = 0;
};

} // object_detection_3d

#endif  // OBJECT_DETECTION_3D_DETECTOR_H
