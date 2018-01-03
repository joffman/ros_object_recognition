/** \file
 * \brief Definition of the Filter class.
 */

// TODO: Do we need a virtual destructor?

#ifndef OBJECT_DETECTION_3D_FILTER_H
#define OBJECT_DETECTION_3D_FILTER_H

#include "pcl_types.h"


namespace object_detection_3d {

/** \brief Abstract filter for filtering a point cloud. */
class Filter {
public:
  /** \brief Filters the given point cloud.
   * \param[in] cloud Point cloud that is filtered.
   * \return New point cloud that contains the filtered cloud.
   */
  virtual PointCloudT::Ptr filter(const PointCloudT::ConstPtr& cloud) = 0;
};

} // object_detection_3d

#endif  // OBJECT_DETECTION_3D_FILTER_H
