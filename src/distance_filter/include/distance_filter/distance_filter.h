/** \file
 * \brief Definition of the DistanceFilter class.
 */

#ifndef DISTANCE_FILTER_DISTANCE_FILTER_H
#define DISTANCE_FILTER_DISTANCE_FILTER_H

// PCL.
#include <pcl/filters/passthrough.h>

// Our headers.
#include <object_detection_3d/pcl_types.h>
#include <object_detection_3d/filter.h>


namespace distance_filter {

class ParameterManager;


/** \brief Filter for filtering out all points of a point cloud with a
 *  distance outside an adjustable range.
 */
class DistanceFilter : public object_detection_3d::Filter {
  using PointT = object_detection_3d::PointT;
  using PointCloudT = object_detection_3d::PointCloudT;

public:
  /** \brief Creates a DistanceFilter and initializes its parameter-manager.
   *
   * \param[in] Pointer to a valid ParameterManager.
   */
  explicit DistanceFilter(ParameterManager* pm);

  /** \brief Filters the given point cloud.
   *
   * The point cloud is filtered so that only those points fulfilling the
   * condition 'min_distance <= distance <= max_distance' are retained.
   *
   * \param[in] cloud Point cloud that is filtered.
   * \return Point cloud containing only the points that passed through the
   *  filter.
   *
   * \note The distance-limits are maintained by the ParameterManager member.
   */
  PointCloudT::Ptr filter(const PointCloudT::ConstPtr& cloud) override;

private:
  pcl::PassThrough<PointT> createPassThroughForCloud(
      const PointCloudT::ConstPtr& cloud) const;

  ParameterManager* param_manager_;
};

} // distance_filter

#endif  // DISTANCE_FILTER_DISTANCE_FILTER_H
