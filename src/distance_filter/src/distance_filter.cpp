/** \file
 * \brief Implementation of the DistanceFilter class.
 */

// std.
#include <stdexcept>

// PCL.
#include <pcl/filters/passthrough.h>

// Our headers.
#include <distance_filter/distance_filter.h>
#include <distance_filter/parameter_manager.h>


namespace distance_filter {

DistanceFilter::DistanceFilter(ParameterManager* pm)
    : param_manager_ {pm}
{
  if (!pm)
    throw std::runtime_error {"DistanceFilter: nullptr passed to constructor"};
}


DistanceFilter::PointCloudT::Ptr DistanceFilter::filter(
    const PointCloudT::ConstPtr& cloud)
{
  pcl::PassThrough<PointT> passthrough {createPassThroughForCloud(cloud)};

  PointCloudT::Ptr filtered_cloud {new PointCloudT};
  passthrough.filter(*filtered_cloud);
  return filtered_cloud;
}

pcl::PassThrough<DistanceFilter::PointT>
DistanceFilter::createPassThroughForCloud(
    const PointCloudT::ConstPtr& cloud) const
{
  pcl::PassThrough<PointT> passthrough;

  passthrough.setInputCloud(cloud);
  passthrough.setFilterFieldName("z");
  passthrough.setFilterLimits(param_manager_->minDistance(),
                              param_manager_->maxDistance());

  return passthrough;
}

} // distance_filter
