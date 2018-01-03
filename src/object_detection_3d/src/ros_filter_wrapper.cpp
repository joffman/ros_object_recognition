/** \file
 * \brief Implementation of the ROSFilterWrapper class.
 */

// std.
#include <stdexcept>  // runtime_error

// Our headers.
#include <object_detection_3d/ros_filter_wrapper.h>
#include <object_detection_3d/filter.h>


namespace object_detection_3d {

ROSFilterWrapper::ROSFilterWrapper(Filter* filter)
    : filter_ {filter}
{
  if (!filter)
    throw std::runtime_error
    {"ROSFilterWrapper: nullptr passed to constructor"};

  sub_ = nh_.subscribe("input_points", 1,
                       &ROSFilterWrapper::cloudCallback, this);
  pub_ = nh_.advertise<PointCloudT>("output_points", 10);
}


void ROSFilterWrapper::cloudCallback(const PointCloudT::ConstPtr& cloud)
{
  PointCloudT::Ptr filtered_cloud {filter_->filter(cloud)};
  pub_.publish(filtered_cloud);
}

} // object_detection_3d
