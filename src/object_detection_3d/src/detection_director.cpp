/** \file
 * \brief Implementation of the DetectionDirector class.
 */

// Our headers.
#include <object_detection_3d_msgs/DetectedObject3DArray.h>
#include <object_detection_3d/detection_director.h>
#include <object_detection_3d/filter.h>
#include <object_detection_3d/detector.h>

using object_detection_3d_msgs::DetectedObject3DArray;


namespace object_detection_3d {

DetectionDirector::DetectionDirector()
    : detector_ {nullptr}
{
  subscriber_ =
      node_handle_.subscribe("input_points", 1,
                             &DetectionDirector::cloudCallback, this);
  publisher_ =
      node_handle_.advertise<DetectedObject3DArray>("detected_objects_3d", 10);
}


void DetectionDirector::addFilter(Filter* filter)
{
  if (filter)
    filters_.push_back(filter);
}


void DetectionDirector::setDetector(Detector* detector)
{
  detector_ = detector;
}


void DetectionDirector::cloudCallback(
    const PointCloudT::ConstPtr& cloud) const
{
  if (!detector_)
    return;

  // Filter cloud.
  PointCloudT::Ptr filtered_cloud {new PointCloudT};
  *filtered_cloud = *cloud;
  for (auto filter_ptr : filters_)
    filtered_cloud = filter_ptr->filter(filtered_cloud);

  // Detect objects and publish them.
  DetectedObject3DArray objects {detector_->detect(filtered_cloud)};
  if (!objects.objects.empty())
    publisher_.publish(objects);
}

} // object_detection_3d
