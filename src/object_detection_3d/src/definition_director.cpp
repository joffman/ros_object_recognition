/** \file
 * \brief Implementation of the DefinitionDirector class.
 */

// Our headers.
#include <object_detection_3d/definition_director.h>
#include <object_detection_3d_msgs/DetectedObject3DArray.h>
#include <object_detection_3d/filter.h>
#include <object_detection_3d/detector.h>
#include <object_detection_3d/gui.h>

using object_detection_3d_msgs::DetectedObject3DArray;


namespace object_detection_3d {

DefinitionDirector::DefinitionDirector()
    : gui_ {nullptr},
    detector_ {nullptr}
{
  subscriber_ =
      node_handle_.subscribe("input_points", 1,
                             &DefinitionDirector::cloudCallback, this);
}


void DefinitionDirector::setGUI(GUI* gui)
{
  gui_ = gui;
}


void DefinitionDirector::addFilter(Filter* filter)
{
  if (filter)
    filters_.push_back(filter);
}


void DefinitionDirector::setDetector(Detector* detector)
{
  detector_ = detector;
}


void DefinitionDirector::cloudCallback(const PointCloudT::ConstPtr& cloud)
  const
{
  if (!gui_)
    return;

  PointCloudT::Ptr filtered_cloud {new PointCloudT};
  *filtered_cloud = *cloud;
  for (auto filter_ptr : filters_)
    filtered_cloud = filter_ptr->filter(filtered_cloud);

  gui_->clearRenderWindow();
  gui_->showCloud(cloud);
  gui_->showColoredCloud(filtered_cloud);

  if (detector_) {
    DetectedObject3DArray objects {detector_->detect(filtered_cloud)};
    for (const auto& obj : objects.objects)
      gui_->showObject(obj);
  }
}

} // object_detection_3d
