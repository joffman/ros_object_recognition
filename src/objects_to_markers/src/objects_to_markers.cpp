/** \file
 * \brief Implementation of the ObjectsToMarkers class.
 */

// ROS.
#include <ros/ros.h>

// Msg types.
#include <std_msgs/Header.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <object_detection_3d_msgs/DetectedObject3D.h>
#include <object_detection_3d_msgs/DetectedObject3DArray.h>

// Headers of this package.
#include <objects_to_markers/objects_to_markers.h>

using std_msgs::Header;

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

using object_detection_3d_msgs::DetectedObject3D;
using object_detection_3d_msgs::DetectedObject3DArray;


namespace {

/** \brief A MarkerBuilder builds a marker from an object.
 */
class MarkerBuilder {
public:
  MarkerBuilder()
  {
    id_ = 0;
  }

  void setHeader(const Header& h) { header_ = h; }

  void setLifetime(const ros::Duration& lt) { lifetime_ = lt; }
 
  Marker buildMarkerFromObject(const DetectedObject3D& obj)
  {
    Marker m;

    m.header = header_;
    m.lifetime = lifetime_;
    m.type = Marker::CUBE;
    m.id = id_++;

    m.text = obj.name;

    m.pose = obj.box.pose;

    m.scale.x = obj.box.depth;
    m.scale.y = obj.box.width; // TODO: width or depth??
    m.scale.z = obj.box.height;

    m.color.r = 0.;
    m.color.g = 1.;
    m.color.b = 0.;
    m.color.a = 0.5;

    return m;
  }

private:
  Header header_;
  ros::Duration lifetime_;
  static int id_;
};

int MarkerBuilder::id_ {0};

} // anonymous ns


namespace objects_to_markers {

ObjectsToMarkers::ObjectsToMarkers()
{
  sub_ = nh_.subscribe("detected_objects_3d", 1,
                       &ObjectsToMarkers::objectsCallback, this);
  pub_ = nh_.advertise<MarkerArray>("object_markers", 10);
}


void ObjectsToMarkers::objectsCallback(
    const DetectedObject3DArray::ConstPtr& objects) const
{
  MarkerArray marker_array;

  MarkerBuilder builder;
  builder.setHeader(objects->header);
  builder.setLifetime(param_manager_.lifetime());

  for (const auto& obj : objects->objects) {
    Marker m {builder.buildMarkerFromObject(obj)};
    marker_array.markers.push_back(m);
  }

  pub_.publish(marker_array);
}

} // objects_to_markers
