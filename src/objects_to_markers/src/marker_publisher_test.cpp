/** \file
 * \brief Simple node to test how markers work.
 *
 * A test-marker is published in an infinite loop.
 * This allows for investigation of the marker, e.g. in rviz.
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

using visualization_msgs::Marker;


Marker createTestMarker()
{
  Marker m;

  m.type = Marker::CUBE;
  m.header.frame_id = "camera_depth_frame";

  m.pose.position.x = 1;
  m.pose.position.y = 2;
  m.pose.position.z = 3;

  m.pose.orientation.x = 0;
  m.pose.orientation.y = 0;
  m.pose.orientation.z = 0.487864315758;
  m.pose.orientation.w = 0.872919474757;

  m.scale.x = 2;
  m.scale.y = 2;
  m.scale.z = 6;

  m.lifetime = ros::Duration{0.2};

  m.color.r = 1;
  m.color.g = 0;
  m.color.b = 0;
  m.color.a = 1;

  return m;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "marker_publisher");
  ros::NodeHandle nh;

  ros::Publisher pub {nh.advertise<Marker>("markers", 10)};

  Marker m {createTestMarker()};

  ros::Rate rate {2};
  while (ros::ok()) {
    pub.publish(m);
    rate.sleep();
    ros::spinOnce();
  }
  
  return 0;
}
