/** \file
 * \brief The objects2d_to_objects3d node.
 */

// std.
#include <iostream>
#include <exception>
#include <algorithm>  // min(), max()
#include <cmath>      // tan(), atan(), M_PI

// ROS.
#include <ros/ros.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Header.h>

// Headers of this project.
#include <object_detection_2d_msgs/DetectedObject2DArray.h>
#include <object_detection_2d_msgs/DetectedObject2D.h>
#include <object_detection_3d_msgs/DetectedObject3DArray.h>
#include <object_detection_3d_msgs/DetectedObject3D.h>
#include <object_detection_3d_msgs/OrientedBox.h>

using std_msgs::Header;
using geometry_msgs::Polygon;
using geometry_msgs::Point32;
using object_detection_2d_msgs::DetectedObject2DArray;
using object_detection_2d_msgs::DetectedObject2D;
using object_detection_3d_msgs::DetectedObject3DArray;
using object_detection_3d_msgs::DetectedObject3D;
using object_detection_3d_msgs::OrientedBox;
using namespace std;


inline Header create3DHeaderFrom2DHeader(const Header& h2d)
{
  Header h3d;
  h3d.stamp = h2d.stamp;
  h3d.frame_id = "camera_rgb_frame";
  return h3d;
}


struct Rectangle {
  float x_min;
  float x_max;

  float y_min;
  float y_max;
};


/** \brief Compute bounding rectangle for a non-empty polygon. */
inline Rectangle computePolygonBoundingRect(const Polygon& polygon)
{
  Rectangle rec {
    polygon.points[0].x,
    polygon.points[0].x,
    polygon.points[0].y,
    polygon.points[0].y
  };
  for_each(polygon.points.begin(), polygon.points.end(),
           [&rec](const Point32& p){
           rec.x_min = min(rec.x_min, p.x);
           rec.x_max = max(rec.x_max, p.x);
           rec.y_min = min(rec.y_min, p.y);
           rec.y_max = max(rec.y_max, p.y);
           });
  return rec;
}


constexpr double degToRad(double deg)
{
  return deg * (M_PI/180);
}


/** \brief Class for computing 3D boxes from 2D rectangles. */
class BoxComputer {
public:
  /** \brief Compute 3D bounding box from rectangle in image that surrounds an
   * object that is standing on the ground.
   *
   * First, a rectangular front side of the box is computed from the rectangle.
   * The bounding box consists of four such sides with right angles between them.
   */
  // TODO: Clean this up.
  OrientedBox computeFromRectangle(const Rectangle& rec) const
  {
    if (rec.y_max < half_image_height)
      throw runtime_error {"BoxComputer::computeFromRectangle: "
        "object does not stand on the ground"};

    // Compute real distances from pixel coordinates.
    const double distance_m {computeBoxDistanceFromBottomYCoord(rec.y_max)};
    const double height_m {
      computeBoxHeightFromTopYCoord(rec.y_min, distance_m)};
    const double left_edge_distance_m {
      computeHorizontalDistanceOfXCoord(rec.x_min, distance_m)};
    const double right_edge_distance_m {
      computeHorizontalDistanceOfXCoord(rec.x_max, distance_m)};

    // Create box from real distances.
    OrientedBox box;
    box.width = box.depth = right_edge_distance_m - left_edge_distance_m;
      // The depth is just an assumption/approximation.
    box.height = height_m;
    box.pose.position.x = distance_m + box.depth/2;
    box.pose.position.y = -(left_edge_distance_m + box.width/2);
      // minus because the robot y-axis points to the left
    box.pose.position.z = -camera_height_m + height_m/2;
    box.pose.orientation.w = 1.;
    return box;
  }

private:
  /** \brief Returns the distance of points on the ground that have the
   * given y-coordinate in the image.
   */
  double computeBoxDistanceFromBottomYCoord(float y_bot) const
  {
    const double angle_base {angleOfYCoord(y_bot)};
    return camera_height_m / tan(angle_base);
  }

  /** \brief Computes the height of the 3D box that is the given distance away
   * from the camera and whose highest point is represented by a pixel with
   * the given y-coordinate.
   */
  double computeBoxHeightFromTopYCoord(float y_top, double distance) const // TODO: change name
  {
    const double angle_top {angleOfYCoord(y_top)};
    return camera_height_m - tan(angle_top) * distance;
  }

  /** \brief Computes the angle between the horizontal plane through the optical
   * axis and rays from the pinhole to points with the given y-coordinate
   * in the image.
   */
  double angleOfYCoord(float y) const
  {
    return atan((y - half_image_height) / focal_x);
  }

  /** \brief Computes horizontal distance from a point with the given distance
   * and the given x-coordinate in the image to the vertical plane through the
   * optical axis.
   */
  double computeHorizontalDistanceOfXCoord(float x, double distance) const // TODO: change name
  {
    return (x - half_image_width) * distance / focal_x;
  }


  // Camera intrinsics; taken from astra datasheet.
  static constexpr int half_image_width {640/2};
  static constexpr int half_image_height {480/2};
  static constexpr double fov_x_deg {60};
  static constexpr double fov_y_deg {49.5};
  static constexpr double focal_x
  {half_image_width / tan(degToRad(fov_x_deg/2))};
  static constexpr double focal_y
  {half_image_height / tan(degToRad(fov_y_deg/2))};

  // Height of optical axis.
  static constexpr double camera_height_m {0.32};
};


/** \brief Computes a 3D object from a non-empty 2D polygon.
 *
 * This function approximates the given 2D polygon by its bounding rectangle.
 * Then it computes a 3D box that represents this rectangle.
 * It assumes that the polygon represents an object that is standing on the
 * ground.
 */
inline OrientedBox compute3DBoxFrom2DPolygon(const Polygon& polygon)
{
  if (polygon.points.empty())
    throw runtime_error {"empty polygon passed to compute3DBoxFrom2DPolygon"};

  Rectangle rec {computePolygonBoundingRect(polygon)};

  BoxComputer bc;
  OrientedBox box {bc.computeFromRectangle(rec)};
  return box;
}


/** \brief Returns a 3D object that represents a 2D object. */
inline DetectedObject3D object2DToObject3D(const DetectedObject2D& obj2d)
{
  DetectedObject3D obj3d;
  obj3d.name = obj2d.name;
  obj3d.box = compute3DBoxFrom2DPolygon(obj2d.polygon);
  return obj3d;
}


/** \brief Tries to convert a 2D object into a 3D object.
 * If it succeeds, it pushs the object into the given vector.
 */
inline void tryPushbackConvertedObject(
    vector<DetectedObject3D>& objects3d, const DetectedObject2D& obj2d)
try {
  objects3d.push_back(object2DToObject3D(obj2d));
}
catch (const exception& e) {
  ROS_INFO_STREAM("exception while converting 2D object to 3D object: "
                  << e.what());
}


/** \brief Returns a vector of 3D objects, each representing a 2D object. */
inline vector<DetectedObject3D> convert2DObjectsTo3DObjects(
    const vector<DetectedObject2D>& objects2d)
{
  vector<DetectedObject3D> objects3d;
  for (const auto& obj2d : objects2d)
    tryPushbackConvertedObject(objects3d, obj2d);
  return objects3d;
}


/** \brief Class for converting 2D objects into corresponding 3D objects.
 *
 * An object of this class subscribes to DetectedObject2DArray messages
 * and publishes corresponding DetectedObject3DArray messages.
 * Each 3D object represents a 2D object.
 * The bounding box of a 3D object is computed from the 2D polygon of the
 * 2D object. This is done under the assumption that the object is
 * standing on the ground (with the camera attached to the TurtleBot)
 * and that its depth is roughly equal to its width.
 */
class ObjectConverter {
public:
  ObjectConverter()
  {
    sub_ = nh_.subscribe("input_2d_objects", 10,
                         &ObjectConverter::objectsCallback, this);
    pub_ = nh_.advertise<DetectedObject3DArray>("output_3d_objects", 10);
  }

private:
  void objectsCallback(const DetectedObject2DArray::ConstPtr& objects2d_msg)
      const
  {
    DetectedObject3DArray objects3d;
    objects3d.header = create3DHeaderFrom2DHeader(objects2d_msg->header);
    objects3d.objects = convert2DObjectsTo3DObjects(objects2d_msg->objects);
    pub_.publish(objects3d);
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
};


int main(int argc, char** argv)
try {
  ros::init(argc, argv, "objects2d_to_objects3d");
  ObjectConverter oc;
  ros::spin();
  return 0;
}
catch (const exception& e) {
  cerr << "std::exception: " << e.what() << '\n';
  return 1;
}
catch (...) {
  cerr << "Unknown exception; terminating program.\n";
  return 1;
}
