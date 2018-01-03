/** \file
 * \brief Node for publishing rectangular areas selected with the mouse.
 */

// std.
#include <algorithm>  // min()
#include <cmath>      // abs()

// ROS.
#include <ros/ros.h>

// Message types.
#include <object_detection_2d/MouseEvent.h>
#include <object_detection_2d/Point2D.h>
#include <object_detection_2d/Rect2D.h>

using object_detection_2d::MouseEvent;
using object_detection_2d::Point2D;
using object_detection_2d::Rect2D;

using namespace std;


namespace {

Rect2D createRectFromPoints(const Point2D& p1, const Point2D& p2)
{
  Point2D upper_left;
  upper_left.x = min(p1.x, p2.x);
  upper_left.y = min(p1.y, p2.y);

  Rect2D rect;
  rect.upper_left = upper_left;
  rect.width = abs(p1.x - p2.x);
  rect.height = abs(p1.y - p2.y);

  return rect;
}

} // anonymous ns


/** \brief Publishes rectangular areas that have been selected with a mouse. */
class AreaSelectionPublisher {
public:
  /** \brief Default constructor.
   *
   * Initializes the ROS communication.
   */
  AreaSelectionPublisher()
  {
    mouse_sub_ = nh_.subscribe("mouse_events", 5,
                               &AreaSelectionPublisher::mouseCallback, this);
    selection_pub_ = nh_.advertise<Rect2D>("selection_area", 10);
  }

private:
  void mouseCallback(const MouseEvent::ConstPtr& event)
  {
    if (event->button == MouseEvent::LEFT_BUTTON)
      processLeftButtonEvent(event);
  }

  void processLeftButtonEvent(const MouseEvent::ConstPtr& event)
  {
    switch (event->type) {
    case MouseEvent::PRESS:
      rememberStartPoint(event);
      break;
    case MouseEvent::RELEASE:
      publishSelectionFromStartToEventPoint(event);
      break;
    }
  }

  void rememberStartPoint(const MouseEvent::ConstPtr& event)
  {
    start_pt_ = event->position;
  }

  void publishSelectionFromStartToEventPoint(const MouseEvent::ConstPtr& event)
      const
  {
    Rect2D rect {createRectFromPoints(start_pt_, event->position)};
    selection_pub_.publish(rect);
  }

  // Data members.
  ros::NodeHandle nh_;
  ros::Subscriber mouse_sub_;
  ros::Publisher selection_pub_;

  Point2D start_pt_;
};


/** \brief main function.
 *
 * Creates an AreaSelectionPublisher and lets it respond to ROS events.
 */
int main(int argc, char** argv)
try {
  ros::init(argc, argv, "area_selection");
  AreaSelectionPublisher asp;

  ros::spin();
  return 0;
}
catch (const exception& e) {
  cerr << "std::exception: " << e.what() << '\n';
  return 1;
}
catch (...) {
  cerr << "Unknown exception caught.\n";
  return 1;
}
