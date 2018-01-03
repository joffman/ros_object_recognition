/** \file
 * \brief Implementation of the Visualizer class.
 */

// Qt.
#include <QMouseEvent>
#include <QHBoxLayout>

// OpenCV.
#include <cv_bridge/cv_bridge.h>

// ROS.
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// object_detection_*.
#include <object_detection_2d/interactive_image.h>
#include <object_detection_2d/MouseEvent.h>
#include <object_detection_2d_vis/visualizer.h>

using object_detection_2d::InteractiveImage;
using object_detection_2d::MouseEvent;


namespace {

int convertQButtonToMouseEventButton(Qt::MouseButton q_button)
{
  switch (q_button) {
  case Qt::NoButton:
    return MouseEvent::NO_BUTTON;
  case Qt::LeftButton:
    return MouseEvent::LEFT_BUTTON;
  case Qt::MiddleButton:
    return MouseEvent::MIDDLE_BUTTON;
  case Qt::RightButton:
    return MouseEvent::RIGHT_BUTTON;
  default:
    return MouseEvent::UNKNOWN_BUTTON;
  }
}


MouseEvent createMouseEventMsg(const QMouseEvent* q_event, int type)
{
  MouseEvent event_msg;

  event_msg.button = convertQButtonToMouseEventButton(q_event->button());
  event_msg.position.x = q_event->x();
  event_msg.position.y = q_event->y();
  event_msg.type = type;

  return event_msg;
}

} // anonymous ns


namespace object_detection_2d_vis {

Visualizer::Visualizer()
{
  initROSCommunication();
  initGUI();
}


void Visualizer::initROSCommunication()
{
  image_sub_ = node_handle_.subscribe("input_image", 1,
                                      &Visualizer::imageCallback, this);
  mouse_pub_ = node_handle_.advertise<MouseEvent>("mouse_events", 10);
}


void Visualizer::initGUI()
{
  initVisualElements();
  connectSignalsAndSlots();
}


void Visualizer::initVisualElements()
{
  inter_image_ = new InteractiveImage;
  QHBoxLayout* layout {new QHBoxLayout};
    // Without a layout the size is not updated automatically.
  layout->addWidget(inter_image_);
  window_.setLayout(layout);
  window_.show();
}


void Visualizer::connectSignalsAndSlots()
{
  connect(inter_image_, &InteractiveImage::mousePressSignal,
          this, &Visualizer::mousePressSlot);
  connect(inter_image_, &InteractiveImage::mouseMoveSignal,
          this, &Visualizer::mouseMoveSlot);
  connect(inter_image_, &InteractiveImage::mouseReleaseSignal,
          this, &Visualizer::mouseReleaseSlot);
}


void Visualizer::imageCallback(const sensor_msgs::Image::ConstPtr& image_msg)
  const
try {
  convertAndShowImageMsg(image_msg);
}
catch (const cv_bridge::Exception& e) {
  ROS_ERROR_STREAM("cv_bridge::Exception when converting image: " << e.what());
}


void Visualizer::convertAndShowImageMsg(const sensor_msgs::Image::ConstPtr& image_msg)
  const
{
  cv_bridge::CvImage::ConstPtr cv_image_ptr {
      cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8)};
  inter_image_->showImage(cv_image_ptr->image);
}


void Visualizer::mousePressSlot(const QMouseEvent* event) const
{
  mouse_pub_.publish(
      createMouseEventMsg(event, MouseEvent::PRESS));
}


void Visualizer::mouseMoveSlot(const QMouseEvent* event) const
{
  mouse_pub_.publish(
      createMouseEventMsg(event, MouseEvent::MOVE));
}


void Visualizer::mouseReleaseSlot(const QMouseEvent* event) const
{
  mouse_pub_.publish(
      createMouseEventMsg(event, MouseEvent::RELEASE));
}

} // object_detection_2d_vis
