/** \file
 * \brief Definition of the Visualizer class.
 */

#ifndef OBJECT_DETECTION_2D_VIS_VISUALIZER_H
#define OBJECT_DETECTION_2D_VIS_VISUALIZER_H

// Qt.
#include <QObject>
#include <QWidget>

// ROS.
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

class QMouseEvent;
namespace object_detection_2d { class InteractiveImage; }


namespace object_detection_2d_vis {

/** \brief Interactive visualizer for showing images and capturing
 *  mouse events.
 */
class Visualizer : public QObject {
  Q_OBJECT

public:
  /** \brief Default constructor.
   *
   * Initializes ROS communication and the gui widgets.
   */
  Visualizer();

  /** \brief Returns true if the gui is visible, false if it has been closed. */
  bool isVisible() const { return window_.isVisible(); }

private slots:
  void mousePressSlot(const QMouseEvent*) const;
  void mouseMoveSlot(const QMouseEvent*) const;
  void mouseReleaseSlot(const QMouseEvent*) const;

private:
  void initROSCommunication();
  void initGUI();
  void initVisualElements();
  void connectSignalsAndSlots();

  void imageCallback(const sensor_msgs::Image::ConstPtr& image_msg) const;
  void convertAndShowImageMsg(const sensor_msgs::Image::ConstPtr& image_msg)
      const;

  // Data members.
  ros::NodeHandle node_handle_;
  ros::Subscriber image_sub_;
  ros::Publisher mouse_pub_;

  QWidget window_;
  object_detection_2d::InteractiveImage* inter_image_;
};

} // object_detection_2d_vis

#endif  // OBJECT_DETECTION_2D_VIS_VISUALIZER_H
