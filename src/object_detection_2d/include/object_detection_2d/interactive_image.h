/** \file
 * \brief Definition of the InteractiveImage class.
 */

#ifndef OBJECT_DETECTION_2D_INTERACTIVE_IMAGE_H
#define OBJECT_DETECTION_2D_INTERACTIVE_IMAGE_H

// Qt.
#include <QWidget>
#include <QImage>
#include <QPoint>
#include <QSize>

// Forward declarations.
class QRect;
class QPaintEvent;
class QMouseEvent;
class QRubberBand;
class QPoint;

namespace cv {
class Mat;
}


namespace object_detection_2d {

/** \brief QWidget for rendering an image and allowing the user to
 *  interact with it by clicking into it with his mouse.
 */
class InteractiveImage : public QWidget {
  Q_OBJECT

public:
  explicit InteractiveImage(QWidget* parent = nullptr);

  /** \brief Renders given image. */
  void showImage(const cv::Mat& cv_image);

  /** \brief Returns the ideal size of the widget. */
  QSize sizeHint() const; // override?!

protected:
  void paintEvent(QPaintEvent*) override;
  void mousePressEvent(QMouseEvent*) override;
  void mouseMoveEvent(QMouseEvent*) override;
  void mouseReleaseEvent(QMouseEvent*) override;

signals:
  void mousePressSignal(const QMouseEvent*);
  void mouseMoveSignal(const QMouseEvent*);
  void mouseReleaseSignal(const QMouseEvent*);
  void areaSelected(const QRect& area);

private:
  void copyToQtImage(const cv::Mat& cv_image);

  // Data members.
  QImage q_image_;
  QRubberBand* rubber_band_;
  bool image_initialized_;
  QPoint selection_origin_;
};

} // object_detection_2d

#endif  // OBJECT_DETECTION_2D_INTERACTIVE_IMAGE_H
