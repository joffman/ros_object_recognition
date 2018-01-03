/** \file
 * \brief Implementation of the InteractiveImage class.
 */

// Qt.
#include <QPainter>
#include <QPaintEvent>
#include <QMouseEvent>
#include <QWidget>
#include <QImage>
#include <QRubberBand>
#include <QSize>
#include <QRect>

// OpenCV.
#include <cv_bridge/cv_bridge.h>

// Our headers.
#include <object_detection_2d/interactive_image.h>


namespace object_detection_2d {

InteractiveImage::InteractiveImage(QWidget* parent)
    : QWidget{parent},
    rubber_band_{new QRubberBand{QRubberBand::Rectangle, this}},
    image_initialized_{false}
{
  setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  rubber_band_->hide();
}


void InteractiveImage::paintEvent(QPaintEvent*)
{
  QPainter painter{this};
  painter.drawImage(QPoint{0, 0}, q_image_);
}


void InteractiveImage::mousePressEvent(QMouseEvent* event)
{
  selection_origin_ = event->pos();
  rubber_band_->setGeometry(QRect{selection_origin_, QSize{}});
  rubber_band_->show();
  setCursor(Qt::CrossCursor);

  emit mousePressSignal(event);
}


void InteractiveImage::mouseMoveEvent(QMouseEvent* event)
{
  rubber_band_->setGeometry(
      QRect{selection_origin_, event->pos()}.normalized()
      );

  emit mouseMoveSignal(event);
}


void InteractiveImage::mouseReleaseEvent(QMouseEvent* event)
{
  rubber_band_->hide();
  unsetCursor();
  emit areaSelected(QRect{selection_origin_, event->pos()});

  emit mouseReleaseSignal(event);
}


void InteractiveImage::showImage(const cv::Mat& cv_image)
{
  copyToQtImage(cv_image);
  update();
}


QSize InteractiveImage::sizeHint() const
{
  return q_image_.size();
}


void InteractiveImage::copyToQtImage(const cv::Mat& cv_image)
{
  if (!image_initialized_) {
    q_image_ = QImage{QSize{cv_image.cols, cv_image.rows}, QImage::Format_RGB888};
    image_initialized_ = true;
    updateGeometry();
  }

  cv::Mat cv_header_to_qt_image(
      cv::Size{q_image_.width(), q_image_.height()},
      CV_8UC3,
      q_image_.bits()
      );
  cv::cvtColor(cv_image, cv_header_to_qt_image, cv::COLOR_BGR2RGB);
}

} // object_detection_2d
