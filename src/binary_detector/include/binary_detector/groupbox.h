/** \file
 * \brief Definition of the GroupBox class.
 */

#ifndef BINARY_DETECTOR_GROUPBOX_H
#define BINARY_DETECTOR_GROUPBOX_H

// TODO: Add widget for object-name.

// Qt.
#include <QGroupBox>

// Our headers.
#include <object_detection/observer.h>

// Forward declarations.
class QWidget;
class QLabel;
class QSlider;
class QLCDNumber;


namespace binary_detector {

class ObservedParameterManager;

/** \brief QGroupBox for accessing the parameters of a BinaryDetector object.
 */
class GroupBox : public QGroupBox,
    public object_detection::Observer {
  Q_OBJECT

public:
  /** \brief Creates and initializes the GroupBox.
   *
   * \param[in] parent Pointer to QWidget that serves as a parent of the
   *  GroupBox.
   */
  explicit GroupBox(QWidget* parent = nullptr); 

  /** \brief Updates the view of the GroupBox so that it displays the current
   *  values of the parameters.
   */
  void update() override;

  /** \brief Sets the parameter-manager of the BinaryDetector.
   *
   * The ObservedParameterManager manages the parameters of a BinaryDetector
   * object.
   * The manager represents the Model of the MVC-pattern, while the GroupBox
   * represents the View and the Controller.
   */
  void setParameterManager(ObservedParameterManager* const);

  /** \brief Number of ticks of each slider in the GroupBox. */
  static constexpr int num_slider_ticks {1000};
  // TODO: I don't like that it's public; necessary for use by helper functions.

signals:
  void minLCDSignal(double);
  void maxLCDSignal(double);

private slots:
  void requestMinLengthChange(const int) const;
  void requestMaxLengthChange(const int) const;

private:
  ObservedParameterManager* param_manager_ {nullptr};

  QLabel* min_length_label_;
  QSlider* min_length_slider_;
  QLCDNumber* min_length_lcd_;

  QLabel* max_length_label_;
  QSlider* max_length_slider_;
  QLCDNumber* max_length_lcd_;
};

} // binary_detector

#endif  // BINARY_DETECTOR_GROUPBOX_H
