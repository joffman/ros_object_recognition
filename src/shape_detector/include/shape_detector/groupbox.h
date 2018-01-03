/** \file
 * \brief Definition of the GroupBox class.
 */

#ifndef SHAPE_DETECTOR_GROUPBOX_H
#define SHAPE_DETECTOR_GROUPBOX_H

// TODO: Add widget for setting the object-name.

// Qt.
#include <QGroupBox>

// Our headers.
#include <object_detection/observer.h>

class QWidget;
class QVBoxLayout;

namespace shape_detector {

class ObservedParameterManager;
class NormalEstimationGroupBox;
class SegmentationGroupBox;


/** \brief GroupBox for accessing the parameters of a ShapeDetector.
 */
class GroupBox : public QGroupBox,
    public object_detection::Observer {
  Q_OBJECT

public:
  /** \brief Creates and initializes a GroupBox.
   *
   * \param[in] parent Pointer to QWidget that serves as a parent of the
   *  GroupBox.
   */
  explicit GroupBox(QWidget* parent = nullptr);

  /** \brief Updates the view of the GroupBox so that it displays the current
   *  values of the parameters.
   */
  void update() override;

  /** \brief Sets the parameter-manager of the ShapeDetector.
   *
   * The ObservedParameterManager manages the parameters of a ShapeDetector
   * object.
   * The manager represents the Model of the MVC-pattern, while the GroupBox
   * represents the View and the Controller.
   */
  void setParameterManager(ObservedParameterManager*);

private:
  QVBoxLayout* createMainLayout() const;

  // Data members.
  NormalEstimationGroupBox* normal_estimation_groupbox_;
  SegmentationGroupBox* segmentation_groupbox_;
};

} // shape_detector

#endif  // SHAPE_DETECTOR_GROUPBOX_H
