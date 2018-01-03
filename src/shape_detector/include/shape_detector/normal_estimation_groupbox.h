/** \file
 * \brief Definition of the NormalEstimationGroupBox class.
 */

#ifndef SHAPE_DETECTOR_NORMAL_ESTIMATION_GROUPBOX_H
#define SHAPE_DETECTOR_NORMAL_ESTIMATION_GROUPBOX_H

// Qt.
#include <QGroupBox>

class QWidget;
class QLabel;
class QSpinBox;
class QHBoxLayout;


namespace shape_detector {

class ObservedParameterManager;


/** \brief QGroupBox for accessing the parameters of a ShapeDetector that are
 *  related to the estimation of normals.
 */
class NormalEstimationGroupBox : public QGroupBox {
  Q_OBJECT

public:
  /** \brief Creates and initializes the GroupBox.
   *
   * \param[in] parent Pointer to QWidget that serves as a parent of the
   *  GroupBox.
   */
  explicit NormalEstimationGroupBox(QWidget* parent = nullptr);
  
  /** \brief Updates the view of the GroupBox so that it displays the current
   *  values of the parameters.
   */
  void update() const;

  /** \brief Sets the parameter-manager of the ShapeDetector.
   *
   * The ObservedParameterManager manages the parameters of a ShapeDetector
   * object.
   * The manager represents the Model of the MVC-pattern, while the GroupBox
   * represents the View and the Controller.
   */
void setParameterManager(ObservedParameterManager* detector);

private slots:
  void requestNumNearestNeighborsChange(int val) const;

private:
  void createNearestNeighborsLabel();
  void createNearestNeighborsSpinBox();
  QHBoxLayout* createMainLayout() const;

  // Data members.
  ObservedParameterManager* param_manager_ {nullptr};

  QLabel* nearest_neighbors_label_;
  QSpinBox* nearest_neighbors_spinbox_;
};

} // shape_detector

#endif  // SHAPE_DETECTOR_NORMAL_ESTIMATION_GROUPBOX_H
