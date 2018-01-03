/** \file
 * \brief Definition of the SegmentationGroupBox class.
 */

#ifndef SHAPE_DETECTOR_SEGMENTATION_GROUPBOX_H
#define SHAPE_DETECTOR_SEGMENTATION_GROUPBOX_H

// std.
#include <map>

// Qt.
#include <QGroupBox>
#include <QString>

// Headers of this package.
#include "shape_enum.h"

// Forward declarations.
class QWidget;
class QGridLayout;
class QHBoxLayout;
class QLabel;
class QSpinBox;
class QDoubleSpinBox;
class QComboBox;
class QCheckBox;


namespace shape_detector {

class ObservedParameterManager;


/** \brief QGroupBox for accessing the parameters of a ShapeDetector that are
 *  related to the SAC-segmentation.
 */
class SegmentationGroupBox : public QGroupBox {
  Q_OBJECT

public:
  /** \brief Creates and initializes the GroupBox.
   *
   * \param[in] parent Pointer to QWidget that serves as a parent of the
   *  GroupBox.
   */
   explicit SegmentationGroupBox(QWidget* parent = nullptr);

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
void setParameterManager(ObservedParameterManager*);

private slots:
  void requestShapeChange(const QString& text) const;
  void requestMinRadiusChange(double val) const;
  void requestMaxRadiusChange(double val) const;
  void requestNormalDistanceWeightChange(double val) const;
  void requestDistanceThresholdChange(double val) const;
  void requestMaxIterationsChange(int val) const;
  void requestOptimizeCoefficientsChange(int check_state) const;

private:
  void createShapeWidgets();
  void createRadiusWidgets();
  void createRadiusLabels();
  void createRadiusSpinBoxes();
  void createNormalDistanceWeightWidgets();
  void createDistanceThresholdWidgets();
  void createMaxIterationsWidgets();
  void createOptimizeCoefficientsCheckBox();

  QGridLayout* createMainLayout() const;

  // static consts.
  static const std::map<QString, Shape> shape_mapping;

  // Data members.
  ObservedParameterManager* param_manager_ {nullptr};

  QLabel* shape_label_;
  QComboBox* shape_combo_;
  QLabel* min_radius_label_;
  QDoubleSpinBox* min_radius_spinbox_;
  QLabel* max_radius_label_;
  QDoubleSpinBox* max_radius_spinbox_;
  QLabel* normal_distance_weight_label_;
  QDoubleSpinBox* normal_distance_weight_spinbox_;
  QLabel* distance_threshold_label_;
  QDoubleSpinBox* distance_threshold_spinbox_;
  QLabel* max_iterations_label_;
  QSpinBox* max_iterations_spinbox_;
  QCheckBox* optimize_coefficients_checkbox_;
};

} // shape_detector

#endif  // SHAPE_DETECTOR_SEGMENTATION_GROUPBOX_H
