/** \file
 * \brief Definition of the SurfGroupBox class.
 */

#ifndef FEATURE_DETECTOR_SURF_GROUPBOX_H
#define FEATURE_DETECTOR_SURF_GROUPBOX_H

// Qt.
#include <QGroupBox>

// Forward declarations.
class QWidget;
class QLabel;
class QSpinBox;
class QDoubleSpinBox;
class QCheckBox;
class QGridLayout;


namespace feature_detector {

class ObservedParameterManager;


/** \brief QGroupBox for accessing the parameters of a FeatureDetector that are
 *  related to the computation of SURF features.
 */
class SurfGroupBox : public QGroupBox {
  Q_OBJECT

public:
  /** \brief Creates and initializes the GroupBox.
   *
   * \param[in] parent Pointer to QWidget that serves as a parent of the
   *  GroupBox.
   */
  explicit SurfGroupBox(QWidget* parent = nullptr);

  /** \brief Updates the view of the GroupBox so that it displays the current
   *  values of the parameters.
   */
  void update() const;

  /** \brief Sets the parameter-manager of the FeatureDetector.
   *
   * The ObservedParameterManager manages the parameters of a FeatureDetector
   * object.
   * The manager represents the Model of the MVC-pattern, while the GroupBox
   * represents the View and the Controller.
   */
  void setParameterManager(ObservedParameterManager*);

private slots:
  void requestHessianThresholdChange(double) const;
  void requestNumOctavesChange(int) const;
  void requestNumOctaveLayersChange(int) const;
  void requestExtendedChange(bool) const;
  void requestUprightChange(bool) const;

private:
  // Initialization.
  void initHessianThresholdWidgets();
  void initNumOctavesWidgets();
  void initNumOctaveLayersWidgets();
  void initExtendedCheckBox();
  void initUprightCheckBox();

  QGridLayout* createMainLayout() const;

  // Data members.
  ObservedParameterManager* param_manager_ {nullptr};

  QLabel* hessian_thresh_label_;
  QDoubleSpinBox* hessian_thresh_spinbox_;
  QLabel* num_octaves_label_;
  QSpinBox* num_octaves_spinbox_;
  QLabel* num_octave_layers_label_;
  QSpinBox* num_octave_layers_spinbox_;
  QCheckBox* extended_checkbox_;
  QCheckBox* upright_checkbox_;
};

} // feature_detector

#endif  // FEATURE_DETECTOR_SURF_GROUPBOX_H
