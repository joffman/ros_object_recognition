/** \file
 * \brief Definition of the ThresholdsGroupBox class.
 */

#ifndef FEATURE_DETECTOR_THRESHOLDS_GROUPBOX_H
#define FEATURE_DETECTOR_THRESHOLDS_GROUPBOX_H

// Qt.
#include <QGroupBox>

// Forward declarations.
class QWidget;
class QLabel;
class QDoubleSpinBox;
class QGridLayout;


namespace feature_detector {

class ObservedParameterManager;


/** \brief QGroupBox for accessing the parameters of a FeatureDetector that are
 *  thresholds that are unrelated to the feature-detector or the matcher.
 */
class ThresholdsGroupBox : public QGroupBox {
  Q_OBJECT

public:
  /** \brief Creates and initializes the GroupBox.
   *
   * \param[in] parent Pointer to QWidget that serves as a parent of the
   *  GroupBox.
   */
  explicit ThresholdsGroupBox(QWidget* parent = nullptr);

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
  // void requestKptResponseChange(double) const;
  void requestMatchDistanceChange(double) const;
  void requestRansacReprojChange(double) const;

private:
  // Initialization.
  void initMatchDistanceWidgets();
  void initRansacReprojWidgets();

  QGridLayout* createMainLayout() const;

  // Data members.
  ObservedParameterManager* param_manager_ {nullptr};

  QLabel* match_distance_label_;
  QDoubleSpinBox* match_distance_spinbox_;

  QLabel* ransac_reproj_label_;
  QDoubleSpinBox* ransac_reproj_spinbox_;
};

} // feature_detector

#endif  // FEATURE_DETECTOR_THRESHOLDS_GROUPBOX_H
