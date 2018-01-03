/** \file
 * \brief Definition of the DescriptorMatcherGroupBox
 */

#ifndef FEATURE_DETECTOR_DESCRIPTOR_MATCHER_GROUPBOX_H
#define FEATURE_DETECTOR_DESCRIPTOR_MATCHER_GROUPBOX_H

// std.
#include <map>

// Qt.
#include <QGroupBox>
#include <QString>

// Headers of this package.
#include "observed_parameter_manager.h"

// Forward declarations.
class QWidget;
class QLabel;
class QComboBox;
class QCheckBox;
class QGridLayout;


namespace feature_detector {

/** \brief QGroupBox for accessing the parameters of a FeatureDetector that are
 *  related to the descriptor-matching process.
 */
class DescriptorMatcherGroupBox : public QGroupBox {
  Q_OBJECT

public:
  /** \brief Creates and initializes the GroupBox.
   *
   * \param[in] parent Pointer to QWidget that serves as a parent of the
   *  GroupBox.
   */
  explicit DescriptorMatcherGroupBox(QWidget* parent = nullptr);

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
  void requestNormTypeChange(const QString&) const;
  void requestCrossCheckChange(bool) const;

private:
  // Initialization.
  void initNormTypeWidgets();
  void initCrossCheckCheckBox();

  QGridLayout* createMainLayout() const;

  // Static consts.
  static const std::map<QString, ObservedParameterManager::NormType> norm_type_mapping;

  // Data members.
  ObservedParameterManager* param_manager_ {nullptr};

  QLabel* norm_type_label_;
  QComboBox* norm_type_combo_;
  QCheckBox* cross_check_checkbox_;
};

} // feature_detector

#endif  // FEATURE_DETECTOR_DESCRIPTOR_MATCHER_GROUPBOX_H
