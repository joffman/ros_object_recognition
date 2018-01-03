/** \file
 * \brief Definition of the GroupBox class.
 */

#ifndef MORPHOLOGY_FILTER_GROUPBOX_H
#define MORPHOLOGY_FILTER_GROUPBOX_H

// std.
#include <map>

// Qt.
#include <QGroupBox>
#include <QString>

// Headers of this project.
#include <object_detection/observer.h>
#include "observed_parameter_manager.h"

// Forward declarations.
class QWidget;
class QGridLayout;
class QLabel;
class QComboBox;
class QSpinBox;


namespace morphology_filter {

/** \brief QGroupBox for accessing the parameters of a MorphologyFilter object.
 */
class GroupBox : public QGroupBox, public object_detection::Observer {
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

    /** \brief Sets the parameter-manager of the MorphologyFilter.
   *
   * The ObservedParameterManager manages the parameters of a MorphologyFilter
   * object.
   * The manager represents the Model of the MVC-pattern, while the GroupBox
   * represents the View and the Controller.
   */
  void setParameterManager(ObservedParameterManager* const);

private slots:
  void requestOperationChange(const QString&) const;
  void requestNumIterationsChange(const int) const;

private:
  // Initialization functions.
  void initOperationWidgets();
  void initNumIterationsWidgets();

  QGridLayout* createMainLayout() const;

  // Static consts.
  static const std::map<QString, ObservedParameterManager::Operation> operation_mapping;

  // Data members and widgets.
  ObservedParameterManager* param_manager_ {nullptr};

  QLabel* op_label_;
  QComboBox* op_combo_;

  QLabel* iter_label_;
  QSpinBox* iter_spinbox_;
};

} // morphology_filter

#endif  // MORPHOLOGY_FILTER_GROUPBOX_H
