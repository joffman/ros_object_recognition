/** \file
 * \brief Definition of the GroupBox class.
 */

#ifndef DISTANCE_FILTER_GROUPBOX_H
#define DISTANCE_FILTER_GROUPBOX_H

// Qt.
#include <QWidget>
#include <QGridLayout>
#include <QGroupBox>

// Our headers.
#include <object_detection/observer.h>

// Forward declarations.
class QLabel;
class QDoubleSpinBox;


namespace distance_filter {

class ObservedParameterManager;


/** \brief GroupBox for accessing the parameters of a DistanceFilter.
 */
class GroupBox : public QGroupBox, public object_detection::Observer {
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

  /** \brief Sets the parameter-manager of the DistanceFilter.
   *
   * The ObservedParameterManager manages the parameters of a DistanceFilter
   * object.
   * The manager represents the Model of the MVC-pattern, while the GroupBox
   * represents the View and the Controller.
   */
  void setParameterManager(ObservedParameterManager* filter);

private slots:
  void requestMinDistanceChange(double) const;
  void requestMaxDistanceChange(double) const;

private:
  void createDistanceLabels();  // or 'initDistanceLabels'?!
  void createDistanceSpinBoxes();
  QGridLayout* createMainLayout();

  // Data members.
  ObservedParameterManager* param_manager_ {nullptr};

  QLabel* min_distance_label_;
  QLabel* max_distance_label_;
  QDoubleSpinBox* min_distance_spinbox_;
  QDoubleSpinBox* max_distance_spinbox_;
};

} // distance_filter

#endif  // DISTANCE_FILTER_GROUPBOX_H
