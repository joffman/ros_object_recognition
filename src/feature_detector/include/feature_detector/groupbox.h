/** \file
 * \brief Definition of the GroupBox class.
 */

#ifndef FEATURE_DETECTOR_GROUPBOX_H
#define FEATURE_DETECTOR_GROUPBOX_H

// TODO: Add widget for object-name.

// Qt.
#include <QGroupBox>

// Headers of this project.
#include <object_detection/observer.h>

// Forward declarations.
class QWidget;
class QVBoxLayout;

namespace feature_detector {

class ObservedParameterManager;
class SurfGroupBox;
class DescriptorMatcherGroupBox;
class ThresholdsGroupBox;


/** \brief QGroupBox for accessing the parameters of a FeatureDetector object.
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

   /** \brief Sets the parameter-manager of the FeatureDetector.
   *
   * The ObservedParameterManager manages the parameters of a FeatureDetector
   * object.
   * The manager represents the Model of the MVC-pattern, while the GroupBox
   * represents the View and the Controller.
   */
void setParameterManager(ObservedParameterManager*);

private:
  QVBoxLayout* createMainLayout() const;

  // Data members.
  SurfGroupBox* surf_groupbox_;
  DescriptorMatcherGroupBox* descriptor_matcher_groupbox_;
  ThresholdsGroupBox* thresholds_groupbox_;
};

} // feature_detector

#endif  // FEATURE_DETECTOR_GROUPBOX_H
