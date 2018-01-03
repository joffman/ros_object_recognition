/** \file
 * \brief Definition of the GroupBox class.
 */

#ifndef HSV_FILTER_GROUPBOX_H
#define HSV_FILTER_GROUPBOX_H

// Qt.
#include <QGroupBox>

// object_detection_*.
#include <object_detection/observer.h>

// Forward declarations.
class QWidget;
class QVBoxLayout;
class QLabel;
class QSlider;
class QLCDNumber;


namespace hsv_filter {

class ObservedParameterManager;

/** \brief QGroupBox for accessing the parameters of a HSVFilter object.
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

  /** \brief Sets the parameter-manager of the HSVFilter.
   *
   * The ObservedParameterManager manages the parameters of a HSVFilter
   * object.
   * The manager represents the Model of the MVC-pattern, while the GroupBox
   * represents the View and the Controller.
   */
   void setParameterManager(ObservedParameterManager* const);

private slots:
  void requestHMinChange(const int val) const;
  void requestHMaxChange(const int val) const;

  void requestSMinChange(const int val) const;
  void requestSMaxChange(const int val) const;

  void requestVMinChange(const int val) const;
  void requestVMaxChange(const int val) const;

private:
  //
  // Initialization functions.
  //
  void initLabels();

  void initSliders();
  void allocateSliderMemory();
  void setSliderRanges() const;
  void setSliderSteps() const;
  void connectSliderSignals() const;

  void initLCDs();
  void allocateLCDMemory();
  void setLCDDigitCounts() const;
  void connectLCDs() const;

  void initGroupBoxes();
  void initHGroupBox();
  void initSGroupBox();
  void initVGroupBox();

  QVBoxLayout* createMainLayout() const;

  //
  // Data members and widgets.
  //
  ObservedParameterManager* param_manager_ {nullptr};

  // Hue.
  QGroupBox* h_groupbox_;

  QLabel* h_min_label_;
  QLabel* h_max_label_;

  QSlider* h_min_slider_;
  QSlider* h_max_slider_;

  QLCDNumber* h_min_lcd_;
  QLCDNumber* h_max_lcd_;

  // Saturation.
  QGroupBox* s_groupbox_;

  QLabel* s_min_label_;
  QLabel* s_max_label_;

  QSlider* s_min_slider_;
  QSlider* s_max_slider_;

  QLCDNumber* s_min_lcd_;
  QLCDNumber* s_max_lcd_;

  // Value.
  QGroupBox* v_groupbox_;

  QLabel* v_min_label_;
  QLabel* v_max_label_;

  QSlider* v_min_slider_;
  QSlider* v_max_slider_;

  QLCDNumber* v_min_lcd_;
  QLCDNumber* v_max_lcd_;
};

} // hsv_filter

#endif  // HSV_FILTER_GROUPBOX_H
