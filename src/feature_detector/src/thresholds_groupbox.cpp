/** \file
 * \brief Implementation of the ThresholdsGroupBox class.
 */

// Qt.
#include <QObject>
#include <QWidget>
#include <QGroupBox>
#include <QGridLayout>
#include <QLabel>
#include <QDoubleSpinBox>

// Headers of this package.
#include <feature_detector/observed_parameter_manager.h>
#include <feature_detector/thresholds_groupbox.h>


namespace feature_detector {

////////////////////////////////////////////////////////////////////////////////
//
// Construction & initialization.
//

ThresholdsGroupBox::ThresholdsGroupBox(QWidget* parent)
    : QGroupBox {"&Thresholds", parent},
    param_manager_ {nullptr}
{
  initMatchDistanceWidgets();
  initRansacReprojWidgets();

  QGridLayout* main_layout {createMainLayout()};
  setLayout(main_layout);
}


void ThresholdsGroupBox::initMatchDistanceWidgets()
{
  match_distance_label_ = new QLabel {"match-distance threshold:"};

  match_distance_spinbox_ = new QDoubleSpinBox;
  match_distance_spinbox_->setSingleStep(0.25);
  match_distance_spinbox_->setRange(
      ObservedParameterManager::min_match_distance_thresh,
      ObservedParameterManager::max_match_distance_thresh);
  connect(match_distance_spinbox_, static_cast<void(QDoubleSpinBox::*)(double)>(
          &QDoubleSpinBox::valueChanged),
      this, &ThresholdsGroupBox::requestMatchDistanceChange);
}


void ThresholdsGroupBox::initRansacReprojWidgets()
{
  ransac_reproj_label_ = new QLabel {"ransac-reprojection threshold:"};

  ransac_reproj_spinbox_ = new QDoubleSpinBox;
  ransac_reproj_spinbox_->setSingleStep(0.5);
  ransac_reproj_spinbox_->setRange(
      ObservedParameterManager::min_ransac_reproj_thresh,
      ObservedParameterManager::max_ransac_reproj_thresh);
  connect(ransac_reproj_spinbox_, static_cast<void(QDoubleSpinBox::*)(double)>(
          &QDoubleSpinBox::valueChanged),
      this, &ThresholdsGroupBox::requestRansacReprojChange);
}


QGridLayout* ThresholdsGroupBox::createMainLayout() const
{
  QGridLayout* layout {new QGridLayout};

  layout->addWidget(match_distance_label_, 0, 0);
  layout->addWidget(match_distance_spinbox_, 0, 1);

  layout->addWidget(ransac_reproj_label_, 1, 0);
  layout->addWidget(ransac_reproj_spinbox_, 1, 1);

  return layout;
}


////////////////////////////////////////////////////////////////////////////////
//
// Observation-related methods.
//

void ThresholdsGroupBox::update() const
{
  if (param_manager_) {
    match_distance_spinbox_->setValue(
        param_manager_->matchDistanceThreshold());

    ransac_reproj_spinbox_->setValue(
        param_manager_->ransacReprojThreshold());
  }
}


void ThresholdsGroupBox::setParameterManager(ObservedParameterManager* pm)
{
  param_manager_ = pm;
  update();
}


////////////////////////////////////////////////////////////////////////////////
//
// Slots.
//

void ThresholdsGroupBox::requestMatchDistanceChange(double val) const
{
  if (param_manager_)
    param_manager_->setMatchDistanceThreshold(val);
}


void ThresholdsGroupBox::requestRansacReprojChange(double val) const
{
  if (param_manager_)
    param_manager_->setRansacReprojThreshold(val);
}

} // feature_detector
