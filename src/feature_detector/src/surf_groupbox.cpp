/** \file
 * \brief Implementation of the SurfGroupBox class.
 */

// Qt.
#include <QObject>
#include <QWidget>
#include <QGroupBox>
#include <QGridLayout>
#include <QLabel>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>

// Headers of this package.
#include <feature_detector/surf_groupbox.h>
#include <feature_detector/observed_parameter_manager.h>


namespace feature_detector {

////////////////////////////////////////////////////////////////////////////////
//
// Construction & initialization.
//

SurfGroupBox::SurfGroupBox(QWidget* parent)
    : QGroupBox {"&SURF", parent}
{
  initHessianThresholdWidgets();
  initNumOctavesWidgets();
  initNumOctaveLayersWidgets();
  initExtendedCheckBox();
  initUprightCheckBox();

  QGridLayout* main_layout {createMainLayout()};
  setLayout(main_layout);
}


void SurfGroupBox::initHessianThresholdWidgets()
{
  hessian_thresh_label_ = new QLabel {"hessian threshold:"};

  hessian_thresh_spinbox_ = new QDoubleSpinBox;
  hessian_thresh_spinbox_->setSingleStep(0.5);
  hessian_thresh_spinbox_->setRange(
      ObservedParameterManager::min_hessian_thresh, // or 'ParameterManager::...'?
      ObservedParameterManager::max_hessian_thresh);
  connect(hessian_thresh_spinbox_, static_cast<void(QDoubleSpinBox::*)(double)>(
          &QDoubleSpinBox::valueChanged),
      this, &SurfGroupBox::requestHessianThresholdChange);
}


void SurfGroupBox::initNumOctavesWidgets()
{
  num_octaves_label_ = new QLabel {"number octaves:"};

  num_octaves_spinbox_ = new QSpinBox;
  num_octaves_spinbox_->setRange(
      ObservedParameterManager::min_num_octaves,
      ObservedParameterManager::max_num_octaves);
  connect(num_octaves_spinbox_, static_cast<void (QSpinBox::*)(int)>(
          &QSpinBox::valueChanged),
      this, &SurfGroupBox::requestNumOctavesChange);
}


void SurfGroupBox::initNumOctaveLayersWidgets()
{
  num_octave_layers_label_ = new QLabel {"number octaves:"};

  num_octave_layers_spinbox_ = new QSpinBox;
  num_octave_layers_spinbox_->setRange(
      ObservedParameterManager::min_num_octave_layers,
      ObservedParameterManager::max_num_octave_layers);
  connect(num_octave_layers_spinbox_, static_cast<void (QSpinBox::*)(int)>(
          &QSpinBox::valueChanged),
      this, &SurfGroupBox::requestNumOctaveLayersChange);
}


void SurfGroupBox::initExtendedCheckBox()
{
  extended_checkbox_ = new QCheckBox {"extended"};
  connect(extended_checkbox_, &QCheckBox::stateChanged,
          this, &SurfGroupBox::requestExtendedChange);
}


void SurfGroupBox::initUprightCheckBox()
{
  upright_checkbox_ = new QCheckBox {"upright"};
  connect(upright_checkbox_, &QCheckBox::stateChanged,
          this, &SurfGroupBox::requestUprightChange);
}


QGridLayout* SurfGroupBox::createMainLayout() const
{
  QGridLayout* layout {new QGridLayout};

  layout->addWidget(hessian_thresh_label_, 0, 0);
  layout->addWidget(hessian_thresh_spinbox_, 0, 1);

  layout->addWidget(num_octaves_label_, 1, 0);
  layout->addWidget(num_octaves_spinbox_, 1, 1);

  layout->addWidget(num_octave_layers_label_, 2, 0);
  layout->addWidget(num_octave_layers_spinbox_, 2, 1);

  layout->addWidget(extended_checkbox_, 3, 0, 1, 2);
  layout->addWidget(upright_checkbox_, 4, 0, 1, 2);

  return layout;
}


////////////////////////////////////////////////////////////////////////////////
//
// Observation-related methods.
//

void SurfGroupBox::update() const
{
  if (param_manager_) {
    hessian_thresh_spinbox_->setValue(param_manager_->hessianThreshold());
    num_octaves_spinbox_->setValue(param_manager_->numOctaves());
    num_octave_layers_spinbox_->setValue(param_manager_->numOctaveLayers());
    extended_checkbox_->setChecked(param_manager_->extended());
    upright_checkbox_->setChecked(param_manager_->upright());
  }
}


void SurfGroupBox::setParameterManager(ObservedParameterManager* pm)
{
  param_manager_ = pm;
  update();
}


////////////////////////////////////////////////////////////////////////////////
//
// Slots.
//

void SurfGroupBox::requestHessianThresholdChange(double val) const
{
  if (param_manager_)
    param_manager_->setHessianThreshold(val);
}


void SurfGroupBox::requestNumOctavesChange(int val) const
{
  if (param_manager_)
    param_manager_->setNumOctaves(val);
}


void SurfGroupBox::requestNumOctaveLayersChange(int val) const
{
  if (param_manager_)
    param_manager_->setNumOctaveLayers(val);
}


void SurfGroupBox::requestExtendedChange(bool val) const
{
  if (param_manager_)
    param_manager_->setExtended(val);
}


void SurfGroupBox::requestUprightChange(bool val) const
{
  if (param_manager_)
    param_manager_->setUpright(val);
}

} // feature_detector
