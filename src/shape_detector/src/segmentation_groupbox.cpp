/** \file
 * \brief Implementation of the SegmentationGroupBox class.
 */

// std.
#include <algorithm>  // find_if()
#include <utility>    // pair
#include <cassert>

// Qt.
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QLabel>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QCheckBox>
#include <QString>

// Headers of this package.
#include <shape_detector/observed_parameter_manager.h>
#include <shape_detector/segmentation_groupbox.h>


namespace shape_detector {

const std::map<QString, Shape>
    SegmentationGroupBox::shape_mapping {
      {"Sphere", Shape::sphere},
      {"Cylinder", Shape::cylinder}
    };


////////////////////////////////////////////////////////////////////////////////
//
// Construction & Initialization.
//

SegmentationGroupBox::SegmentationGroupBox(QWidget* parent)
    : QGroupBox{"&SAC Segmentation", parent}
{
  createShapeWidgets();
  createRadiusWidgets();
  createNormalDistanceWeightWidgets();
  createDistanceThresholdWidgets();
  createMaxIterationsWidgets();
  createOptimizeCoefficientsCheckBox();

  QGridLayout* main_layout {createMainLayout()};

  setLayout(main_layout);
}


void SegmentationGroupBox::createShapeWidgets()
{
  shape_label_ = new QLabel {"Shape:"};

  shape_combo_ = new QComboBox;
  for (const auto& pair : shape_mapping)
    shape_combo_->addItem(pair.first);

  shape_label_->setBuddy(shape_combo_);

  connect(shape_combo_, &QComboBox::currentTextChanged,
      this, &SegmentationGroupBox::requestShapeChange);
}


void SegmentationGroupBox::createRadiusWidgets()
{
  createRadiusLabels();
  createRadiusSpinBoxes();
}


void SegmentationGroupBox::createRadiusLabels()
{
  min_radius_label_ = new QLabel{"Min-Radius:"};
  max_radius_label_ = new QLabel{"Max-Radius:"};
}


void SegmentationGroupBox::createRadiusSpinBoxes()
{
  min_radius_spinbox_ = new QDoubleSpinBox;
  max_radius_spinbox_ = new QDoubleSpinBox;

  min_radius_spinbox_->setSingleStep(0.05);
  max_radius_spinbox_->setSingleStep(0.05);

  min_radius_spinbox_->setRange(
      ObservedParameterManager::radius_lower_limit,
      ObservedParameterManager::radius_upper_limit
      );
  max_radius_spinbox_->setRange(
      ObservedParameterManager::radius_lower_limit,
      ObservedParameterManager::radius_upper_limit
      );

  min_radius_spinbox_->setSuffix(" m");
  max_radius_spinbox_->setSuffix(" m");

  connect(
      min_radius_spinbox_,
      static_cast<void (QDoubleSpinBox::*)(double)>(
          &QDoubleSpinBox::valueChanged),
      this,
      &SegmentationGroupBox::requestMinRadiusChange
      );
  connect(
      max_radius_spinbox_,
      static_cast<void (QDoubleSpinBox::*)(double)>(
          &QDoubleSpinBox::valueChanged),
      this,
      &SegmentationGroupBox::requestMaxRadiusChange
      );
}


void SegmentationGroupBox::createNormalDistanceWeightWidgets()
{
  normal_distance_weight_label_ = new QLabel {"Normal Distance Weight:"};

  normal_distance_weight_spinbox_ = new QDoubleSpinBox;
  normal_distance_weight_spinbox_->setRange(
      ObservedParameterManager::normal_distance_weight_lower_limit,
      ObservedParameterManager::normal_distance_weight_upper_limit
      );
  normal_distance_weight_spinbox_->setSingleStep(0.05);  // TODO: ??

  connect(
      normal_distance_weight_spinbox_,
      static_cast<void (QDoubleSpinBox::*)(double)>(
          &QDoubleSpinBox::valueChanged),
      this,
      &SegmentationGroupBox::requestNormalDistanceWeightChange
      );
}


void SegmentationGroupBox::createDistanceThresholdWidgets()
{
  distance_threshold_label_ = new QLabel {"Distance Threshold:"};

  distance_threshold_spinbox_ = new QDoubleSpinBox;
  distance_threshold_spinbox_->setRange(
      ObservedParameterManager::distance_threshold_lower_limit,
      ObservedParameterManager::distance_threshold_upper_limit
      );
  distance_threshold_spinbox_->setSingleStep(0.01);  // TODO: ??

  connect(
      distance_threshold_spinbox_,
      static_cast<void (QDoubleSpinBox::*)(double)>(
          &QDoubleSpinBox::valueChanged),
      this,
      &SegmentationGroupBox::requestDistanceThresholdChange
      );
}


void SegmentationGroupBox::createMaxIterationsWidgets()
{
  max_iterations_label_ = new QLabel {"Max Iterations:"};

  max_iterations_spinbox_ = new QSpinBox;
  max_iterations_spinbox_->setRange(
      ObservedParameterManager::max_iterations_lower_limit,
      ObservedParameterManager::max_iterations_upper_limit
      );

  connect(
      max_iterations_spinbox_,
      static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
      this,
      &SegmentationGroupBox::requestMaxIterationsChange
      );
}


void SegmentationGroupBox::createOptimizeCoefficientsCheckBox()
{
  optimize_coefficients_checkbox_ = new QCheckBox {"Optimize Coefficients"};

  connect(
      optimize_coefficients_checkbox_,
      &QCheckBox::stateChanged,
      this,
      &SegmentationGroupBox::requestOptimizeCoefficientsChange
      );
}


QGridLayout* SegmentationGroupBox::createMainLayout() const
{
  QGridLayout* grid_layout {new QGridLayout};

  grid_layout->addWidget(shape_label_, 0, 0);
  grid_layout->addWidget(shape_combo_, 0, 1);

  grid_layout->addWidget(min_radius_label_, 1, 0);
  grid_layout->addWidget(min_radius_spinbox_, 1, 1);
  grid_layout->addWidget(max_radius_label_, 2, 0);
  grid_layout->addWidget(max_radius_spinbox_, 2, 1);

  grid_layout->addWidget(normal_distance_weight_label_, 3, 0);
  grid_layout->addWidget(normal_distance_weight_spinbox_, 3, 1);

  grid_layout->addWidget(distance_threshold_label_, 4, 0);
  grid_layout->addWidget(distance_threshold_spinbox_, 4, 1);

  grid_layout->addWidget(max_iterations_label_, 5, 0);
  grid_layout->addWidget(max_iterations_spinbox_, 5, 1);

  grid_layout->addWidget(
      optimize_coefficients_checkbox_, 6, 0,
      1 /* delta-y */, 2 /* delta-x */
      );

  return grid_layout;
}


////////////////////////////////////////////////////////////////////////////////
//
// Observation-related methods.
//

void SegmentationGroupBox::update() const
{
  if (!param_manager_)
    return;

  Shape current_shape = param_manager_->shape();
  auto mapping_it = std::find_if(
      shape_mapping.begin(), shape_mapping.end(),
      [current_shape](const std::pair<decltype(shape_mapping)::key_type,
                      decltype(shape_mapping)::mapped_type>& p)
      { return p.second == current_shape; });
  assert(mapping_it != shape_mapping.end());
  shape_combo_->setCurrentText(mapping_it->first);

  min_radius_spinbox_->setValue(param_manager_->minRadius());
  max_radius_spinbox_->setValue(param_manager_->maxRadius());

  normal_distance_weight_spinbox_->setValue(
      param_manager_->normalDistanceWeight());

  distance_threshold_spinbox_->setValue(param_manager_->distanceThreshold());

  max_iterations_spinbox_->setValue(param_manager_->maxIterations());

  optimize_coefficients_checkbox_->setChecked(
      param_manager_->optimizeCoefficients());
}


void SegmentationGroupBox::setParameterManager(ObservedParameterManager* pm)
{
  param_manager_ = pm;
  update();
}


////////////////////////////////////////////////////////////////////////////////
//
// Slots.
//

void SegmentationGroupBox::requestShapeChange(const QString& text) const
{
  if (param_manager_)
    param_manager_->setShape(shape_mapping.at(text));
}


void SegmentationGroupBox::requestMinRadiusChange(double val) const
{
  if (param_manager_)
    param_manager_->setMinRadius(val);
}


void SegmentationGroupBox::requestMaxRadiusChange(double val) const
{
  if (param_manager_)
    param_manager_->setMaxRadius(val);
}


void SegmentationGroupBox::requestNormalDistanceWeightChange(double val) const
{
  if (param_manager_)
    param_manager_->setNormalDistanceWeight(val);
}


void SegmentationGroupBox::requestDistanceThresholdChange(double val) const
{
  if (param_manager_)
    param_manager_->setDistanceThreshold(val);
}


void SegmentationGroupBox::requestMaxIterationsChange(int val) const
{
  if (param_manager_)
    param_manager_->setMaxIterations(val);
}


void
SegmentationGroupBox::requestOptimizeCoefficientsChange(int check_state) const
{
  if (param_manager_)
    param_manager_->setOptimizeCoefficients(check_state == Qt::Checked);
}

} // shape_detector
