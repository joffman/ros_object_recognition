/** \file
 * \brief Implementation of the GroupBox class.
 */

// Qt.
#include <QObject>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QDoubleSpinBox>

// Our headers.
#include <distance_filter/groupbox.h>
#include <distance_filter/observed_parameter_manager.h>

using namespace std;


namespace distance_filter {

GroupBox::GroupBox(QWidget* parent)
    : QGroupBox {"&Distance Filter", parent}
{
  createDistanceLabels();
  createDistanceSpinBoxes();
  QGridLayout* grid_layout {createMainLayout()};
  setLayout(grid_layout);
}
   
   
void GroupBox::createDistanceLabels()
{
  min_distance_label_ = new QLabel {"Min-Distance"};
  max_distance_label_ = new QLabel {"Max-Distance"};
}


void GroupBox::createDistanceSpinBoxes()
{
  min_distance_spinbox_ = new QDoubleSpinBox;
  max_distance_spinbox_ = new QDoubleSpinBox;

  min_distance_spinbox_->setSingleStep(0.1);
  max_distance_spinbox_->setSingleStep(0.1);

  min_distance_spinbox_->setRange(
      ObservedParameterManager::distance_lower_limit,
      ObservedParameterManager::distance_upper_limit
      );
  max_distance_spinbox_->setRange(
      ObservedParameterManager::distance_lower_limit,
      ObservedParameterManager::distance_upper_limit
      );

  min_distance_spinbox_->setSuffix(" m");
  max_distance_spinbox_->setSuffix(" m");

  connect(
      min_distance_spinbox_,
      static_cast<void (QDoubleSpinBox::*)(double)>(
          &QDoubleSpinBox::valueChanged),
      this, &GroupBox::requestMinDistanceChange
      );
  connect(
      max_distance_spinbox_,
      static_cast<void (QDoubleSpinBox::*)(double)>(
          &QDoubleSpinBox::valueChanged),
      this, &GroupBox::requestMaxDistanceChange
      );
}
  

QGridLayout* GroupBox::createMainLayout()
{
  QGridLayout* grid_layout {new QGridLayout};

  grid_layout->addWidget(min_distance_label_, 0, 0);
  grid_layout->addWidget(max_distance_label_, 0, 1);
  grid_layout->addWidget(min_distance_spinbox_, 1, 0);
  grid_layout->addWidget(max_distance_spinbox_, 1, 1);

  return grid_layout;
}


void GroupBox::update()
{
  if (param_manager_) {
    min_distance_spinbox_->setValue(param_manager_->minDistance());
    max_distance_spinbox_->setValue(param_manager_->maxDistance());
  }
}


void GroupBox::setParameterManager(ObservedParameterManager* const pm)
    // or drop the 'const'?
{
  param_manager_ = pm;
  update();
}


void GroupBox::requestMinDistanceChange(const double value) const
{
  if (param_manager_)
    param_manager_->setMinDistance(value);
}


void GroupBox::requestMaxDistanceChange(const double value) const
{
  if (param_manager_)
    param_manager_->setMaxDistance(value);
}

} // distance_filter
