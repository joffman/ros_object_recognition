/** \file
 * \brief Implementation of the NormalEstimationGroupBox class.
 */

// Qt.
#include <QObject>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLabel>
#include <QSpinBox>

// Headers of this package.
#include <shape_detector/observed_parameter_manager.h>
#include <shape_detector/normal_estimation_groupbox.h>


namespace shape_detector {

NormalEstimationGroupBox::NormalEstimationGroupBox(QWidget* parent)
    : QGroupBox {"&Normal Estimation", parent},
    param_manager_ {nullptr}
{
  createNearestNeighborsLabel();
  createNearestNeighborsSpinBox();
  QHBoxLayout* layout {createMainLayout()};
  setLayout(layout);
}


void NormalEstimationGroupBox::createNearestNeighborsLabel()
{
  nearest_neighbors_label_ = new QLabel {"Nearest Neighbors:"};
}


void NormalEstimationGroupBox::createNearestNeighborsSpinBox()
{
  nearest_neighbors_spinbox_ = new QSpinBox;

  nearest_neighbors_spinbox_->setRange(
      ObservedParameterManager::num_nearest_neighbors_lower_limit,
      ObservedParameterManager::num_nearest_neighbors_upper_limit
      );

  connect(
      nearest_neighbors_spinbox_,
      static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
      this, &NormalEstimationGroupBox::requestNumNearestNeighborsChange
      );
}


QHBoxLayout* NormalEstimationGroupBox::createMainLayout() const
{
  QHBoxLayout* layout {new QHBoxLayout};

  layout->addWidget(nearest_neighbors_label_);
  layout->addWidget(nearest_neighbors_spinbox_);

  return layout;
}


void NormalEstimationGroupBox::update() const
{
  if (param_manager_)
    nearest_neighbors_spinbox_->setValue(
        param_manager_->numNearestNeighbors()
        );
}


void NormalEstimationGroupBox::setParameterManager(ObservedParameterManager* pm)
{
  param_manager_ = pm;
  update();
}


void NormalEstimationGroupBox::requestNumNearestNeighborsChange(int val) const
{
  if (param_manager_)
    param_manager_->setNumNearestNeighbors(val);
}

} // shape_detector
