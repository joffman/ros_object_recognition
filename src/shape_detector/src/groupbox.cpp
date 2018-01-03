/** \file
 * \brief Implementation of the GroupBox class.
 */

// Qt.
#include <QWidget>
#include <QGroupBox>
#include <QVBoxLayout>

// Headers of this package.
#include <shape_detector/observed_parameter_manager.h>
#include <shape_detector/groupbox.h>
#include <shape_detector/normal_estimation_groupbox.h>
#include <shape_detector/segmentation_groupbox.h>


namespace shape_detector {

GroupBox::GroupBox(QWidget* parent)
    : QGroupBox {"Shape Detector", parent},
    normal_estimation_groupbox_ {new NormalEstimationGroupBox {this}},
    segmentation_groupbox_ {new SegmentationGroupBox {this}}
{
  QVBoxLayout* main_layout {createMainLayout()};
  setLayout(main_layout);
}


QVBoxLayout* GroupBox::createMainLayout() const
{
  QVBoxLayout* layout {new QVBoxLayout};
  layout->addWidget(normal_estimation_groupbox_);
  layout->addWidget(segmentation_groupbox_);

  return layout;
}


void GroupBox::update()
{
  normal_estimation_groupbox_->update();
  segmentation_groupbox_->update();
}


void GroupBox::setParameterManager(ObservedParameterManager* pm)
{
  normal_estimation_groupbox_->setParameterManager(pm);
  segmentation_groupbox_->setParameterManager(pm);
}

} // shape_detector
