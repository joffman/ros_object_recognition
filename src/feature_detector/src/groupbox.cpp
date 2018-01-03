/** \file
 * \brief Implementation of the GroupBox class.
 */

// Qt.
#include <QWidget>
#include <QGroupBox>
#include <QVBoxLayout>

// Headers of this package.
#include <feature_detector/observed_parameter_manager.h>
#include <feature_detector/groupbox.h>
#include <feature_detector/surf_groupbox.h>
#include <feature_detector/descriptor_matcher_groupbox.h>
#include <feature_detector/thresholds_groupbox.h>


namespace feature_detector {

GroupBox::GroupBox(QWidget* parent)
    : QGroupBox {"Feature Detector", parent},
    surf_groupbox_ {new SurfGroupBox{this}},
    descriptor_matcher_groupbox_ { new DescriptorMatcherGroupBox{this}},
    thresholds_groupbox_ {new ThresholdsGroupBox{this}}
{
  QVBoxLayout* main_layout {createMainLayout()};
  setLayout(main_layout);
}


QVBoxLayout* GroupBox::createMainLayout() const
{
  QVBoxLayout* layout {new QVBoxLayout};
  layout->addWidget(surf_groupbox_);
  layout->addWidget(descriptor_matcher_groupbox_);
  layout->addWidget(thresholds_groupbox_);
  return layout;
}


void GroupBox::update()
{
  surf_groupbox_->update();
  descriptor_matcher_groupbox_->update();
  thresholds_groupbox_->update();
}


void GroupBox::setParameterManager(ObservedParameterManager* pm)
{
  surf_groupbox_->setParameterManager(pm);
  descriptor_matcher_groupbox_->setParameterManager(pm);
  thresholds_groupbox_->setParameterManager(pm);
}

} // feature_detector
