/** \file
 * \brief Implementation of the DescriptorMatcherGroupBox
 */

// std.
#include <algorithm>  // find_if()
#include <utility>    // pair
#include <cassert>

// Qt.
#include <QObject>
#include <QWidget>
#include <QGroupBox>
#include <QGridLayout>
#include <QLabel>
#include <QComboBox>
#include <QCheckBox>

// Headers of this package.
#include <feature_detector/observed_parameter_manager.h>
#include <feature_detector/descriptor_matcher_groupbox.h>

using NormType = feature_detector::ObservedParameterManager::NormType;
using namespace std;


namespace feature_detector {

////////////////////////////////////////////////////////////////////////////////
//
// Definition of static consts.
//

const map<QString, NormType> DescriptorMatcherGroupBox::norm_type_mapping
{
  {"L1", NormType::l1},
  {"L2", NormType::l2}
};


////////////////////////////////////////////////////////////////////////////////
//
// Construction & initialization.
//

DescriptorMatcherGroupBox::DescriptorMatcherGroupBox(QWidget* parent)
    : QGroupBox {"&Descriptor Matcher", parent},
    param_manager_ {nullptr}
{
  initNormTypeWidgets();
  initCrossCheckCheckBox();

  QGridLayout* main_layout {createMainLayout()};
  setLayout(main_layout);
}


void DescriptorMatcherGroupBox::initNormTypeWidgets()
{
  norm_type_label_ = new QLabel {"norm types:"};

  norm_type_combo_ = new QComboBox;
  for (auto& pair : norm_type_mapping)
    norm_type_combo_->addItem(pair.first);
  connect(norm_type_combo_, &QComboBox::currentTextChanged,
          this, &DescriptorMatcherGroupBox::requestNormTypeChange);
}


void DescriptorMatcherGroupBox::initCrossCheckCheckBox()
{
  cross_check_checkbox_ = new QCheckBox {"cross-check"};
  connect(cross_check_checkbox_, &QCheckBox::stateChanged,
          this, &DescriptorMatcherGroupBox::requestCrossCheckChange);
}


QGridLayout* DescriptorMatcherGroupBox::createMainLayout() const
{
  QGridLayout* layout {new QGridLayout};

  layout->addWidget(norm_type_label_, 0, 0);
  layout->addWidget(norm_type_combo_, 0, 1);

  layout->addWidget(cross_check_checkbox_, 1, 0, 1, 2);

  return layout;
}


////////////////////////////////////////////////////////////////////////////////
//
// Observation-related methods.
//

void DescriptorMatcherGroupBox::update() const
{
  if (param_manager_) {
    NormType type = param_manager_->normType();
    auto it = find_if(norm_type_mapping.begin(), norm_type_mapping.end(),
                      [type](const pair<decltype(norm_type_mapping)::key_type,
                             decltype(norm_type_mapping)::mapped_type>& p)
                      { return p.second == type; });
    assert(it != norm_type_mapping.end());
    norm_type_combo_->setCurrentText(it->first);

    cross_check_checkbox_->setChecked(
        param_manager_->crossCheck());
  }
}


void DescriptorMatcherGroupBox::setParameterManager(ObservedParameterManager* pm)
{
  param_manager_ = pm;
  update();
}


////////////////////////////////////////////////////////////////////////////////
//
// Slots.
//

void DescriptorMatcherGroupBox::requestNormTypeChange(const QString& text) const
{
  if (param_manager_)
    param_manager_->setNormType(norm_type_mapping.at(text));
}


void DescriptorMatcherGroupBox::requestCrossCheckChange(bool val) const
{
  if (param_manager_)
    param_manager_->setCrossCheck(val);
}

} // feature_detector
