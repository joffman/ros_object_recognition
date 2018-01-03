/** \file
 * \brief Implementation of the GroupBox class.
 */

// std.
#include <map>
#include <utility>    // pair
#include <algorithm>  // find_if()

// Qt.
#include <QObject>
#include <QString>
#include <QGroupBox>
#include <QGridLayout>
#include <QLabel>
#include <QComboBox>
#include <QSpinBox>

// Headers of this package.
#include <morphology_filter/groupbox.h>
#include <morphology_filter/observed_parameter_manager.h>

using namespace std;


namespace morphology_filter {

const std::map<QString, ObservedParameterManager::Operation>
    GroupBox::operation_mapping {
          {"erode",   ObservedParameterManager::Operation::erode},
          {"dilate",  ObservedParameterManager::Operation::dilate},
          {"open",    ObservedParameterManager::Operation::open},
          {"close",   ObservedParameterManager::Operation::close}
    };

////////////////////////////////////////////////////////////////////////////////
//
// Construction & Initialization.
//

GroupBox::GroupBox(QWidget* parent)
    : QGroupBox {"&Morphology Filter", parent},
    param_manager_ {nullptr}
{
  initOperationWidgets();
  initNumIterationsWidgets();

  QGridLayout* main_layout {createMainLayout()};
  setLayout(main_layout);
}


void GroupBox::initOperationWidgets()
{
  op_label_ = new QLabel {"Operation:"};

  op_combo_ = new QComboBox;
  for (const auto& pr : operation_mapping)
    op_combo_->addItem(pr.first);
  connect(op_combo_, &QComboBox::currentTextChanged,
          this, &GroupBox::requestOperationChange);
}


void GroupBox::initNumIterationsWidgets()
{
  iter_label_ = new QLabel {"Iterations:"};

  iter_spinbox_ = new QSpinBox;
  iter_spinbox_->setRange(ObservedParameterManager::min_num_iterations,
                          ObservedParameterManager::max_num_iterations);
  connect(iter_spinbox_,
          static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged),
          this, &GroupBox::requestNumIterationsChange);
}


QGridLayout* GroupBox::createMainLayout() const
{
  QGridLayout* layout {new QGridLayout};

  layout->addWidget(op_label_, 0, 0);
  layout->addWidget(op_combo_, 0, 1);
  layout->addWidget(iter_label_, 1, 0);
  layout->addWidget(iter_spinbox_, 1, 1);

  return layout;
}


////////////////////////////////////////////////////////////////////////////////
//
// Observation-related member functions.
//

void GroupBox::update()
{
  if (param_manager_) {
    // Update op_combo_.
    ObservedParameterManager::Operation op = param_manager_->operation();
    auto it = find_if(operation_mapping.begin(), operation_mapping.end(),
                      [op](const pair<decltype(operation_mapping)::key_type,
                           decltype(operation_mapping)::mapped_type>& pr)
                      { return pr.second == op; });
    assert(it != operation_mapping.end());
    op_combo_->setCurrentText(it->first);

    // Update iter_spinbox_.
    iter_spinbox_->setValue(param_manager_->numIterations());
  }
}


void GroupBox::setParameterManager(ObservedParameterManager* const pm)
{
  param_manager_ = pm;
  update();
}


//////////////////////////////////////////////////////////////////////////////////////
//
// Private slots.
//

void GroupBox::requestOperationChange(const QString& text) const
{
  if (param_manager_)
    param_manager_->setOperation(operation_mapping.at(text));
}


void GroupBox::requestNumIterationsChange(const int val) const
{
  if (param_manager_)
    param_manager_->setNumIterations(val);
}

} // morphology_filter
