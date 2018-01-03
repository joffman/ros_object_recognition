/** \file
 * \brief Implementation of the GroupBox class.
 */

// std.
#include <vector>

// Qt.
#include <QObject>
#include <QWidget>
#include <QGroupBox>
#include <QLabel>
#include <QSlider>
#include <QLCDNumber>
#include <QGridLayout>

// Headers of this project.
#include <binary_detector/groupbox.h>
#include <binary_detector/observed_parameter_manager.h>

using namespace std;


namespace binary_detector {

namespace {

int lengthFractionToTicks(const double frac)
{
  constexpr double range {
    ObservedParameterManager::length_fraction_upper_limit -
    ObservedParameterManager::length_fraction_lower_limit};

  return static_cast<int>(
      GroupBox::num_slider_ticks *
      (frac - ObservedParameterManager::length_fraction_lower_limit) / range);
}
  

double ticksToLengthFraction(const int ticks)
{
  constexpr double range {ObservedParameterManager::length_fraction_upper_limit -
    ObservedParameterManager::length_fraction_lower_limit};

  return ObservedParameterManager::length_fraction_lower_limit +
      ticks * range / GroupBox::num_slider_ticks;
}

} // anonymous namespace


GroupBox::GroupBox(QWidget* parent)
  : QGroupBox {"&Binary Detector", parent}
{
  min_length_label_ = new QLabel {"min-length-fraction:"};
  max_length_label_ = new QLabel {"max-length-fraction:"};

  min_length_slider_ = new QSlider {Qt::Horizontal};
  max_length_slider_ = new QSlider {Qt::Horizontal};

  vector<QSlider*> sliders {min_length_slider_, max_length_slider_};
  for (auto& sl : sliders) {
    sl->setMinimum(0);
    sl->setMaximum(num_slider_ticks);

    sl->setSingleStep(10);
    sl->setPageStep(100);
  }
  connect(min_length_slider_, &QSlider::valueChanged,
          this, &GroupBox::requestMinLengthChange);
  connect(max_length_slider_, &QSlider::valueChanged,
          this, &GroupBox::requestMaxLengthChange);

  min_length_lcd_ = new QLCDNumber;
  max_length_lcd_ = new QLCDNumber;
  /*
  vector<QLCDNumber*> lcds {min_length_lcd_, max_length_lcd_};
  for (auto& l : lcds)
    l->setDigitCount(5);
    */
  connect(this, &GroupBox::minLCDSignal,
          min_length_lcd_, static_cast<void(QLCDNumber::*)(double)>(
              &QLCDNumber::display));
  connect(this, &GroupBox::maxLCDSignal,
          max_length_lcd_, static_cast<void(QLCDNumber::*)(double)>(
              &QLCDNumber::display));

  QGridLayout* layout {new QGridLayout};
  layout->addWidget(min_length_label_, 0, 0);
  layout->addWidget(min_length_slider_, 0, 1);
  layout->addWidget(min_length_lcd_, 0, 2);
  layout->addWidget(max_length_label_, 1, 0);
  layout->addWidget(max_length_slider_, 1, 1);
  layout->addWidget(max_length_lcd_, 1, 2);

  setLayout(layout);
}


void GroupBox::update()
{
  if (param_manager_) {
    const double min_length_frac = param_manager_->minLengthFraction();
    min_length_slider_->setValue(
        lengthFractionToTicks(min_length_frac));
    emit minLCDSignal(min_length_frac);

    const double max_length_frac = param_manager_->maxLengthFraction();
    max_length_slider_->setValue(
        lengthFractionToTicks(max_length_frac));
    emit maxLCDSignal(max_length_frac);
  }
}


void GroupBox::setParameterManager(ObservedParameterManager* const pm)
{
  param_manager_ = pm;
  update();
}


////////////////////////////////////////////////////////////////////////////////
//
// Private slots.
//

void GroupBox::requestMinLengthChange(const int ticks) const
{
  if (param_manager_)
    param_manager_->setMinLengthFraction(ticksToLengthFraction(ticks));
}


void GroupBox::requestMaxLengthChange(const int ticks) const
{
  if (param_manager_)
    param_manager_->setMaxLengthFraction(ticksToLengthFraction(ticks));
}

} // binary_detector
