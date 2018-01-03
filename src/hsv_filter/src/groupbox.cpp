/** \file
 * \brief Implementation of the GroupBox class.
 */

// Qt.
#include <QObject>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QLabel>
#include <QSlider>
#include <QLCDNumber>

// Headers of this package.
#include <hsv_filter/groupbox.h>
#include <hsv_filter/observed_parameter_manager.h>

using namespace std;


namespace hsv_filter {

//////////////////////////////////////////////////////////////////////////////////////
//
// Construction & Initialization.
//

GroupBox::GroupBox(QWidget* parent)
    : QGroupBox {"&HSV Filter", parent},
    param_manager_ {nullptr}
{
  initLabels();
  initSliders();
  initLCDs();
  initGroupBoxes();

  QVBoxLayout* main_layout {createMainLayout()};
  setLayout(main_layout);
}


void GroupBox::initLabels()
{
  h_min_label_ = new QLabel {"h-min:"};
  h_max_label_ = new QLabel {"h-max:"};

  s_min_label_ = new QLabel {"s-min:"};
  s_max_label_ = new QLabel {"s-max:"};

  v_min_label_ = new QLabel {"v-min:"};
  v_max_label_ = new QLabel {"v-max:"};
}


void GroupBox::initSliders()
{
  allocateSliderMemory();
  setSliderRanges();
  setSliderSteps();
  connectSliderSignals();
}


void GroupBox::allocateSliderMemory()
{
  h_min_slider_ = new QSlider {Qt::Horizontal};
  h_max_slider_ = new QSlider {Qt::Horizontal};

  s_min_slider_ = new QSlider {Qt::Horizontal};
  s_max_slider_ = new QSlider {Qt::Horizontal};

  v_min_slider_ = new QSlider {Qt::Horizontal};
  v_max_slider_ = new QSlider {Qt::Horizontal};
}


void GroupBox::setSliderRanges() const
{
  vector<QSlider*> h_sliders {h_min_slider_, h_max_slider_};
  for (auto& hs : h_sliders) {
    hs->setMinimum(ObservedParameterManager::h_lower_limit);
    hs->setMaximum(ObservedParameterManager::h_upper_limit);
  }

  vector<QSlider*> s_sliders {s_min_slider_, s_max_slider_};
  for (auto& ss : s_sliders) {
    ss->setMinimum(ObservedParameterManager::s_lower_limit);
    ss->setMaximum(ObservedParameterManager::s_upper_limit);
  }

  vector<QSlider*> v_sliders {v_min_slider_, v_max_slider_};
  for (auto& vs : v_sliders) {
    vs->setMinimum(ObservedParameterManager::v_lower_limit);
    vs->setMaximum(ObservedParameterManager::v_upper_limit);
  }
}


void GroupBox::setSliderSteps() const
{
  vector<QSlider*> sliders {h_min_slider_, h_max_slider_,
    s_min_slider_, s_max_slider_,
    v_min_slider_, v_max_slider_};

  for (auto& s : sliders) {
    s->setSingleStep(1);
    s->setPageStep(10);
  }
}


void GroupBox::connectSliderSignals() const
{
  connect(h_min_slider_, &QSlider::valueChanged,
          this, &GroupBox::requestHMinChange);
  connect(h_max_slider_, &QSlider::valueChanged,
          this, &GroupBox::requestHMaxChange);

  connect(s_min_slider_, &QSlider::valueChanged,
          this, &GroupBox::requestSMinChange);
  connect(s_max_slider_, &QSlider::valueChanged,
          this, &GroupBox::requestSMaxChange);

  connect(v_min_slider_, &QSlider::valueChanged,
          this, &GroupBox::requestVMinChange);
  connect(v_max_slider_, &QSlider::valueChanged,
          this, &GroupBox::requestVMaxChange);
}


void GroupBox::initLCDs()
{
  allocateLCDMemory();
  setLCDDigitCounts();
  connectLCDs();
}


void GroupBox::allocateLCDMemory()
{
  h_min_lcd_ = new QLCDNumber;
  h_max_lcd_ = new QLCDNumber;

  s_min_lcd_ = new QLCDNumber;
  s_max_lcd_ = new QLCDNumber;

  v_min_lcd_ = new QLCDNumber;
  v_max_lcd_ = new QLCDNumber;
}


void GroupBox::setLCDDigitCounts() const
{
  vector<QLCDNumber*> lcds {h_min_lcd_, h_max_lcd_,
    s_min_lcd_, s_max_lcd_,
    v_min_lcd_, v_max_lcd_};
  for (auto& l : lcds)
    l->setDigitCount(3);
}


void GroupBox::connectLCDs() const
{
  connect(h_min_slider_, &QSlider::valueChanged,
      h_min_lcd_, static_cast<void(QLCDNumber::*)(int)>(&QLCDNumber::display));
  connect(h_max_slider_, &QSlider::valueChanged,
      h_max_lcd_, static_cast<void(QLCDNumber::*)(int)>(&QLCDNumber::display));

  connect(s_min_slider_, &QSlider::valueChanged,
      s_min_lcd_, static_cast<void(QLCDNumber::*)(int)>(&QLCDNumber::display));
  connect(s_max_slider_, &QSlider::valueChanged,
      s_max_lcd_, static_cast<void(QLCDNumber::*)(int)>(&QLCDNumber::display));

  connect(v_min_slider_, &QSlider::valueChanged,
      v_min_lcd_, static_cast<void(QLCDNumber::*)(int)>(&QLCDNumber::display));
  connect(v_max_slider_, &QSlider::valueChanged,
      v_max_lcd_, static_cast<void(QLCDNumber::*)(int)>(&QLCDNumber::display));
}


void GroupBox::initGroupBoxes()
{
  initHGroupBox();
  initSGroupBox();
  initVGroupBox();
}


void GroupBox::initHGroupBox()
{
  QGridLayout* layout {new QGridLayout};

  layout->addWidget(h_min_label_, 0, 0);
  layout->addWidget(h_min_slider_, 0, 1);
  layout->addWidget(h_min_lcd_, 0, 2);

  layout->addWidget(h_max_label_, 1, 0);
  layout->addWidget(h_max_slider_, 1, 1);
  layout->addWidget(h_max_lcd_, 1, 2);

  h_groupbox_ = new QGroupBox {"Hue"};
  h_groupbox_->setLayout(layout);
}


void GroupBox::initSGroupBox()
{
  QGridLayout* layout {new QGridLayout};

  layout->addWidget(s_min_label_, 0, 0);
  layout->addWidget(s_min_slider_, 0, 1);
  layout->addWidget(s_min_lcd_, 0, 2);

  layout->addWidget(s_max_label_, 1, 0);
  layout->addWidget(s_max_slider_, 1, 1);
  layout->addWidget(s_max_lcd_, 1, 2);

  s_groupbox_ = new QGroupBox {"Saturation"};
  s_groupbox_->setLayout(layout);
}


void GroupBox::initVGroupBox()
{
  QGridLayout* layout {new QGridLayout};

  layout->addWidget(v_min_label_, 0, 0);
  layout->addWidget(v_min_slider_, 0, 1);
  layout->addWidget(v_min_lcd_, 0, 2);

  layout->addWidget(v_max_label_, 1, 0);
  layout->addWidget(v_max_slider_, 1, 1);
  layout->addWidget(v_max_lcd_, 1, 2);

  v_groupbox_ = new QGroupBox {"Value"};
  v_groupbox_->setLayout(layout);
}


QVBoxLayout* GroupBox::createMainLayout() const
{
  QVBoxLayout* layout {new QVBoxLayout};

  layout->addWidget(h_groupbox_);
  layout->addWidget(s_groupbox_);
  layout->addWidget(v_groupbox_);

  return layout;
}


//////////////////////////////////////////////////////////////////////////////////////
//
// Observation-related member functions.
//

void GroupBox::update()
{
  if (param_manager_) {
    h_min_slider_->setSliderPosition(param_manager_->hMin());
    h_max_slider_->setSliderPosition(param_manager_->hMax());

    s_min_slider_->setSliderPosition(param_manager_->sMin());
    s_max_slider_->setSliderPosition(param_manager_->sMax());

    v_min_slider_->setSliderPosition(param_manager_->vMin());
    v_max_slider_->setSliderPosition(param_manager_->vMax());
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

void GroupBox::requestHMinChange(const int val) const
{
  if (param_manager_)
    param_manager_->setHMin(val);
}


void GroupBox::requestHMaxChange(const int val) const
{

  if (param_manager_)
    param_manager_->setHMax(val);
}


void GroupBox::requestSMinChange(const int val) const
{
  if (param_manager_)
    param_manager_->setSMin(val);
}


void GroupBox::requestSMaxChange(const int val) const
{
  if (param_manager_)
    param_manager_->setSMax(val);
}


void GroupBox::requestVMinChange(const int val) const
{
  if (param_manager_)
    param_manager_->setVMin(val);
}


void GroupBox::requestVMaxChange(const int val) const
{
  if (param_manager_)
    param_manager_->setVMax(val);
}

} // hsv_filter
