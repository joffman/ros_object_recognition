/** \file
 * \brief Implementation of the GUI class.
 */

// std.
#include <iostream>
#include <string>
#include <stdexcept>  // runtime_error

// Qt.
#include <QWidget>
#include <QString>
#include <QMainWindow>
#include <QMenuBar>
#include <QMenu>
#include <QAction>
#include <QStatusBar>
#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QMessageBox>
#include <QFileDialog>
#include <QRect>

// object_detection_*.
#include <object_detection_2d/parameter_storage.h>
#include <object_detection_2d/gui.h>
#include <object_detection_2d/interactive_image.h>

using namespace std;


namespace object_detection_2d {

////////////////////////////////////////////////////////////////////////////////
//
// Construction & Initialization.
//
GUI::GUI(QWidget* parent)
    : QMainWindow{parent},
    inter_image_{nullptr},
    storage_{nullptr}
{
  initRenderWindow();
  initActions();
  initMenus();
  initRightLayout();
  QHBoxLayout* main_layout {createMainLayout()};

  QWidget* central_widget {new QWidget};
  central_widget->setLayout(main_layout);
  setCentralWidget(central_widget);

  initStatusBar();

  setWindowTitle("Object Detection 2D");
}


void GUI::initRenderWindow()
{
  inter_image_ = new InteractiveImage;
  connect(inter_image_, &InteractiveImage::areaSelected,
          this, &GUI::areaSelected);
}


void GUI::initRightLayout()
{
  right_layout_ = new QVBoxLayout;
  right_layout_->addStretch();
}


QHBoxLayout* GUI::createMainLayout() const
{
  QVBoxLayout* image_layout {new QVBoxLayout};
  image_layout->addWidget(inter_image_);
  image_layout->addStretch();

  QHBoxLayout* main_layout {new QHBoxLayout};
  main_layout->addLayout(image_layout);
  main_layout->addLayout(right_layout_);
  main_layout->addStretch();

  return main_layout;
}


void GUI::initActions()
{
  initOpenAction();
  initSaveAction();
  initSaveAsAction();
  initQuitAction();
  initAboutAction();
}


void GUI::initOpenAction()
{
  open_action_ = new QAction {"&Open", this};
  open_action_->setShortcut(QKeySequence::Open);
  open_action_->setStatusTip("Load an existing object defintion");
  connect(open_action_, &QAction::triggered, this, &GUI::open);
}


void GUI::initSaveAction()
{
  save_action_ = new QAction {"&Save", this};
  save_action_->setShortcut(QKeySequence::Save);
  save_action_->setStatusTip("Save object definition");
  connect(save_action_, &QAction::triggered, this, &GUI::save);
}


void GUI::initSaveAsAction()
{
  save_as_action_ = new QAction {"Save &As", this};
  save_as_action_->setShortcut(QKeySequence::SaveAs);
  save_as_action_->setStatusTip("Save new object definition");
  connect(save_as_action_, &QAction::triggered, this, &GUI::saveAs);
}


void GUI::initQuitAction()
{
  quit_action_ = new QAction {"&Quit", this};
  quit_action_->setShortcut(tr("Ctrl+Q"));
  quit_action_->setStatusTip("Exit application");
  connect(quit_action_, &QAction::triggered, this, &QMainWindow::close);
}


void GUI::initAboutAction()
{
  about_action_ = new QAction {"&About", this};
  about_action_->setStatusTip("Short explanation of this application");
  connect(about_action_, &QAction::triggered, this, &GUI::about);
}


void GUI::initMenus()
{
  initFileMenu();
  initHelpMenu();
}


void GUI::initFileMenu()
{
  file_menu_ = menuBar()->addMenu("&File");
  file_menu_->addAction(open_action_);
  file_menu_->addAction(save_action_);
  file_menu_->addAction(save_as_action_);
  file_menu_->addSeparator();
  file_menu_->addAction(quit_action_);
}


void GUI::initHelpMenu()
{
  help_menu_ = menuBar()->addMenu("&Help");
  help_menu_->addAction(about_action_);
}


void GUI::initStatusBar()
{
  file_label_ = new QLabel {current_file_.c_str()};
  statusBar()->addWidget(file_label_);
}


////////////////////////////////////////////////////////////////////////////////
//
// Rendering.
//

void GUI::showImage(const cv::Mat& image) const
{
  inter_image_->showImage(image);
}


////////////////////////////////////////////////////////////////////////////////
//
// Adding/Setting components.
//

void GUI::setParameterStorage(ParameterStorage* storage)
{
  storage_ = storage;
}


void GUI::addRightWidget(QWidget* widget)
{
  // Insert widget in front of the terminating stretch.
  right_layout_->insertWidget(right_layout_->count()-1, widget);
}


void GUI::registerAreaSelectionCallback(AreaSelectionCallback cb)
{
  area_selection_cbs_.push_back(cb);
}


////////////////////////////////////////////////////////////////////////////////
//
// Private slots & related functions.
//

void GUI::areaSelected(const QRect& q_area)
{
  QRect norm_area {q_area.normalized()};
  cv::Rect cv_area {norm_area.left(), norm_area.top(),
    norm_area.width(), norm_area.height()};
  for (auto& cb : area_selection_cbs_)
    cb(cv_area);
}


bool GUI::open()
{
  QString q_filename {
    QFileDialog::getOpenFileName(this, "Open Parameter-File", ".",
        "XML file (*.xml)\n"
        "YAML file (*.yml *.yaml)")
  };
  if (q_filename.isEmpty()) {
    statusBar()->showMessage("Open canceled", 2000);
    return false;
  }
  setCurrentFile(q_filename.toStdString());

  return loadFile(current_file_);
}


bool GUI::loadFile(const string& filename) const
{
  if (!storage_)
    throw runtime_error("ParameterStorage member has not been set yet.");

  if (storage_->loadParameters(filename)) {
    statusBar()->showMessage("Loaded file successfully", 2000);
    return true;
  }
  statusBar()->showMessage("Loading file failed", 2000);
  return false;
}


bool GUI::save()
{
  if (current_file_.empty())
    return saveAs();
  return saveFile(current_file_);
}


bool GUI::saveAs()
{
  QString q_filename {
    QFileDialog::getSaveFileName(this, "Save Parameter-File", ".",
        "XML file (*.xml)\n"
        "YAML file (*.yml *.yaml)")
  };
  if (q_filename.isEmpty()) {
    statusBar()->showMessage("Save canceled", 2000);
    return false;
  }
  setCurrentFile(q_filename.toStdString());

  return saveFile(current_file_);
}


bool GUI::saveFile(const string& filename) const
{
  if (!storage_)
    throw runtime_error("ParameterStorage member has not been set yet.");

  if (storage_->saveParameters(filename)) {
    statusBar()->showMessage("Saved file successfully", 2000);
    return true;
  }
  statusBar()->showMessage("Saving file failed", 2000);
  return false;
}


void GUI::about()
{
  QMessageBox::about(this, "About Object-Detection-2D",
                     "<h2>Object-Detection-2D</h2>"
                     "<p>Combine filters with a detector to detect 2D objects."
                     "<p>Use the given widgets to parametrize the filters and "
                     "detectors. "
                     "Press 'File|Save' to save the current parameters as a "
                     "new object-definition file. "
                     "Press 'File|Open' to load an existing file.");
}


void GUI::setCurrentFile(const string& filename)
{
  current_file_ = filename;
  file_label_->setText(current_file_.c_str());
}

} // object_detection_2d
