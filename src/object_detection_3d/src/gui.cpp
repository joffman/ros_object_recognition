/** \file
 * \brief Implementation of the GUI class.
 */

// std.
#include <string>
#include <sstream>
#include <stdexcept>  // runtime_error

// Eigen.
#include <Eigen/Geometry>

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
#include <QVTKWidget.h>

// PCL.
#include <pcl/visualization/point_cloud_handlers.h>

// Our headers.
#include <object_detection_3d/gui.h>
#include <object_detection_3d_msgs/DetectedObject3D.h>

using namespace std;
using object_detection_3d_msgs::DetectedObject3D;


namespace object_detection_3d {

namespace {

/** \brief Compute affine transformation for object.
 *
 * An affine transformation for transforming the given object from the origin
 * to its current pose is computed and returned.
 *
 * \param[in] object Object for which the transformation is computed.
 * \return Affine transform that would move the object from the origin to
 *    its current pose.
 */
// TODO: Is 'compute' the right term? Is it consistent?
// TODO: Create converting functions for converting from object/ros transforms
//  to Eigen-transforms or other functions; or maybe even a class like
//  ObjectTransformation?
Eigen::Affine3f computeObjectTransform(const DetectedObject3D& object)
{
  /* Alternative 1:
     * Eigen::Affine3f affine {Eigen::Affine3f::Identity()};
     * setRotationFromObject(object, affine);
     * setTranslationFromObject(object, affine);
     * return affine;
     */

  /* Alternative 2:
     * ObjectTransform transform; ||
     *    ObjectTransform transform {ObjectTransform::fromObject(object)};
     * transform.setObject(object);
     * Eigen::Affine3f affine {transform.getAffine()};
     * return affine;
     */

  Eigen::Affine3f affine {Eigen::Affine3f::Identity()};

  // Set rotation.
  Eigen::Quaternionf quaternion {
    static_cast<float>(object.box.pose.orientation.w),
    static_cast<float>(object.box.pose.orientation.x),
    static_cast<float>(object.box.pose.orientation.y),
    static_cast<float>(object.box.pose.orientation.z)
  };
  affine.matrix().topLeftCorner<3,3>() = quaternion.toRotationMatrix();

  // Set translation.
  Eigen::Vector3f translation {
    static_cast<float>(object.box.pose.position.x),
    static_cast<float>(object.box.pose.position.y),
    static_cast<float>(object.box.pose.position.z)
  };
  affine.translation() = translation;

  // Return transform.
  return affine;
}

} // anonymous ns


const std::string GUI::CLOUD_NAME {"cloud"};
const std::string GUI::OBJECT_FRAME_NAME {"object frame"};
const std::string GUI::OBJECT_BOX_NAME {"object box"};


////////////////////////////////////////////////////////////////////////////////
//
// Construction & Initialization.
//

GUI::GUI(QWidget* parent)
    : QMainWindow {parent},
    visualizer_ {"3D Viewer", false},
    qvtk_widget_ {new QVTKWidget},
    cloud_idx_ {0},
    object_idx_ {0},
    storage_ {nullptr}
{
  initActions();
  initMenus();
  initRenderWindow();
  initRightLayout();
  QHBoxLayout* main_layout {createMainLayout()};

  QWidget* central_widget {new QWidget};
  central_widget->setLayout(main_layout);
  setCentralWidget(central_widget);

  initStatusBar();
}


void GUI::initRenderWindow()
{
  // Connect qvtk-widget and pcl-visualizer.
  qvtk_widget_->SetRenderWindow(visualizer_.getRenderWindow());
  visualizer_.setupInteractor(qvtk_widget_->GetInteractor(),
                              qvtk_widget_->GetRenderWindow());

  // Settings the minimum size.
  // This seems to be necessary. We get a segfault otherwise.
  qvtk_widget_->setMinimumSize(600, 400);
}


void GUI::initRightLayout()
{
  right_layout_ = new QVBoxLayout;
  right_layout_->addStretch();
}


QHBoxLayout* GUI::createMainLayout() const
{
  QHBoxLayout* main_layout {new QHBoxLayout};
  main_layout->addWidget(qvtk_widget_);
  main_layout->addLayout(right_layout_);

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

void GUI::clearRenderWindow()
{
  // Clear visualizer.
  visualizer_.removeAllPointClouds();
  cloud_idx_ = 0;
  visualizer_.removeAllShapes();
  for (int obj_idx {0}; obj_idx != object_idx_; ++obj_idx) {
    ostringstream oss;
    oss << OBJECT_FRAME_NAME << obj_idx;
    visualizer_.removeCoordinateSystem(oss.str());
  }   // TODO: Put this into 'removeAllFrames()'-method.
  object_idx_ = 0;

  // Update vtk-window.
  qvtk_widget_->update();
}


void GUI::showCloud(const PointCloudT::ConstPtr& cloud_ptr)
{
  // Add point cloud to graphics window and update it.
  ostringstream oss;
  oss << CLOUD_NAME << cloud_idx_;
  visualizer_.addPointCloud(cloud_ptr, oss.str());
  ++cloud_idx_;
  qvtk_widget_->update();
}


// TODO: Clean this mess up. Everything is redundant (see showCloud()).
void GUI::showColoredCloud(const PointCloudT::ConstPtr& cloud_ptr)
{
  pcl::visualization::PointCloudColorHandlerCustom<PointT> color {
    cloud_ptr, 255, 0, 0
  };

  ostringstream oss;
  oss << CLOUD_NAME << cloud_idx_;

  visualizer_.addPointCloud(cloud_ptr, color, oss.str());
  ++cloud_idx_;
  qvtk_widget_->update();
}


void GUI::showObject(const DetectedObject3D& object)
{
  // Create Affine3f to represent pose of bounding box.
  Eigen::Affine3f affine {computeObjectTransform(object)};

  // Add object to visualizer and update window.
  {
    ostringstream name_oss;
    name_oss << OBJECT_FRAME_NAME << object_idx_;
    visualizer_.addCoordinateSystem(1.0, affine, name_oss.str());
  }
  // Note: PCL's "height" is measured in the y-direction ("camera-frame"). We
  // measure the height along the z-axis according to the normal "ROS-frame".
  // That's why 'our height' is "PCL's depth" and the other way around.
  // Similar width and depth are turned around.
  // Is this the right logic or should we confirm to PCL's convention?
  {
    ostringstream name_oss;
    name_oss << OBJECT_BOX_NAME << object_idx_;
    visualizer_.addCube(
        affine.translation(), Eigen::Quaternionf {affine.rotation()},
        object.box.depth, object.box.width, object.box.height,
        name_oss.str()
        );
  }

  ++object_idx_;
  qvtk_widget_->update();
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


////////////////////////////////////////////////////////////////////////////////
//
// Private slots & related functions.
//

bool GUI::open()
{
  QString q_filename {
    QFileDialog::getOpenFileName(
        this,
        "Open Object-Definition",
        ".",
        "3D object definitions (*.object3d)"
        )
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
    QFileDialog::getSaveFileName(
        this,
        "Save Object-Definition",
        ".",
        "3D object definitions (*.object3d)"
        )
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
  QMessageBox::about(this, "About Object-Detection-3D",
                     "<h2>Object-Detection-3D</h2>"
                     "<p>Combine filters with a detector to detect 3D objects."
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

} // object_detection_3d
