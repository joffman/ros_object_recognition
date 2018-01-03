/** \file
 * \brief Definition of the GUI class.
 */

#ifndef OBJECT_DETECTION_3D_GUI_H
#define OBJECT_DETECTION_3D_GUI_H

// std.
#include <string>

// Qt and VTK.
#include <QMainWindow>
#include <vtkRenderWindow.h>

// PCL.
#include <pcl/visualization/pcl_visualizer.h>

// Our headers.
#include <object_detection_3d_msgs/DetectedObject3D.h>
#include "pcl_types.h"
#include "parameter_storage.h"

// Forward declarations.
class QWidget;
class QHBoxLayout;
class QVBoxLayout;
class QAction;
class QMenu;
class QLabel;
class QVTKWidget;


namespace object_detection_3d {

/** \brief GUI for managing parameters and rendering point clouds and objects in
 *  3D object recognition systems.
 */
class GUI : public QMainWindow {
  Q_OBJECT

public:
  /** \brief Creates and initializes the GUI.
   * \param[in] parent QWidget that serves as a parent of the GUI.
   */
  explicit GUI(QWidget* parent = nullptr);

  //
  // Rendering functions.
  //

  /** \brief Removes all point clouds, shapes, and coordinate frames from the
   *  GUI.
   */
  void clearRenderWindow();

  /** \brief Renders the given point cloud. */
  void showCloud(const PointCloudT::ConstPtr& cloud_ptr);

  /** \brief Renders the given point cloud in red.
   * \todo Add a parameter for the color of the cloud.
   */
  void showColoredCloud(const PointCloudT::ConstPtr& cloud_ptr);

  /** \brief Renders an object.
   *
   * The object is represented by a coordinate frame whose pose is equal to the
   * pose of the object's center and by a 3-dimensional box that is equal to
   * the bounding box of the object.
   */
  void showObject(const object_detection_3d_msgs::DetectedObject3D& object);

  //
  // Adding components.
  //

  /** \brief Sets the ParameterStorage reference that is used to create
   *  and load parameter-files.
   */
  void setParameterStorage(ParameterStorage* storage);

  /** \brief Adds a widget to the right side of the GUI. */
  void addRightWidget(QWidget* widget);

private slots:
  bool open();
  bool save();
  bool saveAs();
  void about();

private:
  // Initialization functions.
  void initActions();
  void initOpenAction();
  void initSaveAction();
  void initSaveAsAction();
  void initQuitAction();
  void initAboutAction();

  void initMenus();
  void initFileMenu();
  void initHelpMenu();

  void initRenderWindow();
  void initRightLayout();

  QHBoxLayout* createMainLayout() const;

  void initStatusBar();

  // Other private functions.
  bool saveFile(const std::string& filename) const;
  bool loadFile(const std::string& filename) const;

  void setCurrentFile(const std::string& filename);

  // Data members.
  pcl::visualization::PCLVisualizer visualizer_;
  QVTKWidget* qvtk_widget_;
  QVBoxLayout* right_layout_;

  int cloud_idx_;
  int object_idx_;

  ParameterStorage* storage_;
  std::string current_file_;

  QMenu* file_menu_;
  QMenu* help_menu_;

  QAction* open_action_;
  QAction* save_action_;
  QAction* save_as_action_;
  QAction* quit_action_;
  QAction* about_action_;

  QLabel* file_label_;

  // Names that are handles for the pcl-visualizer.
  static const std::string CLOUD_NAME;
  static const std::string OBJECT_FRAME_NAME;
  static const std::string OBJECT_BOX_NAME;
};

} // object_detection_3d

#endif  // OBJECT_DETECTION_3D_GUI_H
