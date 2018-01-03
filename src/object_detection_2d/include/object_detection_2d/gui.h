/** \file
 * \brief Definition of the GUI class.
 */

#ifndef OBJECT_DETECTION_2D_GUI_H
#define OBJECT_DETECTION_2D_GUI_H

// std.
#include <functional>
#include <string>

// Qt.
#include <QMainWindow>

// OpenCV.
#include <cv_bridge/cv_bridge.h>  // Mat, Rect

// Forward declarations.
class QWidget;
class QHBoxLayout;
class QVBoxLayout;
class QAction;
class QMenu;
class QLabel;
class QRect;


namespace object_detection_2d {

class InteractiveImage;
class ParameterStorage;


/** \brief GUI for managing parameters and rendering images and objects in
 *  2D object recognition systems.
 */
class GUI : public QMainWindow {
  Q_OBJECT

public:
  using AreaSelectionCallback = std::function<void(const cv::Rect&)>;

  /** \brief Creates and initializes the GUI.
   * \param[in] parent QWidget that serves as a parent of the GUI.
   */
  explicit GUI(QWidget* parent = nullptr);

  /** \brief Renders the given image in the GUI. */
  void showImage(const cv::Mat& image) const;

  /** \brief Sets the ParameterStorage reference that is used to create
   *  and load parameter-files.
   */
  void setParameterStorage(ParameterStorage* storage);
  
  /** \brief Adds a widget to the right side of the GUI. */
  void addRightWidget(QWidget* widget);
  
  /** \brief Registers a callback function for area selections.
   *
   * Registered callback functions are called when the user selects a
   * rectangular area within the rendered image.
   */
  void registerAreaSelectionCallback(AreaSelectionCallback cb);

private slots:
  void areaSelected(const QRect& area);
  bool open();
  bool save();
  bool saveAs();
  void about();

private:
  // Initialization functions.
  void initRenderWindow();
  void initActions();
  void initOpenAction();
  void initSaveAction();
  void initSaveAsAction();
  void initQuitAction();
  void initAboutAction();

  void initMenus();
  void initFileMenu();
  void initHelpMenu();

  void initRightLayout();

  QHBoxLayout* createMainLayout() const;

  void initStatusBar();

  // Others private functions.
  bool saveFile(const std::string& filename) const;
  bool loadFile(const std::string& filename) const;

  void setCurrentFile(const std::string& filename);

  // Data members.
  InteractiveImage* inter_image_;
  QVBoxLayout* right_layout_;

  QMenu* file_menu_;
  QMenu* help_menu_;

  QAction* open_action_;
  QAction* save_action_;
  QAction* save_as_action_;
  QAction* quit_action_;
  QAction* about_action_;

  QLabel* file_label_;

  ParameterStorage* storage_;
  std::string current_file_;

  std::vector<AreaSelectionCallback> area_selection_cbs_;
    // use pointer to functions instead?
};

} // object_detection_2d

#endif  // OBJECT_DETECTION_2D_GUI_H
