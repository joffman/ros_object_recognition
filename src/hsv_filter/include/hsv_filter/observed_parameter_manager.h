/** \file
 * \brief Definition of the ObservedParameterManager class.
 */

#ifndef HSV_FILTER_OBSERVED_PARAMETER_MANAGER_H
#define HSV_FILTER_OBSERVED_PARAMETER_MANAGER_H

// Headers of this project.
#include <object_detection/subject.h>
#include <object_detection_2d/parametrizable.h>
#include "parameter_manager.h"


namespace cv {
class FileStorage;
}
    
namespace hsv_filter {

/** \brief Observable parameter-manager for an HSVFilter object.
 *
 * This ParameterManager inherits from object_detection:Subject,
 * making it possible to register observers for it.
 * It also inherits from object_detection_2d::Parametrizable.
 *
 * Using this parameter-manager makes it possible to access the parameters
 * of an HSVFilter via the object_detection_2d::GUI.
 */
class ObservedParameterManager : public ParameterManager,
    public object_detection::Subject,
    public object_detection_2d::Parametrizable {
public:
  //
  // Parametrizable interface.
  //

  /** \brief Writes all parameters to the given FileStorage object. */
  void writeParametersToStorage(cv::FileStorage&) const override;

  /** \brief Reads all parameters from the given FileStorage object
   *  and sets its parameters accordingly.
   */
  void setParametersFromStorage(cv::FileStorage&) override;

  //
  // Setters.
  //
  /** \brief Sets the h-min parameter. */
  void setHMin(int);
  /** \brief Sets the h-min parameter. */
  void setHMax(int);

  /** \brief Sets the s-min parameter. */
  void setSMin(int);
  /** \brief Sets the s-min parameter. */
  void setSMax(int);

  /** \brief Sets the v-min parameter. */
  void setVMin(int);
  /** \brief Sets the v-min parameter. */
  void setVMax(int);
};

} // hsv_filter

#endif  // HSV_FILTER_OBSERVED_PARAMETER_MANAGER_H
