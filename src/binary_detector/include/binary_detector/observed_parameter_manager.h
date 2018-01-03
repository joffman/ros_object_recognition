/** \file
 * \brief Definition of the ObservedParameterManager class.
 */

#ifndef BINARY_DETECTOR_OBSERVED_PARAMETER_MANAGER_H
#define BINARY_DETECTOR_OBSERVED_PARAMETER_MANAGER_H

// std.
#include <string>

// Headers of this project.
#include <object_detection/subject.h>
#include <object_detection_2d/parametrizable.h>
#include "parameter_manager.h"


namespace cv {
class FileStorage;
}
    
namespace binary_detector {

/** \brief Observable parameter-manager for a BinaryDetector object.
 *
 * This ParameterManager inherits from object_detection:Subject,
 * making it possible to register observers for it.
 * It also inherits from object_detection_2d::Parametrizable.
 *
 * Using this parameter-manager makes it possible to access the parameters
 * of a BinaryDetector via the object_detection_2d::GUI.
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

  /** \brief Sets the name of detected objects. */
  void setObjectName(const std::string&);


  /** \brief Sets the minimum-length-fraction parameter. */
  void setMinLengthFraction(double);

  /** \brief Sets the maximum-length-fraction parameter. */
  void setMaxLengthFraction(double);
};

} // binary_detector

#endif  // BINARY_DETECTOR_OBSERVED_PARAMETER_MANAGER_H
