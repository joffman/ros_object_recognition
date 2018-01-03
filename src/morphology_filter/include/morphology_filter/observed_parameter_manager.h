/** \file
 * \brief Definition of the ObservedParameterManager class.
 */

#ifndef MORPHOLOGY_FILTER_OBSERVED_PARAMETER_MANAGER_H
#define MORPHOLOGY_FILTER_OBSERVED_PARAMETER_MANAGER_H

// Headers of this project.
#include <object_detection/subject.h>
#include <object_detection_2d/parametrizable.h>
#include "parameter_manager.h"


namespace cv {
class FileStorage;
}
    
namespace morphology_filter {

/** \brief Observable parameter-manager for a MorphologyFilter object.
 *
 * This ParameterManager inherits from object_detection:Subject,
 * making it possible to register observers for it.
 * It also inherits from object_detection_2d::Parametrizable.
 *
 * Using this parameter-manager makes it possible to access the parameters
 * of a MorphologyFilter via the object_detection_2d::GUI.
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

  /** \brief Sets the operation parameter. */
  void setOperation(Operation);
  /** \brief Sets the number-iterations parameter. */
  void setNumIterations(int);
};

} // morphology_filter

#endif  // MORPHOLOGY_FILTER_OBSERVED_PARAMETER_MANAGER_H
