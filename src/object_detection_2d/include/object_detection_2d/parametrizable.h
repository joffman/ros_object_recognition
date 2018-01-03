/** \file
 * \brief Definition of the Parametrizable class.
 */

#ifndef OBJECT_DETECTION_2D_PARAMETRIZABLE_H
#define OBJECT_DETECTION_2D_PARAMETRIZABLE_H

#include <opencv2/core/persistence.hpp>


namespace object_detection_2d {

/** \brief Class for objects that have parameters and are able to write
 *  them to and load them from a parameter-file.
 */
class Parametrizable {
  // TODO: Do we need a virtual destructor?
public:
  /** \brief Writes the parameters of the object to the given FileStorage. */
  virtual void writeParametersToStorage(cv::FileStorage&) const = 0;

  /** \brief Reads the values of the parameters of the object from the
   *  given FileStorage and sets the internal parameters accordingly.
   */
  virtual void setParametersFromStorage(cv::FileStorage&) = 0;
};

} // object_detection_2d

#endif  // OBJECT_DETECTION_2D_PARAMETRIZABLE_H
