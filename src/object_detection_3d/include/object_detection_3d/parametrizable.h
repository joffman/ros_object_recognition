/** \file
 * \brief Definition of the Parametrizable class.
 */

#ifndef OBJECT_DETECTION_3D_PARAMETRIZABLE_H
#define OBJECT_DETECTION_3D_PARAMETRIZABLE_H

// std.
#include <string>
#include <map>


namespace object_detection_3d {

/** \brief Class for objects that have parameters and that are able to create a
 *  parameter-map that contains the values of those parameters and
 *  to extract those values from such a map.
 */
class Parametrizable {
public:
  using SingleClassParameterMap = std::map<std::string,std::string>;
  using MultiClassParameterMap = std::map<std::string,SingleClassParameterMap>;

  /** \brief Returns a parameter-map that contains the names and values
   *  of all of the object's parameters.
   */
  virtual MultiClassParameterMap createParameterMap() const = 0;

  /** \brief Reads the values of the parameters of the object from the
   *  given parameter-map and sets the internal parameters accordingly.
   */
  virtual void setParametersFromMap(const MultiClassParameterMap&) = 0;
};

} // object_detection_3d

#endif  // OBJECT_DETECTION_3D_PARAMETRIZABLE_H
