/** \file
 * \brief Definition of the ParameterStorage class.
 */

#ifndef OBJECT_DETECTION_2D_PARAMETER_STORAGE_H
#define OBJECT_DETECTION_2D_PARAMETER_STORAGE_H

// std.
#include <string>
#include <vector>

// object_detection*.
#include "parametrizable.h"


namespace object_detection_2d {

/** \brief Class for handling the saving/loading of parameters
 *  to/from parameter files.
 */
class ParameterStorage {
public:
  /** \brief Registers a Parametrizable object to the internal vector of
   *  references.
   */
  void addParametrizable(Parametrizable*);

  /** \brief Saves the parameters of all registered Parametrizable objects
   *  to a parameter file.
   *
   *  The function creates the file with the given name and passes a handle
   *  to the file to all registered Parametrizable objects, asking them to
   *  write their parameters to the file.
   *
   * \param[in] filename Name of the parameter-file.
   * \return true if the file has been created successfully, false otherwise.
   */
  bool saveParameters(const std::string& filename) const;

  /** \brief Opens a parameter-file and asks all registered Parametrizable
   *  objects to write their parameters to that file.
   *
   * \param[in] filename Name of the parameter-file.
   * \return true if the file with the given name has been opened successfully,
   *  false otherwise.
   */
  bool loadParameters(const std::string& filename) const;

private:
  std::vector<Parametrizable*> parametrizables_;
};

} // object_detection_2d

#endif  // OBJECT_DETECTION_2D_PARAMETER_STORAGE_H
