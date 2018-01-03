/** \file
 * \brief Definition of the ParameterStorage class.
 */

#ifndef OBJECT_DETECTION_3D_PARAMETER_STORAGE_H
#define OBJECT_DETECTION_3D_PARAMETER_STORAGE_H

// std.
#include <string>
#include <vector>

// Our headers.
#include "parametrizable.h"


namespace object_detection_3d {

/** \brief Class for handling the saving/loading of parameters
 *  to/from parameter files.
 */
class ParameterStorage {
  using MultiClassParameterMap = Parametrizable::MultiClassParameterMap;
  using SingleClassParameterMap = Parametrizable::SingleClassParameterMap;

public:
  /** \brief Registers a Parametrizable object to the internal vector of
   *  references.
   */
  void addParametrizable(Parametrizable*);

  /** \brief Saves the parameters of all registered Parametrizable objects
   *  to a parameter file.
   *
   * The file with the given name is opened.
   * Then each registered Parametrizable is asked to provide a parameter-map
   * that contains all of its parameter-values, and those parameters are
   * written to the parameter file.
   *
   * \param[in] filename Name of the parameter-file.
   * \return true if the file has been created successfully, false otherwise.
   */
  bool saveParameters(const std::string& filename) const;

  /** \brief Reads parameters from the given parameter-file, collects them
   *  in a parameter-map, and passes the map to each registered Parametrizable.
   * \param[in] filename Name of the parameter-file.
   * \return true if the file with the given name has been opened successfully,
   *  false otherwise.
   */
  bool loadParameters(const std::string& filename) const;

private:
  std::vector<Parametrizable*> parametrizables_;
};

} // object_detection_3d

#endif  // OBJECT_DETECTION_3D_PARAMETER_STORAGE_H
