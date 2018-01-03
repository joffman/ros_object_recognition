/** \file
 * \brief Definition of the ObservedParameterManager class.
 */

#ifndef DISTANCE_FILTER_OBSERVED_PARAMETER_MANAGER_H
#define DISTANCE_FILTER_OBSERVED_PARAMETER_MANAGER_H

// std.
#include <string>

// Our headers.
#include <object_detection/subject.h>
#include <object_detection_3d/parametrizable.h>
#include "parameter_manager.h"


namespace distance_filter {

/** \brief Observable parameter-manager for a DistanceFilter object.
 *
 * This ParameterManager inherits from object_detection:Subject,
 * making it possible to register observers for it.
 * It also inherits from object_detection_3d::Parametrizable.
 *
 * Using this parameter-manager makes it possible to access the parameters
 * of an DistanceFilter via the object_detection_3d::GUI.
 */
class ObservedParameterManager : public ParameterManager,
    public object_detection::Subject,
    public object_detection_3d::Parametrizable {

  using MultiClassParameterMap = Parametrizable::MultiClassParameterMap;
  using SingleClassParameterMap = Parametrizable::SingleClassParameterMap;

public:
  //
  // Parametrizable interface.
  //

  /** Creates a parameter-map that contains the values of all parameters. */
  MultiClassParameterMap createParameterMap() const override;

  /** Sets all parameters to the values found in the given parameter-map. */
  void setParametersFromMap(const MultiClassParameterMap&) override;

  //
  // Setters.
  //

  /** Sets the minimum-distance parameter. */
  void setMinDistance(double);

  /** Sets the maximum-distance parameter. */
  void setMaxDistance(double);

private:
  void setParametersFromSingleClassMap(
      const SingleClassParameterMap& parameters);
  void setDistancesFromStrings( // maybe better call it 'setParameters...'?!
      const std::string& min_dist_str, const std::string& max_dist_str);

  bool trySetDistances(const double min_dist, const double max_dist);
};

} // distance_filter

#endif  // DISTANCE_FILTER_OBSERVED_PARAMETER_MANAGER_H
