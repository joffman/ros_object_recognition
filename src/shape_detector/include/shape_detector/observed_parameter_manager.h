/** \file
 * \brief Definition of the ObservedParameterManager class.
 */

#ifndef SHAPE_DETECTOR_OBSERVED_PARAMETER_MANAGER
#define SHAPE_DETECTOR_OBSERVED_PARAMETER_MANAGER

// std.
#include <string>

// Our headers.
#include <object_detection/subject.h>
#include <object_detection_3d/parametrizable.h>
#include "shape_enum.h"
#include "parameter_manager.h"


namespace shape_detector {

/** \brief Observable parameter-manager for a ShapeDetector object.
 *
 * This ParameterManager inherits from object_detection:Subject,
 * making it possible to register observers for it.
 * It also inherits from object_detection_3d::Parametrizable.
 *
 * Using this parameter-manager makes it possible to access the parameters
 * of an ShapeDetector via the object_detection_3d::GUI.
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
  Parametrizable::MultiClassParameterMap createParameterMap() const override;

  /** Sets all parameters to the values found in the given parameter-map. */
  void setParametersFromMap(const Parametrizable::MultiClassParameterMap&)
      override;

  //
  // Setters.
  //

  void setObjectName(const std::string& name);
  void setShape(const Shape val);
  void setMinRadius(const double val);
  void setMaxRadius(const double val);
  void setNormalDistanceWeight(const double val);
  void setDistanceThreshold(const double val);
  void setNumNearestNeighbors(const int val);
  void setMaxIterations(const int val);
  void setOptimizeCoefficients(const bool val);

private:
  void setParametersFromSingleClassMap(const SingleClassParameterMap&);

  void setObjectNameFromMap(const SingleClassParameterMap&);

  void setShapeFromMap(const SingleClassParameterMap&);
  void setShapeFromString(const std::string&);

  void setRadiiFromMap(const SingleClassParameterMap&);
  void setRadiiFromStrings(
      const std::string& min_radius, const std::string& max_radius);
  bool setRadii(const double min_radius, const double max_radius);

  void setNormalDistanceWeightFromMap(const SingleClassParameterMap&);
  void setNormalDistanceWeightFromString(const std::string&);

  void setDistanceThresholdFromMap(const SingleClassParameterMap&);
  void setDistanceThresholdFromString(const std::string&);

  void setNumNearestNeighborsFromMap(const SingleClassParameterMap&);
  void setNumNearestNeighborsFromString(const std::string&);

  void setMaxIterationsFromMap(const SingleClassParameterMap&);
  void setMaxIterationsFromString(const std::string&);

  void setOptimizeCoefficientsFromMap(const SingleClassParameterMap&);
  void setOptimizeCoefficientsFromString(const std::string&);
};


std::ostream& operator<<(std::ostream& os, const Shape& shape);
std::istream& operator>>(std::istream& is, Shape& shape);

} // shape_detector

#endif  // SHAPE_DETECTOR_OBSERVED_PARAMETER_MANAGER
