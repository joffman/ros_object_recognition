/** \file
 * \brief Definition of the ParameterManager class.
 */

#ifndef SHAPE_DETECTOR_PARAMETER_MANAGER_H
#define SHAPE_DETECTOR_PARAMETER_MANAGER_H

// std.
#include <string>

// Headers of this package.
#include "shape_enum.h"


namespace shape_detector {

/** \brief Parameter-manager for ShapeDetector objects.
 *
 * ParameterManager objects manage the parameters of
 * ShapeDetector objects and provide getter- and setter-methods for accessing
 * those parameters.
 */
class ParameterManager {
public:
  // static consts.
  static constexpr double radius_lower_limit {0.01};
  static constexpr double radius_upper_limit {1.};

  static constexpr double normal_distance_weight_lower_limit {0.001};
  static constexpr double normal_distance_weight_upper_limit {1.};

  static constexpr double distance_threshold_lower_limit {0.05};
  static constexpr double distance_threshold_upper_limit {1.};

  static constexpr int num_nearest_neighbors_lower_limit {5};
  static constexpr int num_nearest_neighbors_upper_limit {100};

  static constexpr int max_iterations_lower_limit {1};
  static constexpr int max_iterations_upper_limit {20000};

  // Getters & Setters.
  std::string objectName() const { return object_name_; }
  void setObjectName(const std::string&);

  Shape shape() const { return shape_; }
  void setShape(Shape);

  double minRadius() const { return min_radius_; }
  void setMinRadius(double);

  double maxRadius() const { return max_radius_; }
  void setMaxRadius(double);

  double normalDistanceWeight() const { return normal_distance_weight_; }
  void setNormalDistanceWeight(double);

  double distanceThreshold() const { return distance_threshold_; }
  void setDistanceThreshold(double);

  int numNearestNeighbors() const { return num_nearest_neighbors_; }
  void setNumNearestNeighbors(int);

  int maxIterations() const { return max_iterations_; }
  void setMaxIterations(int);

  bool optimizeCoefficients() const { return optimize_coefficients_; }
  void setOptimizeCoefficients(bool);

private:
  // Parameters.
  std::string object_name_;

  Shape shape_ {Shape::cylinder};
  double min_radius_ {0.1};
  double max_radius_ {0.2};
  double normal_distance_weight_ {0.1};
  double distance_threshold_ {0.05};
  int num_nearest_neighbors_{50};
  int max_iterations_ {100};
  bool optimize_coefficients_ {true};
};

} // shape_detector

#endif  // SHAPE_DETECTOR_PARAMETER_MANAGER_H
