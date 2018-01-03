/** \file
 * \brief Implementation of the ParameterManager class.
 */

// std.
#include <string>
#include <algorithm>  // min(), max()

// Headers of this package.
#include <shape_detector/parameter_manager.h>
#include <shape_detector/shape_enum.h>

using namespace std;


namespace {

template <typename T>
T truncateToRange(T val, T lower_limit, T upper_limit)
{
  return min(max(val, lower_limit), upper_limit);
}

} // anonymous ns


namespace shape_detector {

void ParameterManager::setObjectName(const string& name)
{
  object_name_ = name;
}


void ParameterManager::setShape(Shape s)
{
  switch (s) {
  case Shape::sphere:
  case Shape::cylinder:
    break;
  default:
    throw runtime_error {"ParameterManager::setShape: invalid argument"};
  }

  shape_ = s;
}


void ParameterManager::setMinRadius(double val)
{
  min_radius_ = truncateToRange(val, radius_lower_limit, radius_upper_limit);
}


void ParameterManager::setMaxRadius(double val)
{
  max_radius_ = truncateToRange(val, radius_lower_limit, radius_upper_limit);
}


void ParameterManager::setNormalDistanceWeight(double val)
{
  normal_distance_weight_ =
      truncateToRange(val, normal_distance_weight_lower_limit,
                      normal_distance_weight_upper_limit);
}


void ParameterManager::setDistanceThreshold(double val)
{
  distance_threshold_ = truncateToRange(val, distance_threshold_lower_limit,
                                        distance_threshold_upper_limit);
}


void ParameterManager::setNumNearestNeighbors(int val)
{
  num_nearest_neighbors_ = truncateToRange(val, num_nearest_neighbors_lower_limit,
                                           num_nearest_neighbors_upper_limit);
}


void ParameterManager::setMaxIterations(int val)
{
  max_iterations_ = truncateToRange(val, max_iterations_lower_limit,
                                    max_iterations_upper_limit);
}


void ParameterManager::setOptimizeCoefficients(bool val)
{
  optimize_coefficients_ = val;
}

} // shape_detector
