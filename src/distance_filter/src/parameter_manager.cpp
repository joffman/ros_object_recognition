/** \file
 * \brief Implementation of the ParameterManager class.
 */

// std.
#include <stdexcept>  // runtime_error
#include <algorithm>  // min(), max()

// Our headers.
#include <distance_filter/parameter_manager.h>


namespace {

template <typename T>
T truncateToRange(T val, T min, T max)
{
  return std::min(std::max(val, min), max);
}

} // anonymous ns


namespace distance_filter {

void ParameterManager::setMinDistance(double val)
{
  min_distance_ = truncateToRange(val, distance_lower_limit,
                                  distance_upper_limit);
}


void ParameterManager::setMaxDistance(double val)
{
  max_distance_ = truncateToRange(val, distance_lower_limit, distance_upper_limit);
}


void ParameterManager::setDistances(double min_distance, double max_distance)
{
  if (max_distance < min_distance || min_distance < distance_lower_limit ||
      distance_upper_limit < max_distance)
    throw std::runtime_error
    {"ParameterManager::setDistances: invalid argument values"};

  min_distance_ = min_distance;
  max_distance_ = max_distance;
}

} // distance_filter
