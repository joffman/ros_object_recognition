/** \file
 * \brief Implementation of the ParameterManager.
 */

// std.
#include <string>
#include <algorithm>  // min(), max()

// binary_detector.
#include <binary_detector/parameter_manager.h>


namespace {

template <typename T>
inline T truncateToRange(T val, T min, T max)
{
  return std::min(std::max(val, min), max);
}

} // anonymous ns


namespace binary_detector {

constexpr double ParameterManager::length_fraction_lower_limit;
constexpr double ParameterManager::length_fraction_upper_limit;

void ParameterManager::setObjectName(const std::string& name)
{
  object_name_ = name;
}

void ParameterManager::setMinLengthFraction(double val)
{
  min_length_fraction_ = truncateToRange(val, length_fraction_lower_limit,
                                         length_fraction_upper_limit);
}

void ParameterManager::setMaxLengthFraction(double val)
{
  max_length_fraction_ = truncateToRange(val, length_fraction_lower_limit,
                                         length_fraction_upper_limit);
}

} // binary_detector
