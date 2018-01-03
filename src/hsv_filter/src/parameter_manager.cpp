/** \file
 * \brief Implementation of the ParameterManager class.
 */

// std.
#include <algorithm>  // min(), max()

// Headers of this project.
#include <hsv_filter/parameter_manager.h>


namespace {

inline int truncateToRange(int val, int min, int max)
{
  return std::min(std::max(val, min), max);
}

} // anonymous ns


namespace hsv_filter {

void ParameterManager::setHMin(int val)
{
  h_min_ = truncateToRange(val, h_lower_limit, h_upper_limit);
}

void ParameterManager::setHMax(int val)
{
  h_max_ = truncateToRange(val, h_lower_limit, h_upper_limit);
}

void ParameterManager::setSMin(int val)
{
  s_min_ = truncateToRange(val, s_lower_limit, s_upper_limit);
}

void ParameterManager::setSMax(int val)
{
  s_max_ = truncateToRange(val, s_lower_limit, s_upper_limit);
}

void ParameterManager::setVMin(int val)
{
  v_min_ = truncateToRange(val, v_lower_limit, v_upper_limit);
}

void ParameterManager::setVMax(int val)
{
  v_max_ = truncateToRange(val, v_lower_limit, v_upper_limit);
}

} // hsv_filter
