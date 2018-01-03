/** \file
 * \brief Implementation of the ParameterManager class.
 */

// std.
#include <stdexcept>  // runtime_error
#include <algorithm>  // min(), max()

// Headers of this project.
#include <morphology_filter/parameter_manager.h>


namespace {

inline int truncateToRange(int val, int min, int max)
{
  return std::min(std::max(val, min), max);
}

} // anonymous ns


namespace morphology_filter {

void ParameterManager::setOperation(const Operation op)
{
  switch (op) {
  case Operation::erode:
  case Operation::dilate:
  case Operation::open:
  case Operation::close:
    break;
  default:
    throw std::runtime_error
    {"Invalid value passed to ParameterManager::setOperation"};
  }

  operation_ = op;
}


void ParameterManager::setNumIterations(int val)
{
  num_iterations_ = truncateToRange(val, min_num_iterations, max_num_iterations);
}

} // morphology_filter
