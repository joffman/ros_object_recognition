/** \file
 * \brief Implementation of the ObservedParameterManager class.
 */

// std.
#include <iostream>
#include <stdexcept>  // runtime_error
#include <string>
#include <sstream>

// Our headers.
#include <distance_filter/observed_parameter_manager.h>


namespace {

struct ConversionError {};


template <typename T>
std::string toString(const T& t)
{
  std::ostringstream oss;
  oss << t;
  return oss.str();
}


template <typename T>
T fromString(const std::string& s)
{
  // TODO: I think we would also have to write into std::ws to make sure
  //  that there is nothing more in the the istringstream.
  std::istringstream iss {s};
  T t;
  if (!(iss >> t))
    throw ConversionError{};
  return t;
}

} // anonymous ns


namespace distance_filter {

ObservedParameterManager::MultiClassParameterMap
ObservedParameterManager::createParameterMap() const
{
  MultiClassParameterMap param_map;
  SingleClassParameterMap& my_params {param_map["DistanceFilter"]};

  my_params["min_distance_"] = toString(minDistance());
  my_params["max_distance_"] = toString(maxDistance());

  return param_map;
}


void ObservedParameterManager::setParametersFromMap(
    const MultiClassParameterMap& parameters)
{
  const std::string class_name {"DistanceFilter"};
  auto it = parameters.find(class_name);
  if (it != parameters.end()) {
    const SingleClassParameterMap& my_params {it->second};
    setParametersFromSingleClassMap(my_params);
  }
  else
    std::cerr << "ObservedParameterManager::setParametersFromMap: "
        << "Cannot find DistanceFilter-parameters.\n";
}


void ObservedParameterManager::setParametersFromSingleClassMap(
    const SingleClassParameterMap& parameters)
{
  bool found {false};
  auto min_dist_it = parameters.find("min_distance_");
  if (min_dist_it != parameters.end()) {
    auto max_dist_it = parameters.find("max_distance_");
    if (max_dist_it != parameters.end()) {
      found = true;
      setDistancesFromStrings(min_dist_it->second, max_dist_it->second);
    }
  }
  if (!found)
    std::cerr << "ObservedParameterManager::setParametersFromSingleClassMap: "
        << "Cannot find min-distance or max-distance parameter.\n";
}


void ObservedParameterManager::setDistancesFromStrings(
    const std::string& min_dist_str, const std::string& max_dist_str)
try {
  double min_dist {fromString<double>(min_dist_str)};
  double max_dist {fromString<double>(max_dist_str)};

  if (!trySetDistances(min_dist, max_dist))
    std::cerr << "ObservedParameterManager::setDistancesFromStrings: "
        << "Cannot set given parameters (invalid values).\n";
}
catch(const ConversionError& e) {
  std::cerr << "ObservedParameterManager::setDistancesFromStrings: "
      << "Cannot convert min-distance or max-distance.\n";
}


bool ObservedParameterManager::trySetDistances(const double min_dist, const double max_dist)
{
  try {
    ParameterManager::setDistances(min_dist, max_dist);
  }
  catch (const std::runtime_error&) {
    return false;
  }

  notify();
  return true;
}


void ObservedParameterManager::setMinDistance(double val)
{
  ParameterManager::setMinDistance(val);
  notify();
}


void ObservedParameterManager::setMaxDistance(double val)
{
  ParameterManager::setMaxDistance(val);
  notify();
}

} // distance_filter
