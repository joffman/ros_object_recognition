/** \file
 * \brief Implementation of the ObservedParameterManager class.
 */

// std.
#include <iostream>
#include <string>
#include <sstream>

// Headers of this package.
#include <shape_detector/observed_parameter_manager.h>

using namespace std;


namespace {

struct ConversionError {};  // Should this inherit from std::exception?


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


namespace shape_detector {

namespace {

bool validRadii(const double min_radius, const double max_radius)
{
  return (ObservedParameterManager::radius_lower_limit <= min_radius &&
          min_radius <= max_radius &&
          max_radius <= ObservedParameterManager::radius_upper_limit);
}

} // anonymous ns


ObservedParameterManager::MultiClassParameterMap
ObservedParameterManager::createParameterMap() const
{
  MultiClassParameterMap param_map;
  SingleClassParameterMap& my_params {
    param_map["ShapeDetector"]
  };

  my_params["object_name_"] = objectName();
  my_params["shape_"] = toString(shape());
  my_params["min_radius_"] = toString(minRadius());
  my_params["max_radius_"] = toString(maxRadius());
  my_params["normal_distance_weight_"] = toString(normalDistanceWeight());
  my_params["distance_threshold_"] = toString(distanceThreshold());
  my_params["num_nearest_neighbors_"] = toString(numNearestNeighbors());
  my_params["max_iterations_"] = toString(maxIterations());
  my_params["optimize_coefficients_"] = toString(optimizeCoefficients());

  return param_map;
}


void ObservedParameterManager::setParametersFromMap(
    const MultiClassParameterMap& param_map)
{
  const string class_name {"ShapeDetector"};
  auto it = param_map.find(class_name);
  if (it != param_map.end()) {
    const SingleClassParameterMap& my_params {it->second};
    setParametersFromSingleClassMap(my_params);
  }
  else
    cerr << "ObservedParameterManager::setParametersFromMap: "
        << "Cannot find ShapeDetector-parameters.\n";
}


void ObservedParameterManager::setParametersFromSingleClassMap(
    const SingleClassParameterMap& param_map)
{
  setObjectNameFromMap(param_map);
  setShapeFromMap(param_map);
  setRadiiFromMap(param_map);
  setNormalDistanceWeightFromMap(param_map);
  setDistanceThresholdFromMap(param_map);
  setNumNearestNeighborsFromMap(param_map);
  setMaxIterationsFromMap(param_map);
  setOptimizeCoefficientsFromMap(param_map);
}


void ObservedParameterManager::setObjectNameFromMap(
    const SingleClassParameterMap& param_map)
{
  auto it = param_map.find("object_name_");
  if (it != param_map.end())
    setObjectName(it->second);
  else
    cerr << "ObservedParameterManager::setObjectNameFromMap: "
        << "Cannot find object-name.\n";
}


void ObservedParameterManager::setShapeFromMap(
    const SingleClassParameterMap& param_map)
{
  auto it = param_map.find("shape_");
  if (it != param_map.end())
    setShapeFromString(it->second);
  else
    cerr << "ObservedParameterManager::setShapeFromMap: "
        << "Cannot find shape.\n";
}


void ObservedParameterManager::setShapeFromString(const string& s)
try {
  setShape(fromString<Shape>(s));
}
catch(const ConversionError& e) {
  cerr << "ObservedParameterManager::setShapeFromString: "
      << "Conversion failed.\n";
}


void ObservedParameterManager::setShape(const Shape val)
{
  ParameterManager::setShape(val);
  notify();
}


void ObservedParameterManager::setRadiiFromMap(const SingleClassParameterMap& param_map)
{
  bool found {false};
  auto min_radius_it = param_map.find("min_radius_");
  if (min_radius_it != param_map.end()) {
    auto max_radius_it = param_map.find("max_radius_");
    if (max_radius_it != param_map.end()) {
      found = true;
      setRadiiFromStrings(min_radius_it->second, max_radius_it->second);
    }
  }
  if (!found)
    cerr << "ObservedParameterManager::setRadiiFromMap: "
        << "Cannot find min- or max-radius.\n";
}


void ObservedParameterManager::setRadiiFromStrings(
    const string& min_radius_str, const string& max_radius_str)
try {
  double min_radius {fromString<double>(min_radius_str)};
  double max_radius {fromString<double>(max_radius_str)};

  if (!setRadii(min_radius, max_radius))
    cerr << "ObservedParameterManager::setRadiiFromStrings: "
        << "Cannot set given parameters (invalid values).\n";
}
catch(const ConversionError& e) {
  cerr << "ObservedParameterManager::setRadiiFromStrings: "
      << "Cannot convert min- or max-radius.\n";
}


bool ObservedParameterManager::setRadii(const double min_radius,
                                        const double max_radius)
{
  if (validRadii(min_radius, max_radius)) {
    setMinRadius(min_radius);
    setMaxRadius(max_radius);

    return true;
  }
  else
    return false;
}


void ObservedParameterManager::setMinRadius(const double val)
{
  ParameterManager::setMinRadius(val);
  notify();
}


void ObservedParameterManager::setMaxRadius(const double val)
{
  ParameterManager::setMaxRadius(val);
  notify();
}


void ObservedParameterManager::setNormalDistanceWeightFromMap(
    const SingleClassParameterMap& param_map)
{
  auto it = param_map.find("normal_distance_weight_");
  if (it != param_map.end())
    setNormalDistanceWeightFromString(it->second);
  else
    cerr << "ObservedParameterManager::setNormalDistanceWeightFromMap: "
        << "Cannot find normal-distance-weight.\n";
}


void ObservedParameterManager::setNormalDistanceWeightFromString(const string& s)
try {
  setNormalDistanceWeight(fromString<double>(s));
}
catch(const ConversionError& e) {
  cerr << "ObservedParameterManager::setNormalDistanceWeightFromString: "
      << "Conversion failed.\n";
}


void ObservedParameterManager::setNormalDistanceWeight(const double val)
{
  ParameterManager::setNormalDistanceWeight(val);
  notify();
}


void ObservedParameterManager::setDistanceThresholdFromMap(const SingleClassParameterMap& param_map)
{
  auto it = param_map.find("distance_threshold_");
  if (it != param_map.end())
    setDistanceThresholdFromString(it->second);
  else
    cerr << "ObservedParameterManager::setDistanceThresholdFromMap: "
        << "Cannot find distance-threshold.\n";
}


void ObservedParameterManager::setDistanceThresholdFromString(const string& s)
try {
  setDistanceThreshold(fromString<double>(s));
}
catch(const ConversionError& e) {
  cerr << "ObservedParameterManager::setDistanceThresholdFromString: "
      << "Conversion failed.\n";
}


void ObservedParameterManager::setDistanceThreshold(const double val)
{
  ParameterManager::setDistanceThreshold(val);
  notify();
}


void ObservedParameterManager::setNumNearestNeighborsFromMap(
    const SingleClassParameterMap& param_map)
{
  auto it = param_map.find("num_nearest_neighbors_");
  if (it != param_map.end())
    setNumNearestNeighborsFromString(it->second);
  else
    cerr << "ObservedParameterManager::setNumNearestNeighborsFromMap: "
        << "Cannot find number-nearest-neighbors.\n";
}


void ObservedParameterManager::setNumNearestNeighborsFromString(const string& s)
try {
  setNumNearestNeighbors(fromString<int>(s));
}
catch(const ConversionError& e) {
  cerr << "ObservedParameterManager::setNumNearestNeighborsFromString: "
      << "Conversion failed.\n";
}


void ObservedParameterManager::setNumNearestNeighbors(const int val)
{
  ParameterManager::setNumNearestNeighbors(val);
  notify();
}


void ObservedParameterManager::setMaxIterationsFromMap(const SingleClassParameterMap& param_map)
{
  auto it = param_map.find("max_iterations_");
  if (it != param_map.end())
    setMaxIterationsFromString(it->second);
  else
    cerr << "ObservedParameterManager::setMaxIterationsFromMap: "
        << "Cannot find max-iterations.\n";
}


void ObservedParameterManager::setMaxIterationsFromString(const string& s)
try {
  setMaxIterations(fromString<int>(s));
}
catch(const ConversionError& e) {
  cerr << "ObservedParameterManager::setMaxIterationsFromString: "
      << "Conversion failed.\n";
}


void ObservedParameterManager::setMaxIterations(const int val)
{
  ParameterManager::setMaxIterations(val);
  notify();
}


void ObservedParameterManager::setOptimizeCoefficientsFromMap(const SingleClassParameterMap& param_map)
{
  auto it = param_map.find("optimize_coefficients_");
  if (it != param_map.end())
    setOptimizeCoefficientsFromString(it->second);
  else
    cerr << "ObservedParameterManager::setOptimizeCoefficientsFromMap: "
        << "Cannot find optimize-coefficients-parameter.\n";
}


void ObservedParameterManager::setOptimizeCoefficientsFromString(const string& s)
try {
  setOptimizeCoefficients(fromString<bool>(s));
}
catch(const ConversionError& e) {
  cerr << "ObservedParameterManager::setOptimizeCoefficientsFromString: "
      << "Conversion failed.\n";
}


void ObservedParameterManager::setOptimizeCoefficients(const bool val)
{
  ParameterManager::setOptimizeCoefficients(val);
  notify();
}


////////////////////////////////////////////////////////////////////////////////
//
// Shape operators.
//
// TODO: Should these be defined in the ParameterManager class?!


ostream& operator<<(ostream& os, const Shape& shape)
{
  return os << static_cast<int>(shape);
}


istream& operator>>(istream& is, Shape& shape)
{
  int i;
  if (!(is>>i))
    return is;

  switch (i) {
  case static_cast<int>(Shape::sphere):
  case static_cast<int>(Shape::cylinder):
    shape = static_cast<Shape>(i);
    break;
  default:
    is.clear(ios_base::failbit);
    break;
  }
  return is;
}


void ObservedParameterManager::setObjectName(const string& name)
{
  ParameterManager::setObjectName(name);
  notify();
}

} // shape_detector
