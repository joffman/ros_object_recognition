/** \file
 * \brief Definition of the ParameterManager class.
 */

#ifndef BINARY_DETECTOR_PARAMETER_MANAGER_H
#define BINARY_DETECTOR_PARAMETER_MANAGER_H

// std.
#include <string>


namespace binary_detector {

/** \brief Parameter-manager for BinaryDetector objects.
 *
 * A ParameterManager object manages the parameters of
 * a BinaryDetector object and provides getter- and setter-methods for
 * accessing those parameters.
 */
class ParameterManager {
public:
  /** \brief Lower limit for the length-fraction-parameters. */
  static constexpr double length_fraction_lower_limit {1. / 200.};
  /** \brief Upper limit for the length-fraction-parameters. */
  static constexpr double length_fraction_upper_limit {3.};

  //
  // Getters.
  //

  /** \brief Returns the name that is given to detected objects. */
  std::string objectName() const { return object_name_; }

  /** \brief Returns the value of the minimum-length-fraction parameter. */
  double minLengthFraction() const { return min_length_fraction_; }
  /** \brief Returns the value of the maximum-length-fraction parameter. */
  double maxLengthFraction() const { return max_length_fraction_; }

  //
  // Setters.
  //

  /** \brief Sets the name that is given to detected objects. */
  void setObjectName(const std::string&);

  /** \brief Sets the value of the minimum-length-fraction parameter. */
  void setMinLengthFraction(double);
  /** \brief Sets the value of the maximum-length-fraction parameter. */
  void setMaxLengthFraction(double);
 
private:
  std::string object_name_;

  double min_length_fraction_ {1. / 20.};
  double max_length_fraction_ {1. / 3.};
};

} // binary_detector

#endif  // BINARY_DETECTOR_PARAMETER_MANAGER_H
