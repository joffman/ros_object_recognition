/** \file
 * \brief Definition of the ParameterManager class.
 */

#ifndef DISTANCE_FILTER_PARAMETER_MANAGER_H
#define DISTANCE_FILTER_PARAMETER_MANAGER_H


namespace distance_filter {

/** \brief Class for maintaining the parameters of DistanceFilter objects.
 *
 * The parameters are the minimum and the maximum distance of points that
 * pass through the filter.
 */
class ParameterManager {
public:
  /** \brief Lower limit of the distance parameters. */
  static constexpr double distance_lower_limit {0.};
  /** \brief Upper limit of the distance parameters. */
  static constexpr double distance_upper_limit {10.};

  //
  // Getters & Setters.
  //
  /** \brief Returns the minimum-distance parameter. */
  double minDistance() const { return min_distance_; }
  /** \brief Sets the minimum-distance parameter.
   *
   * \note The value is truncated to the valid range, given by the
   *  distance_lower_limit and distance_upper_limit constants.
   */
  void setMinDistance(double);

  /** \brief Returns the maximum-distance parameter. */
  double maxDistance() const { return max_distance_; }
  /** \brief Sets the maximum-distance parameter.
   *
   * \note The value is truncated to the valid range, given by the
   *  distance_lower_limit and distance_upper_limit constants.
   */
  void setMaxDistance(double);

  /** Sets the minimum and the maximum distance parameters.
   *
   * \note The arguments have to fulfill the condition 'distance_lower_limit <=
   *  min_distance <= max_distance <= distance_upper_limit'.
   *  Otherwise a runtime_error is thrown.
   */
  void setDistances(double min_distance, double max_distance);

private:
  double min_distance_ {1.};
  double max_distance_ {1.5};
};

} // distance_filter

#endif  // DISTANCE_FILTER_PARAMETER_MANAGER_H
