/** \file
 * \brief Definition of the ParameterManager class.
 */

/* TODO: Maybe it's better to use delegation instead of inheritance.
 *  A way to do this would be to give the HSVFilter a ParameterManager as a
 *  member (not just a reference) and to give the ParameterManager object
 *  a member that is the only one that is allowed to set the parameters,
 *  therefore encapsulating how the parameters are set (from a reconfigure-
 *  callback or through some request of an observer).
 *  The HSVFilter would than provide a method for setting this setter-object
 *  in the filter's parameter-manager object.
 */

#ifndef HSV_FILTER_PARAMETER_MANAGER_H
#define HSV_FILTER_PARAMETER_MANAGER_H


namespace hsv_filter {

/** \brief Parameter-manager for BinaryDetector objects.
 *
 * A ParameterManager object manages the parameters of
 * a BinaryDetector object and provides getter- and setter-methods for
 * accessing those parameters.
 */
class ParameterManager {
public:
  //
  // Limits / Consts.
  //
  /** \brief Lower limit of the h-min and h-max parameters. */
  static constexpr int h_lower_limit {0};
  /** \brief Upper limit of the h-min and h-max parameters. */
  static constexpr int h_upper_limit {179};

  /** \brief Lower limit of the s-min and s-max parameters. */
  static constexpr int s_lower_limit {0};
  /** \brief Upper limit of the s-min and s-max parameters. */
  static constexpr int s_upper_limit {255};

  /** \brief Lower limit of the v-min and v-max parameters. */
  static constexpr int v_lower_limit {0};
  /** \brief Upper limit of the v-min and v-max parameters. */
  static constexpr int v_upper_limit {255};

  //
  // Getters.
  //
  int hMin() const { return h_min_; }
  int hMax() const { return h_max_; }

  int sMin() const { return s_min_; }
  int sMax() const { return s_max_; }

  int vMin() const { return v_min_; }
  int vMax() const { return v_max_; }

  //
  // Setters. Maybe make these protected.
  //
  void setHMin(int val);
  void setHMax(int val);

  void setSMin(int val);
  void setSMax(int val);

  void setVMin(int val);
  void setVMax(int val);

private:
  // Parameters.
  int h_min_ {h_lower_limit};
  int h_max_ {h_upper_limit};

  int s_min_ {s_lower_limit};
  int s_max_ {s_upper_limit};

  int v_min_ {v_lower_limit};
  int v_max_ {v_upper_limit};
};

} // hsv_filter

#endif  // HSV_FILTER_PARAMETER_MANAGER_H
