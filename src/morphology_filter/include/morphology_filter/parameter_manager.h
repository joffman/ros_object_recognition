/** \file
 * \brief Definition of the ParameterManager class.
 */

/* TODO: Maybe it's better to make the ParameterManager class non-abstract.
 *  A way to do this would be to give the MorphologyFilter a ParameterManager as a
 *  member (not just a reference) and to give the ParameterManager object
 *  a member that is the only one that is allowed to set the parameters,
 *  therefore encapsulating how the parameters are set (from a reconfigure-
 *  callback or through some request of an observer).
 *  The MorphologyFilter would than provide a method for setting this setter-object
 *  in the filter's parameter-manager object.
 */

#ifndef MORPHOLOGY_FILTER_PARAMETER_MANAGER_H
#define MORPHOLOGY_FILTER_PARAMETER_MANAGER_H

#include <cv_bridge/cv_bridge.h>


namespace morphology_filter {

/** \brief Parameter-manager for BinaryDetector objects.
 *
 * A ParameterManager object manages the parameters of
 * a BinaryDetector object and provides getter- and setter-methods for
 * accessing those parameters.
 */
class ParameterManager {
public:
  /** \brief Enumeration for values that can be assigned to the operation
   *  parameter.
   */
  enum class Operation {
    erode   = cv::MORPH_ERODE,
    dilate  = cv::MORPH_DILATE,
    open    = cv::MORPH_OPEN,
    close   = cv::MORPH_CLOSE
  };

  //
  // Limits / Consts.
  //
  /** \brief Lower limit of the number-iterations parameter. */
  static constexpr int min_num_iterations {0};
  /** \brief Upper limit of the number-iterations parameter. */
  static constexpr int max_num_iterations {10};

  //
  // Getters & Setters.
  //

  /** \brief Returns the operation parameter, i.e. the morphological
   *  transformation (e.g. 'dilate') that is applied.
   */
  Operation operation() const { return operation_; }
  /** \brief Sets the operation parameter. */
  void setOperation(Operation);

  /** \brief Returns the number-iterations parameter, i.e. the
   *  the number of iterations used in the transformation.
   */
  int numIterations() const { return num_iterations_; }
  /** \brief Sets the number-iterations parameter. */
  void setNumIterations(int);

private:
  // Parameters.
  Operation operation_ {Operation::open};
  int num_iterations_ {1};
};

} // morphology_filter

#endif  // MORPHOLOGY_FILTER_PARAMETER_MANAGER_H
