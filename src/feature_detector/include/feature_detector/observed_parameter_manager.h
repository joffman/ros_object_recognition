/** \file
 * \brief Definition of the ObservedParameterManager class.
 */

#ifndef FEATURE_DETECTOR_OBSERVED_PARAMETER_MANAGER_H
#define FEATURE_DETECTOR_OBSERVED_PARAMETER_MANAGER_H

// std.
#include <string>

// Headers of this project.
#include <object_detection/subject.h>
#include <object_detection_2d/parametrizable.h>
#include "parameter_manager.h"


namespace cv {
class FileStorage;
}
    
namespace feature_detector {

/** \brief Observable parameter-manager for a FeatureDetector object.
 *
 * This ParameterManager inherits from object_detection:Subject,
 * making it possible to register observers for it.
 * It also inherits from object_detection_2d::Parametrizable.
 *
 * Using this parameter-manager makes it possible to access the parameters
 * of a FeatureDetector via the object_detection_2d::GUI.
 */
class ObservedParameterManager : public ParameterManager,
    public object_detection::Subject,
    public object_detection_2d::Parametrizable {
public:
  //
  // Parametrizable interface.
  //

  /** \brief Writes all parameters to the given FileStorage object. */
  void writeParametersToStorage(cv::FileStorage&) const override;

  /** \brief Reads all parameters from the given FileStorage object
   *  and sets its parameters accordingly.
   */
  void setParametersFromStorage(cv::FileStorage&) override;

  //
  // Setters.
  //

  /** \brief Sets the name of detected objects. */
  void setObjectName(const std::string&);
  /** \brief Sets the hessian-threshold parameter. */
  void setHessianThreshold(double);
  /** \brief Sets the number-octaves parameter. */
  void setNumOctaves(int);
  /** \brief Sets the number-octave-layers parameter. */
  void setNumOctaveLayers(int);
  /** \brief Sets the extended parameter. */
  void setExtended(bool);
  /** \brief Sets the upright parameter. */
  void setUpright(bool);
  /** \brief Sets the norm-type parameter. */
  void setNormType(NormType);
  /** \brief Sets the cross-check parameter. */
  void setCrossCheck(bool);
  /** \brief Sets the match-distance parameter. */
  void setMatchDistanceThreshold(double);
  /** \brief Sets the ransac-reprojection-threhold parameter. */
  void setRansacReprojThreshold(double);
};

} // feature_detector

#endif  // FEATURE_DETECTOR_OBSERVED_PARAMETER_MANAGER_H
