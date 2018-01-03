/** \file
 * \brief Implementation of the ParameterStorage class.
 */

// std.
#include <string>

// OpenCV.
#include <opencv2/core/core.hpp>

// object_detection_*.
#include <object_detection_2d/parametrizable.h>
#include <object_detection_2d/parameter_storage.h>

using namespace std;


namespace object_detection_2d {

void ParameterStorage::addParametrizable(Parametrizable* obj_ptr)
{
  parametrizables_.push_back(obj_ptr);
}


bool ParameterStorage::saveParameters(const string& filename) const
try {
  cv::FileStorage storage {filename, cv::FileStorage::WRITE};
  if (storage.isOpened()) {
    for (auto& obj : parametrizables_)
      obj->writeParametersToStorage(storage);
    return true;
  }
  return false;
}
catch (const cv::Exception& e) {  // e.g. when a parametrizable uses invalid keys
  return false;
}


bool ParameterStorage::loadParameters(const string& filename) const
try {
  cv::FileStorage storage {filename, cv::FileStorage::READ};
  if (storage.isOpened()) {
    for (Parametrizable* obj : parametrizables_)
      obj->setParametersFromStorage(storage);
    return true;
  }
  return false;
}
catch (const cv::Exception& e) {
  return false;
}

} // object_detection_2d
