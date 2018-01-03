/** \file
 * \brief Implementation of general ObjectComputer-related functionality.
 */

// std.
#include <utility>  	// forward
#include <memory>   	// unique_ptr
#include <stdexcept>  // runtime_error

// Headers of this package.
#include <shape_detector/shape_enum.h>
#include <shape_detector/object_computer.h>

using namespace std;


namespace {

template<typename T, typename... Args>
unique_ptr<T> make_unique(Args&&... args)
{
  return unique_ptr<T>{new T{forward<Args>(args)...}};
} // just because it isn't available in C++11

} // anonymous ns


namespace shape_detector {

unique_ptr<ObjectComputer> createObjectComputerForShape(Shape shape)
{
  switch (shape) {
  case Shape::cylinder:
    return make_unique<CylinderComputer>();
  case Shape::sphere:
    return make_unique<SphereComputer>();
  }

  throw runtime_error {"createObjectComputerForShape: invalid shape"};
}

} // shape_detector
