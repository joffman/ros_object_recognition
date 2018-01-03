/** \file shape_enum.h
 * \brief Definition of the Shape enumeration.
 */

#ifndef SHAPE_DETECTOR_SHAPE_ENUM_H
#define SHAPE_DETECTOR_SHAPE_ENUM_H

#include <pcl/sample_consensus/model_types.h>


namespace shape_detector {

/** \brief Enumeration for shapes that can be detected by the ShapeDetector.
 */
enum class Shape {
  sphere = pcl::SACMODEL_SPHERE,
  cylinder = pcl::SACMODEL_CYLINDER
};

} // shape_detector

#endif  // SHAPE_DETECTOR_SHAPE_ENUM_H
