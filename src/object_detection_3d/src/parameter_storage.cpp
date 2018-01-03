/** \file
 * \brief Implementation of the ParameterStorage class.
 */

// std.
#include <string>
#include <fstream>
#include <regex>

// Our headers.
#include <object_detection_3d/parametrizable.h>
#include <object_detection_3d/parameter_storage.h>

using namespace std;


namespace object_detection_3d {

void ParameterStorage::addParametrizable(Parametrizable* obj_ptr)
{
  parametrizables_.push_back(obj_ptr);
}


bool ParameterStorage::saveParameters(const string& filename) const
{
  ofstream ofs {filename};
  if (!ofs)
    return false;

  for (auto obj : parametrizables_) {
    MultiClassParameterMap param_map {
      obj->createParameterMap()
    };

    for (const auto& class_pair : param_map) {  // normally just one class per obj
      const auto& current_class_map = class_pair.second;
      for (const auto& param_pair : current_class_map)
        ofs << class_pair.first << '/'       // classname
            << param_pair.first << " : "     // param-name
            << param_pair.second << '\n';    // param-value
    }

    const string seperator {"----"};
    ofs << seperator << '\n';
  }
  return true;
}


bool ParameterStorage::loadParameters(const string& filename) const
{
  // Open file.
  ifstream ifs {filename};
  if (!ifs)
    return false;

  // Read each line and create map from all parameters found.
  MultiClassParameterMap param_map;
  for (string line; getline(ifs, line); ) {
    regex pattern {R"(^\s*(\w+)\s*/\s*(\w+)\s*:\s*(.*)$)"};
      // e.g. "  DistanceFilter  /min_distance_ :  0.5  "
    smatch matches;
    if (regex_match(line, matches, pattern))
      param_map[matches[1]][matches[2]] = matches[3];
  }

  // Give the map to all parametrizables_.
  for (Parametrizable* obj : parametrizables_)
    obj->setParametersFromMap(param_map);

  return true;
}

} // object_detection_3d
