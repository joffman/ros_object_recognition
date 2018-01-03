/** \file
 * \brief main function for the object_painter node.
 */

// std.
#include <iostream>
#include <exception>

// object_painter.
#include <object_painter/object_painter.h>

using object_painter::ObjectPainter;
using namespace std;


int main(int argc, char** argv)
try {
  ros::init(argc, argv, "object_painter");
  object_painter::ObjectPainter p;

  ros::spin();
  return 0;
}
catch (const exception& e) {
  cerr << "std::exception: " << e.what() << '\n';
  return 1;
}
catch (...) {
  cerr << "Unknown exception caught.\n";
  return 1;
}
