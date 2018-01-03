string name

# We use a polygon to represent the location of the object in pixel space.
# This gives us a very general description of the object and many possibilities
# for describing the contour of the object.
# Polygon is also predefined in the geometry_msgs package. We set the
# z-coordinate of the points in the polygon to zero since we work in pixel
# space.
geometry_msgs/Polygon polygon
