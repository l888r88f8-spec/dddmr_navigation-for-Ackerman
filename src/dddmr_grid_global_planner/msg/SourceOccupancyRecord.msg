# One frozen occupied or cleared point record in the map frame.
#
# point is the XY/Z observation location; min_z/max_z describe the observed
# obstacle slab at that location. A point-only cloud has min_z == max_z ==
# point.z and must not be interpreted as free vertical extent.
geometry_msgs/Point point
float64 min_z
float64 max_z
