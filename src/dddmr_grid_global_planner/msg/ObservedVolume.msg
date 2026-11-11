# Explicit vertical evidence for one observed source volume.
#
# Contract:
#  - known == false means no observation covers the volume. It never means free.
#  - free_space == true declares that the whole min/max box was observed FREE
#    (except any same-source occupied slab enumerated in occupied/the terrain
#    association, which is never cleared through).
#  - free_space == false declares an observed OCCUPIED extent only. It is never
#    rasterized as free space; it remains occupied/unknown evidence.
#  - headroom_known is independent of both flags because the occupied extent may
#    be known while the free clearance above it is not.
geometry_msgs/Point min
geometry_msgs/Point max
bool known
bool free_space
float64 headroom_m
bool headroom_known
