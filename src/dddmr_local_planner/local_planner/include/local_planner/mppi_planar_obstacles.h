#ifndef LOCAL_PLANNER_MPPI_PLANAR_OBSTACLES_H_
#define LOCAL_PLANNER_MPPI_PLANAR_OBSTACLES_H_

#include <cfloat>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Pure planar geometry helpers are callable from CUDA device kernels. Plain
// C++ translation units see an empty macro.
#ifndef MPPI_PLANAR_HD
#if defined(__CUDACC__)
#define MPPI_PLANAR_HD __host__ __device__
#else
#define MPPI_PLANAR_HD
#endif
#endif

namespace local_planner
{

// Height band of the frozen base_link plane, inclusive on both edges. These
// live in this device-safe header (no ROS dependency, plain double math) so a
// future CUDA translation unit can filter with the identical rule.
constexpr double kMppiPlanarMinObstacleZ = -0.20;
constexpr double kMppiPlanarMaxObstacleZ = 0.60;

// Shared height-band predicate: both edges are inclusive, and a NaN or
// infinite height is outside the band.
MPPI_PLANAR_HD inline bool mppiPlanarObstacleBandContains(const double plane_z)
{
  return plane_z >= kMppiPlanarMinObstacleZ &&
         plane_z <= kMppiPlanarMaxObstacleZ;
}

// Boundary inclusive, conservative 1e-6 m tolerance shared by the CPU and CUDA
// rectangle test. It matches the legacy 3-D cuboid tolerance so a rotated
// boundary point cannot flip between implementations through 1-ulp libm
// differences. Points within the tolerance count as colliding and their
// clearance saturates at 0.
constexpr double kMppiPlanarBoundaryTolerance = 1e-6;

// Cell-range safety margin in quotient units (distance from the grid origin
// measured in cells). Scaling with the grid position keeps far-origin maps
// correct; a margin based on the absolute pose is not sufficient there.
constexpr double kMppiPlanarCellQueryMargin = 64.0;

// Flattened XY envelope of the configured body cuboid in the frozen
// base_link plane frame. Queries rotate this rectangle by a candidate yaw and
// translate it to the candidate body origin.
struct MppiPlanarBodyRect
{
  bool valid = false;
  double x_min = 0.0;
  double x_max = 0.0;
  double y_min = 0.0;
  double y_max = 0.0;
};

// Shared CPU/CUDA geometry. The point is in the plane; the body's base_link
// origin is at (center_x, center_y) with heading yaw. dx/dy are per-axis gaps
// of the point from the oriented rectangle and are never negative. inside()
// is boundary inclusive with the conservative tolerance above; clearance() is
// clamped to 0 for points inside that tolerance and is hypot(dx, dy)
// otherwise.
struct MppiPlanarBoxDistance
{
  double dx = 0.0;
  double dy = 0.0;

  MPPI_PLANAR_HD bool inside() const
  {
    return dx <= kMppiPlanarBoundaryTolerance &&
           dy <= kMppiPlanarBoundaryTolerance;
  }

  MPPI_PLANAR_HD double clearance() const
  {
    return inside() ? 0.0 : ::hypot(dx, dy);
  }
};

// Precomputed heading trig for the per-point hot loops.
struct MppiPlanarBodyTrig
{
  double cos_yaw = 1.0;
  double sin_yaw = 0.0;
};

MPPI_PLANAR_HD inline MppiPlanarBodyTrig mppiPlanarBodyTrig(const double yaw)
{
  MppiPlanarBodyTrig trig;
  trig.cos_yaw = ::cos(yaw);
  trig.sin_yaw = ::sin(yaw);
  return trig;
}

MPPI_PLANAR_HD inline MppiPlanarBoxDistance mppiPlanarPointToBody(
  const MppiPlanarBodyRect & body, const double center_x,
  const double center_y, const MppiPlanarBodyTrig & trig,
  const double point_x, const double point_y)
{
  const double delta_x = point_x - center_x;
  const double delta_y = point_y - center_y;
  // (u, v) = Rz(-yaw) * (point - body origin).
  const double u = trig.cos_yaw * delta_x + trig.sin_yaw * delta_y;
  const double v = -trig.sin_yaw * delta_x + trig.cos_yaw * delta_y;

  MppiPlanarBoxDistance distance;
  distance.dx = body.x_min - u;
  if(distance.dx < 0.0){
    distance.dx = 0.0;
  }
  const double positive_x = u - body.x_max;
  if(positive_x > distance.dx){
    distance.dx = positive_x;
  }
  distance.dy = body.y_min - v;
  if(distance.dy < 0.0){
    distance.dy = 0.0;
  }
  const double positive_y = v - body.y_max;
  if(positive_y > distance.dy){
    distance.dy = positive_y;
  }
  return distance;
}

// Convenience wrapper for tests and one-off host calls.
MPPI_PLANAR_HD inline MppiPlanarBoxDistance mppiPlanarPointToBody(
  const MppiPlanarBodyRect & body, const double center_x,
  const double center_y, const double yaw, const double point_x,
  const double point_y)
{
  return mppiPlanarPointToBody(
    body, center_x, center_y, mppiPlanarBodyTrig(yaw), point_x, point_y);
}

// Axis-aligned bounds of the oriented body rectangle. local_expansion widens
// the rectangle along its own x/y axes before rotation (the 1e-6
// classification tolerance); world_expansion widens the resulting
// plane-axis bounds (the clearance radius). Both are 0 when not needed.
struct MppiPlanarAabb
{
  double minimum_x = 0.0;
  double maximum_x = 0.0;
  double minimum_y = 0.0;
  double maximum_y = 0.0;
};

MPPI_PLANAR_HD inline MppiPlanarAabb mppiPlanarBodyAabb(
  const MppiPlanarBodyRect & body, const double center_x,
  const double center_y, const MppiPlanarBodyTrig & trig,
  const double local_expansion, const double world_expansion)
{
  const double half_x =
    0.5 * (body.x_max - body.x_min) + local_expansion;
  const double half_y =
    0.5 * (body.y_max - body.y_min) + local_expansion;
  const double offset_x = 0.5 * (body.x_max + body.x_min);
  const double offset_y = 0.5 * (body.y_max + body.y_min);
  const double aabb_center_x =
    center_x + trig.cos_yaw * offset_x - trig.sin_yaw * offset_y;
  const double aabb_center_y =
    center_y + trig.sin_yaw * offset_x + trig.cos_yaw * offset_y;
  const double aabb_half_x =
    ::fabs(trig.cos_yaw) * half_x + ::fabs(trig.sin_yaw) * half_y;
  const double aabb_half_y =
    ::fabs(trig.sin_yaw) * half_x + ::fabs(trig.cos_yaw) * half_y;

  MppiPlanarAabb aabb;
  aabb.minimum_x = aabb_center_x - aabb_half_x - world_expansion;
  aabb.maximum_x = aabb_center_x + aabb_half_x + world_expansion;
  aabb.minimum_y = aabb_center_y - aabb_half_y - world_expansion;
  aabb.maximum_y = aabb_center_y + aabb_half_y + world_expansion;
  return aabb;
}

struct MppiPlanarObstacleGridConfig
{
  double cell_size = 0.20;
};

struct MppiPlanarDistanceFieldView
{
  const float * distances = nullptr;
  std::size_t width = 0;
  std::size_t height = 0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double resolution = 0.0;
};

struct MppiPlanarDistanceField
{
  std::vector<float> distances;
  std::size_t width = 0;
  std::size_t height = 0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double resolution = 0.0;
};

// Approximate clearance for scoring from three circles covering the rectangle.
// Within field coverage, the estimate supplies the score at every distance. Collision remains a
// separate check against the original points and the oriented rectangle.
MPPI_PLANAR_HD inline bool mppiPlanarFieldClearance(
  const MppiPlanarDistanceFieldView & field, const MppiPlanarBodyRect & body,
  const double x, const double y, const MppiPlanarBodyTrig & trig,
  const double radius, double * clearance)
{
  if(field.distances == nullptr){
    return false;
  }

  constexpr int circle_count = 3;
  const double slice_width = (body.x_max - body.x_min) / circle_count;
  const double circle_radius = ::hypot(slice_width, body.y_max - body.y_min) * 0.5;
  const double body_y = (body.y_min + body.y_max) * 0.5;
  double min_center_distance = DBL_MAX;
  for(int i = 0; i < circle_count; ++i){
    const double body_x = body.x_min + (i + 0.5) * slice_width;
    const double col = (x + trig.cos_yaw * body_x - trig.sin_yaw * body_y -
      field.origin_x) / field.resolution;
    const double row = (y + trig.sin_yaw * body_x + trig.cos_yaw * body_y -
      field.origin_y) / field.resolution;
    if(!(col >= 0.0 && col < static_cast<double>(field.width) &&
         row >= 0.0 && row < static_cast<double>(field.height)))
    {
      return false;
    }
    const std::size_t cell = static_cast<std::size_t>(row) * field.width +
      static_cast<std::size_t>(col);
    min_center_distance = ::fmin(min_center_distance, field.distances[cell]);
  }

  const double estimate = ::fmax(0.0, min_center_distance - circle_radius);
  if(estimate <= radius){
    *clearance = estimate;
  }
  return true;
}

// Uniform XY bucket index over points already projected into the frozen
// plane. Only grids produced by buildMppiPlanarObstacleGrid are supported.
// Cell boundaries are aligned with this build's point bounding box: cell
// (0, 0) starts at (origin_x, origin_y) and the last cell ends at or beyond
// the maximum point. The indexed span covers only the local obstacle set
// supplied by the upstream perception window, so the grid cannot report an
// obstacle outside it. A query AABB entirely outside that span visits no cell
// and returns evaluated=true with in_collision=false: that result certifies
// freedom only in the indexed window's convention, not in the world. Each
// cell's points are stored contiguously in points_xy, and cell_offsets holds
// cells_x * cells_y + 1 prefix sums so cell i owns the point-index range
// [cell_offsets[i], cell_offsets[i + 1]).
struct MppiPlanarObstacleGrid
{
  bool valid = false;
  double cell_size = 0.20;
  double origin_x = 0.0;
  double origin_y = 0.0;
  std::size_t cells_x = 0;
  std::size_t cells_y = 0;
  std::vector<std::uint32_t> cell_offsets;
  std::vector<double> points_xy;  // compact x,y pairs
  std::size_t point_count = 0;
  MppiPlanarDistanceField distance_field;
};

// POD view of a built grid for a device upload. The pointers alias the host
// grid's vectors, so the grid must outlive the view and must not be modified
// while the view is in use. Upload workflow: copy cell_offsets and points_xy
// into device buffers, then construct a NEW view on the device side (or on
// the host over the device pointers) with the same scalars. Never pass this
// host-pointer view to a kernel. A device kernel reuses the shared primitives
// mppiPlanarCellRangeForAabb and mppiPlanarPointToBody to implement the same
// range + point scan as the host query; the host-only nested loop itself
// lives in the .cpp.
struct MppiPlanarObstacleGridView
{
  const std::uint32_t * cell_offsets = nullptr;
  const double * points_xy = nullptr;
  std::size_t cells_x = 0;
  std::size_t cells_y = 0;
  std::size_t point_count = 0;
  double cell_size = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  bool valid = false;
  MppiPlanarDistanceFieldView distance_field;
};

MppiPlanarObstacleGridView mppiPlanarObstacleGridView(
  const MppiPlanarObstacleGrid & grid);

// Optional local distance field. An empty result selects pointwise clearance.
void buildMppiPlanarDistanceField(
  MppiPlanarObstacleGrid & grid, double clearance_radius);

// Inclusive cell range covered by a query AABB.
struct MppiPlanarCellRange
{
  std::size_t x_begin = 0;
  std::size_t x_end = 0;  // inclusive
  std::size_t y_begin = 0;
  std::size_t y_end = 0;  // inclusive
  bool empty = true;
};

// Intersects one axis of the requested range with [0, cells - 1]. Contact
// with a cell boundary is included; ranges entirely outside the grid are
// rejected so the caller can skip the scan.
MPPI_PLANAR_HD inline bool mppiPlanarAxisCellRange(
  const double minimum, const double maximum, const double origin,
  const double cell_size, const std::size_t cells,
  std::size_t * begin, std::size_t * end)
{
  if(cells == 0){
    return false;
  }
  const double lower_quotient = (minimum - origin) / cell_size;
  const double upper_quotient = (maximum - origin) / cell_size;
  // The margin grows with the quotient magnitude, i.e. with the distance from
  // the grid origin in cells. This keeps far-origin grids and large spans
  // safe without a per-point cost.
  const double margin = kMppiPlanarCellQueryMargin * DBL_EPSILON *
    (::fabs(lower_quotient) + ::fabs(upper_quotient) + 1.0);
  const double lower = ::floor(lower_quotient - margin);
  const double upper = ::floor(upper_quotient + margin);
  if(!(upper >= 0.0) || !(lower <= static_cast<double>(cells - 1))){
    return false;
  }
  const double last = static_cast<double>(cells - 1);
  *begin = lower < 0.0 ? 0 : static_cast<std::size_t>(lower);
  *end = upper > last ? cells - 1 : static_cast<std::size_t>(upper);
  return *begin <= *end;
}

// Fills the inclusive cell range of an AABB. Returns false when the AABB
// touches no cell or the view cannot be used.
MPPI_PLANAR_HD inline bool mppiPlanarCellRangeForAabb(
  const MppiPlanarObstacleGridView & view, const double minimum_x,
  const double maximum_x, const double minimum_y, const double maximum_y,
  MppiPlanarCellRange * range)
{
  if(range == nullptr || !view.valid || view.cells_x == 0 ||
     view.cells_y == 0 || !(view.cell_size > 0.0))
  {
    return false;
  }
  MppiPlanarCellRange result;
  if(!mppiPlanarAxisCellRange(
      minimum_x, maximum_x, view.origin_x, view.cell_size, view.cells_x,
      &result.x_begin, &result.x_end) ||
     !mppiPlanarAxisCellRange(
      minimum_y, maximum_y, view.origin_y, view.cell_size, view.cells_y,
      &result.y_begin, &result.y_end))
  {
    return false;
  }
  result.empty = false;
  *range = result;
  return true;
}

// Uses the min/max XY of the configured cuboid vertices. Returns an invalid
// rectangle for an empty vertex list, non-finite coordinates, or a degenerate
// (zero width or height) envelope.
MppiPlanarBodyRect mppiPlanarBodyRectFromCuboid(
  const std::vector<pcl::PointXYZ> & vertices);

// Builds the grid once per control tick with count, prefix sum and scatter
// passes: O(N + C) work for N points and C cells. Returns false on an invalid
// cell size, a non-finite point XY, or an index size that cannot be
// represented by std::uint32_t or allocated. An empty input builds a valid
// empty grid (valid == true, point_count == 0).
bool buildMppiPlanarObstacleGrid(
  const pcl::PointCloud<pcl::PointXYZI> & plane_points,
  const MppiPlanarObstacleGridConfig & config,
  MppiPlanarObstacleGrid * grid, std::string * error = nullptr);

struct MppiPlanarQueryResult
{
  bool in_collision = false;
  double clearance = std::numeric_limits<double>::infinity();
  // The collision and clearance passes scan separately, so their work is
  // reported separately. A pass that did not run (clearance not requested,
  // collision found, or a radius that excludes the body AABB) leaves its own
  // counters at zero; the two channels never double-count one scan.
  std::size_t collision_cell_checks = 0;
  std::size_t collision_point_checks = 0;
  std::size_t clearance_cell_checks = 0;
  std::size_t clearance_point_checks = 0;
  // False means the query did not run (unknown, not free). Callers must never
  // treat it as collision-free.
  bool evaluated = false;
};

// Visits only the cells overlapping the body's axis-aligned bounding box
// (expanded by inflation_radius for the clearance pass). Exact geometry is
// evaluated on the original XY points. Collision is always checked first and
// stops the query early; a negative, NaN or infinite inflation_radius only
// disables the clearance refinement. Clearance is the minimum
// point-to-rectangle distance among points within inflation_radius; it stays
// +infinity when nothing is that close or the clearance pass did not run. The
// query performs no allocation.
MppiPlanarQueryResult queryMppiPlanarObstacleGrid(
  const MppiPlanarObstacleGrid & grid, const MppiPlanarBodyRect & body,
  double center_x, double center_y, double yaw, double inflation_radius);

}  // namespace local_planner

#endif  // LOCAL_PLANNER_MPPI_PLANAR_OBSTACLES_H_
