#include "local_planner/mppi_planar_obstacles.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <new>
#include <stdexcept>
#include <utility>

namespace local_planner
{
namespace
{

constexpr std::size_t kMaximumIndexedPoints =
  static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max());
constexpr std::uint64_t kMaximumIndexedCells =
  static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max());
constexpr double kDistanceFieldResolution = 0.10;
constexpr std::size_t kMaximumDistanceFieldCells = 262144;

void setError(std::string * error, const std::string & message)
{
  if(error != nullptr){
    *error = message;
  }
}

void resetGrid(MppiPlanarObstacleGrid * grid)
{
  *grid = MppiPlanarObstacleGrid{};
}

// Maps a finite coordinate into [0, cells - 1]. The last cell absorbs points
// on (or rounded onto) the maximum boundary so no index ever escapes the
// grid; negative coordinates resolve through the aligned origin.
std::size_t gridCellIndex(
  const double value, const double origin, const double cell_size,
  const std::size_t cells)
{
  const double index = std::floor((value - origin) / cell_size);
  if(!(index > 0.0)){
    return 0;
  }
  if(index >= static_cast<double>(cells)){
    return cells - 1;
  }
  return static_cast<std::size_t>(index);
}

// O(1) structural checks only. Prefix-sum monotonicity is trusted from the
// builder; re-scanning it per query would make queries O(C).
bool planarGridStructureValid(const MppiPlanarObstacleGrid & grid)
{
  if(!grid.valid || !std::isfinite(grid.cell_size) ||
     grid.cell_size <= 0.0 || !std::isfinite(grid.origin_x) ||
     !std::isfinite(grid.origin_y))
  {
    return false;
  }
  if(grid.cells_x > kMaximumIndexedPoints ||
     grid.cells_y > kMaximumIndexedPoints)
  {
    return false;
  }
  if(grid.cells_y != 0 &&
     grid.cells_x > std::numeric_limits<std::size_t>::max() / grid.cells_y)
  {
    return false;
  }
  const std::size_t cell_count = grid.cells_x * grid.cells_y;
  if(cell_count == std::numeric_limits<std::size_t>::max()){
    return false;
  }
  if(grid.point_count > kMaximumIndexedPoints ||
     grid.point_count > std::numeric_limits<std::size_t>::max() / 2)
  {
    return false;
  }
  if(grid.cell_offsets.size() != cell_count + 1 ||
     grid.points_xy.size() != grid.point_count * 2 ||
     grid.cell_offsets.front() != 0 ||
     grid.cell_offsets.back() != grid.point_count)
  {
    return false;
  }
  return true;
}

bool planarRangeHasCollision(
  const MppiPlanarObstacleGridView & view, const MppiPlanarBodyRect & body,
  const double center_x, const double center_y,
  const MppiPlanarBodyTrig & trig, const MppiPlanarCellRange & range,
  std::size_t * cell_checks, std::size_t * point_checks)
{
  for(std::size_t y = range.y_begin; y <= range.y_end; ++y){
    for(std::size_t x = range.x_begin; x <= range.x_end; ++x){
      ++*cell_checks;
      const std::size_t cell = y * view.cells_x + x;
      const std::uint32_t begin = view.cell_offsets[cell];
      const std::uint32_t end = view.cell_offsets[cell + 1];
      for(std::uint32_t index = begin; index < end; ++index){
        ++*point_checks;
        const std::size_t point = 2 * static_cast<std::size_t>(index);
        if(mppiPlanarPointToBody(
            body, center_x, center_y, trig, view.points_xy[point],
            view.points_xy[point + 1]).inside())
        {
          return true;
        }
      }
    }
  }
  return false;
}

double planarRangeMinimumClearance(
  const MppiPlanarObstacleGridView & view, const MppiPlanarBodyRect & body,
  const double center_x, const double center_y,
  const MppiPlanarBodyTrig & trig, const double inflation_radius,
  const MppiPlanarCellRange & range, std::size_t * cell_checks,
  std::size_t * point_checks)
{
  double minimum = std::numeric_limits<double>::infinity();
  double search_limit = std::max(inflation_radius, kMppiPlanarBoundaryTolerance);
  for(std::size_t y = range.y_begin; y <= range.y_end; ++y){
    for(std::size_t x = range.x_begin; x <= range.x_end; ++x){
      ++*cell_checks;
      const std::size_t cell = y * view.cells_x + x;
      const std::uint32_t begin = view.cell_offsets[cell];
      const std::uint32_t end = view.cell_offsets[cell + 1];
      for(std::uint32_t index = begin; index < end; ++index){
        ++*point_checks;
        const std::size_t point = 2 * static_cast<std::size_t>(index);
        const auto distance = mppiPlanarPointToBody(
          body, center_x, center_y, trig, view.points_xy[point],
          view.points_xy[point + 1]);
        // Either axis is a lower bound on the distance. Keep the inside
        // tolerance in the bound because those distances are reported as zero.
        if(distance.dx > search_limit || distance.dy > search_limit){
          continue;
        }
        const double clearance = distance.clearance();
        if(clearance <= inflation_radius && clearance < minimum){
          minimum = clearance;
          search_limit = std::max(minimum, kMppiPlanarBoundaryTolerance);
        }
      }
    }
  }
  return minimum;
}

// Lower envelope of parabolas for the separable squared Euclidean transform.
void squaredDistanceTransform(
  const std::vector<double> & input, const std::size_t count,
  std::vector<double> & output, std::vector<std::size_t> & sites,
  std::vector<double> & starts)
{
  std::size_t site_count = 0;
  const double infinity = std::numeric_limits<double>::infinity();
  for(std::size_t q = 0; q < count; ++q){
    if(!std::isfinite(input[q])){
      continue;
    }
    double start = -infinity;
    const double x = static_cast<double>(q);
    while(site_count > 0){
      const std::size_t p = sites[site_count - 1];
      const double previous_x = static_cast<double>(p);
      start = (input[q] + x * x - input[p] - previous_x * previous_x) /
        (2.0 * (x - previous_x));
      if(start > starts[site_count - 1]){
        break;
      }
      --site_count;
    }
    starts[site_count] = site_count == 0 ? -infinity : start;
    sites[site_count++] = q;
  }

  if(site_count == 0){
    std::fill_n(output.begin(), count, infinity);
    return;
  }
  std::size_t site = 0;
  for(std::size_t q = 0; q < count; ++q){
    const double x = static_cast<double>(q);
    while(site + 1 < site_count && starts[site + 1] < x){
      ++site;
    }
    const double dx = x - static_cast<double>(sites[site]);
    output[q] = input[sites[site]] + dx * dx;
  }
}

}  // namespace

MppiPlanarBodyRect mppiPlanarBodyRectFromCuboid(
  const std::vector<pcl::PointXYZ> & vertices)
{
  MppiPlanarBodyRect rect;
  if(vertices.empty()){
    return rect;
  }

  double x_min = std::numeric_limits<double>::infinity();
  double x_max = -std::numeric_limits<double>::infinity();
  double y_min = x_min;
  double y_max = x_max;
  for(const auto & vertex : vertices){
    const double x = static_cast<double>(vertex.x);
    const double y = static_cast<double>(vertex.y);
    if(!std::isfinite(x) || !std::isfinite(y)){
      return rect;
    }
    x_min = std::min(x_min, x);
    x_max = std::max(x_max, x);
    y_min = std::min(y_min, y);
    y_max = std::max(y_max, y);
  }
  if(!(x_max > x_min) || !(y_max > y_min)){
    return rect;
  }

  rect.x_min = x_min;
  rect.x_max = x_max;
  rect.y_min = y_min;
  rect.y_max = y_max;
  rect.valid = true;
  return rect;
}

MppiPlanarObstacleGridView mppiPlanarObstacleGridView(
  const MppiPlanarObstacleGrid & grid)
{
  MppiPlanarObstacleGridView view;
  view.cell_offsets = grid.cell_offsets.data();
  view.points_xy = grid.points_xy.data();
  view.cells_x = grid.cells_x;
  view.cells_y = grid.cells_y;
  view.point_count = grid.point_count;
  view.cell_size = grid.cell_size;
  view.origin_x = grid.origin_x;
  view.origin_y = grid.origin_y;
  view.valid = grid.valid;
  if(!grid.distance_field.distances.empty()){
    const auto & field = grid.distance_field;
    view.distance_field = {
      field.distances.data(), field.width, field.height, field.origin_x,
      field.origin_y, field.resolution};
  }
  return view;
}

void buildMppiPlanarDistanceField(
  MppiPlanarObstacleGrid & grid, const double clearance_radius)
{
  grid.distance_field = {};
  if(!planarGridStructureValid(grid) || grid.point_count == 0 ||
     !std::isfinite(clearance_radius) || clearance_radius <= 0.0)
  {
    return;
  }

  // Include useful query locations outside the obstacle bounding box. Queries
  // beyond the padded field still use the point grid.
  const double padding = clearance_radius + 1.0;
  const double width = std::ceil(
    (grid.cells_x * grid.cell_size + 2.0 * padding) / kDistanceFieldResolution);
  const double height = std::ceil(
    (grid.cells_y * grid.cell_size + 2.0 * padding) / kDistanceFieldResolution);
  if(!(width >= 1.0 && height >= 1.0 && width * height <= kMaximumDistanceFieldCells)){
    return;
  }

  MppiPlanarDistanceField field;
  field.width = static_cast<std::size_t>(width);
  field.height = static_cast<std::size_t>(height);
  field.origin_x = grid.origin_x - padding;
  field.origin_y = grid.origin_y - padding;
  field.resolution = kDistanceFieldResolution;
  try{
    const std::size_t cell_count = field.width * field.height;
    std::vector<double> work(cell_count, std::numeric_limits<double>::infinity());
    for(std::size_t i = 0; i < grid.points_xy.size(); i += 2){
      const double x = (grid.points_xy[i] - field.origin_x) / field.resolution;
      const double y = (grid.points_xy[i + 1] - field.origin_y) / field.resolution;
      if(!(x >= 0.0 && x < width && y >= 0.0 && y < height)){
        return;
      }
      work[static_cast<std::size_t>(y) * field.width + static_cast<std::size_t>(x)] = 0.0;
    }

    const std::size_t line_size = std::max(field.width, field.height);
    std::vector<double> input(line_size);
    std::vector<double> output(line_size);
    std::vector<double> starts(line_size);
    std::vector<std::size_t> sites(line_size);
    for(std::size_t y = 0; y < field.height; ++y){
      auto row = work.begin() + y * field.width;
      std::copy_n(row, field.width, input.begin());
      squaredDistanceTransform(input, field.width, output, sites, starts);
      std::copy_n(output.begin(), field.width, row);
    }
    field.distances.resize(cell_count);
    for(std::size_t x = 0; x < field.width; ++x){
      for(std::size_t y = 0; y < field.height; ++y){
        input[y] = work[y * field.width + x];
      }
      squaredDistanceTransform(input, field.height, output, sites, starts);
      for(std::size_t y = 0; y < field.height; ++y){
        field.distances[y * field.width + x] =
          static_cast<float>(std::sqrt(output[y]) * field.resolution);
      }
    }
  }
  catch(const std::bad_alloc &){
    return;
  }
  catch(const std::length_error &){
    return;
  }
  grid.distance_field = std::move(field);
}

bool buildMppiPlanarObstacleGrid(
  const pcl::PointCloud<pcl::PointXYZI> & plane_points,
  const MppiPlanarObstacleGridConfig & config,
  MppiPlanarObstacleGrid * grid, std::string * error)
{
  if(error != nullptr){
    error->clear();
  }
  if(grid == nullptr){
    setError(error, "planar obstacle grid output is null");
    return false;
  }
  resetGrid(grid);
  if(!std::isfinite(config.cell_size) || config.cell_size <= 0.0){
    setError(error, "planar obstacle grid cell_size must be finite and positive");
    return false;
  }

  const std::size_t point_count = plane_points.points.size();
  if(point_count > kMaximumIndexedPoints){
    setError(error, "planar obstacle point count exceeds the 32-bit index range");
    return false;
  }
  if(point_count > std::numeric_limits<std::size_t>::max() / 2){
    setError(error, "planar obstacle point count exceeds the addressable index range");
    return false;
  }
  if(point_count == 0){
    // A valid but empty cloud still produces a usable empty index.
    grid->valid = true;
    grid->cell_size = config.cell_size;
    grid->cell_offsets.assign(1, std::uint32_t{0});
    return true;
  }

  double minimum_x = std::numeric_limits<double>::infinity();
  double maximum_x = -std::numeric_limits<double>::infinity();
  double minimum_y = minimum_x;
  double maximum_y = maximum_x;
  for(std::size_t index = 0; index < point_count; ++index){
    const auto & point = plane_points.points[index];
    const double x = static_cast<double>(point.x);
    const double y = static_cast<double>(point.y);
    if(!std::isfinite(x) || !std::isfinite(y)){
      setError(
        error, "planar obstacle point " + std::to_string(index) +
        " has a non-finite XY coordinate");
      return false;
    }
    minimum_x = std::min(minimum_x, x);
    maximum_x = std::max(maximum_x, x);
    minimum_y = std::min(minimum_y, y);
    maximum_y = std::max(maximum_y, y);
  }
  const double span_x = maximum_x - minimum_x;
  const double span_y = maximum_y - minimum_y;
  if(!std::isfinite(span_x) || !std::isfinite(span_y)){
    setError(error, "planar obstacle point bounds are not finite");
    return false;
  }

  // Every non-empty cloud owns at least one cell per axis; a zero span still
  // needs one cell to hold its points.
  const double cells_x_value = std::max(
    1.0, std::ceil(span_x / config.cell_size));
  const double cells_y_value = std::max(
    1.0, std::ceil(span_y / config.cell_size));
  if(!std::isfinite(cells_x_value) || !std::isfinite(cells_y_value) ||
     cells_x_value > static_cast<double>(kMaximumIndexedCells) ||
     cells_y_value > static_cast<double>(kMaximumIndexedCells))
  {
    setError(error, "planar obstacle grid cell count exceeds the 32-bit index range");
    return false;
  }
  const std::uint64_t cells_x = static_cast<std::uint64_t>(cells_x_value);
  const std::uint64_t cells_y = static_cast<std::uint64_t>(cells_y_value);
  if(cells_x != 0 && cells_y > kMaximumIndexedCells / cells_x){
    setError(error, "planar obstacle grid cell count overflows the 32-bit index range");
    return false;
  }
  const std::uint64_t cell_count = cells_x * cells_y;
  // cell_offsets holds cell_count + 1 elements; keep that addition inside the
  // addressable range even on a 32-bit size_t build.
  if(cell_count >
     static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max()) - 1u)
  {
    setError(error, "planar obstacle grid cell count exceeds the addressable index range");
    return false;
  }

  grid->cell_size = config.cell_size;
  grid->origin_x = minimum_x;
  grid->origin_y = minimum_y;
  grid->cells_x = static_cast<std::size_t>(cells_x);
  grid->cells_y = static_cast<std::size_t>(cells_y);
  grid->point_count = point_count;

  try{
    // Count pass: cell_offsets[i + 1] receives the number of points in cell i.
    grid->cell_offsets.assign(
      static_cast<std::size_t>(cell_count) + 1, std::uint32_t{0});
    grid->points_xy.resize(point_count * 2);
    for(std::size_t index = 0; index < point_count; ++index){
      const auto & point = plane_points.points[index];
      const std::size_t x_index = gridCellIndex(
        static_cast<double>(point.x), minimum_x, config.cell_size,
        grid->cells_x);
      const std::size_t y_index = gridCellIndex(
        static_cast<double>(point.y), minimum_y, config.cell_size,
        grid->cells_y);
      ++grid->cell_offsets[y_index * grid->cells_x + x_index + 1];
    }
    // Prefix sum: cell_offsets[c] is now the first point index of cell c.
    for(std::size_t cell = 1; cell < grid->cell_offsets.size(); ++cell){
      grid->cell_offsets[cell] += grid->cell_offsets[cell - 1];
    }
    // Scatter pass: write each point into its cell's contiguous slice.
    std::vector<std::uint32_t> write_cursor(
      grid->cell_offsets.begin(), grid->cell_offsets.end() - 1);
    for(std::size_t index = 0; index < point_count; ++index){
      const auto & point = plane_points.points[index];
      const std::size_t x_index = gridCellIndex(
        static_cast<double>(point.x), minimum_x, config.cell_size,
        grid->cells_x);
      const std::size_t y_index = gridCellIndex(
        static_cast<double>(point.y), minimum_y, config.cell_size,
        grid->cells_y);
      const std::size_t cell = y_index * grid->cells_x + x_index;
      const std::size_t destination =
        static_cast<std::size_t>(write_cursor[cell]++);
      grid->points_xy[2 * destination] = static_cast<double>(point.x);
      grid->points_xy[2 * destination + 1] = static_cast<double>(point.y);
    }
  }
  catch(const std::bad_alloc &){
    resetGrid(grid);
    setError(error, "planar obstacle grid allocation failed");
    return false;
  }
  catch(const std::length_error &){
    resetGrid(grid);
    setError(error, "planar obstacle grid size is not allocatable");
    return false;
  }

  grid->valid = true;
  return true;
}

MppiPlanarQueryResult queryMppiPlanarObstacleGrid(
  const MppiPlanarObstacleGrid & grid, const MppiPlanarBodyRect & body,
  const double center_x, const double center_y, const double yaw,
  const double inflation_radius)
{
  MppiPlanarQueryResult result;
  if(!body.valid || !std::isfinite(body.x_min) ||
     !std::isfinite(body.x_max) || !std::isfinite(body.y_min) ||
     !std::isfinite(body.y_max) || body.x_max < body.x_min ||
     body.y_max < body.y_min ||
     !std::isfinite(center_x) || !std::isfinite(center_y) ||
     !std::isfinite(yaw) || !planarGridStructureValid(grid))
  {
    return result;
  }
  result.evaluated = true;
  if(grid.cells_x == 0 || grid.cells_y == 0){
    return result;
  }

  const MppiPlanarBodyTrig trig = mppiPlanarBodyTrig(yaw);
  const MppiPlanarObstacleGridView view = mppiPlanarObstacleGridView(grid);

  // Collision is always checked, whatever the inflation radius says. The
  // AABB carries the local 1e-6 classification tolerance because inside()
  // accepts points just outside an edge; without it such a point could be
  // classified as a collision in a cell the range never visits.
  const MppiPlanarAabb collision_aabb = mppiPlanarBodyAabb(
    body, center_x, center_y, trig, kMppiPlanarBoundaryTolerance, 0.0);
  MppiPlanarCellRange collision_range;
  if(mppiPlanarCellRangeForAabb(
      view, collision_aabb.minimum_x, collision_aabb.maximum_x,
      collision_aabb.minimum_y, collision_aabb.maximum_y,
      &collision_range) &&
     planarRangeHasCollision(
      view, body, center_x, center_y, trig, collision_range,
      &result.collision_cell_checks, &result.collision_point_checks))
  {
    result.in_collision = true;
    result.clearance = 0.0;
    return result;
  }

  // Clearance is an optional refinement for a finite positive radius only. A
  // negative, NaN or infinite radius leaves the query collision-only.
  if(!std::isfinite(inflation_radius) || inflation_radius <= 0.0){
    return result;
  }

  // Keep the same local tolerance here so a boundary point that clearance()
  // clamps to 0 is still visited when the radius itself is tiny.
  const MppiPlanarAabb clearance_aabb = mppiPlanarBodyAabb(
    body, center_x, center_y, trig, kMppiPlanarBoundaryTolerance,
    inflation_radius);
  MppiPlanarCellRange clearance_range;
  if(!mppiPlanarCellRangeForAabb(
      view, clearance_aabb.minimum_x, clearance_aabb.maximum_x,
      clearance_aabb.minimum_y, clearance_aabb.maximum_y,
      &clearance_range))
  {
    return result;
  }
  result.clearance = planarRangeMinimumClearance(
    view, body, center_x, center_y, trig, inflation_radius, clearance_range,
    &result.clearance_cell_checks, &result.clearance_point_checks);
  return result;
}

}  // namespace local_planner
