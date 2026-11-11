#include <global_planner/terrain_grid.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>

namespace global_planner {
namespace {

// Cell coordinates are intentionally bounded to the int range because the
// public metadata and getCell() use int.  This is far beyond any practical map
// extent while keeping all query-loop arithmetic free of int64 overflow.
constexpr std::int64_t kCellCoordLimit =
    static_cast<std::int64_t>(std::numeric_limits<int>::max()) - 1;

struct QueryHit {
  float squared_distance = 0.0f;
  int index = 0;
};

}  // namespace

TerrainGrid::TerrainGrid(const Params& params) : params_(params) {}

bool TerrainGrid::validParams(const Params& params) {
  return std::isfinite(params.cell_size) && params.cell_size > 0.0f &&
         std::isfinite(params.z_tolerance) && params.z_tolerance > 0.0f &&
         std::isfinite(params.max_query_radius) &&
         params.max_query_radius > 0.0f && params.max_query_cells > 0;
}

bool TerrainGrid::validParams() const { return validParams(params_); }

bool TerrainGrid::setParams(const Params& params) {
  if (!validParams(params)) {
    return false;
  }
  params_ = params;
  clear();
  return true;
}

std::size_t TerrainGrid::CellKeyHash::operator()(const CellKey& key) const {
  // 64-bit mix; the exact function is not part of the public contract.
  std::uint64_t hash =
      static_cast<std::uint64_t>(key.ix) * 0x9E3779B97F4A7C15ULL;
  hash ^= static_cast<std::uint64_t>(key.iy) + 0x9E3779B97F4A7C15ULL +
          (hash << 6) + (hash >> 2);
  return static_cast<std::size_t>(hash);
}

void TerrainGrid::clear() {
  cells_.clear();
  built_ = false;
  point_count_ = 0;
  surface_count_ = 0;
  skipped_non_finite_ = 0;
  skipped_out_of_range_ = 0;
}

bool TerrainGrid::cellCoordinate(float value, std::int64_t* out) const {
  if (out == nullptr || !std::isfinite(value)) {
    return false;
  }
  const double scaled =
      std::floor(static_cast<double>(value) /
                 static_cast<double>(params_.cell_size));
  if (!std::isfinite(scaled) ||
      scaled < -static_cast<double>(kCellCoordLimit) ||
      scaled > static_cast<double>(kCellCoordLimit)) {
    return false;
  }
  *out = static_cast<std::int64_t>(scaled);
  return true;
}

void TerrainGrid::insertPointInternal(int index,
                                      const TerrainPointXYZ& point) {
  std::int64_t ix = 0;
  std::int64_t iy = 0;
  if (!cellCoordinate(point.x, &ix) || !cellCoordinate(point.y, &iy)) {
    ++skipped_out_of_range_;
    return;
  }

  const auto inserted =
      cells_.emplace(CellKey{ix, iy}, CellData{});
  CellData& cell = inserted.first->second;
  if (inserted.second) {
    cell.ix = static_cast<int>(ix);
    cell.iy = static_cast<int>(iy);
  }
  cell.points.push_back(StoredPoint{index, point.x, point.y, point.z});
  ++point_count_;
}

void TerrainGrid::finalizeBuild() {
  surface_count_ = 0;
  const double tolerance = static_cast<double>(params_.z_tolerance);

  for (auto& entry : cells_) {
    CellData& cell = entry.second;
    const std::size_t total = cell.points.size();
    if (total == 0) {
      continue;
    }

    // Total order by height, then original index.  The geometric clustering is
    // independent of input order; the index tie-break only makes equal-height
    // data deterministic while preserving each point's original index.
    std::sort(cell.points.begin(), cell.points.end(),
              [](const StoredPoint& a, const StoredPoint& b) {
                if (a.z != b.z) {
                  return a.z < b.z;
                }
                return a.index < b.index;
              });

    double cell_sum = 0.0;
    for (const StoredPoint& point : cell.points) {
      cell_sum += static_cast<double>(point.z);
    }
    cell.point_count = static_cast<int>(total);
    cell.z_min = cell.points.front().z;
    cell.z_max = cell.points.back().z;
    cell.z_mean =
        static_cast<float>(cell_sum / static_cast<double>(total));

    cell.surfaces.clear();
    cell.surfaces.reserve(1);
    std::size_t begin = 0;
    while (begin < total) {
      // A surface's total height span never exceeds z_tolerance.  This is
      // deliberately non-transitive: a chain of small gaps cannot merge an
      // upper and lower surface whose overall span is larger than the
      // tolerance.
      const double anchor_z = static_cast<double>(cell.points[begin].z);
      std::size_t end = begin + 1;
      while (end < total &&
             static_cast<double>(cell.points[end].z) - anchor_z <= tolerance) {
        ++end;
      }

      SurfaceData surface;
      surface.point_count = static_cast<int>(end - begin);
      surface.z_min = cell.points[begin].z;
      surface.z_max = cell.points[end - 1].z;
      surface.points.reserve(end - begin);
      double surface_sum = 0.0;
      for (std::size_t i = begin; i < end; ++i) {
        surface_sum += static_cast<double>(cell.points[i].z);
        surface.points.push_back(std::move(cell.points[i]));
      }
      surface.z_mean = static_cast<float>(
          surface_sum / static_cast<double>(surface.point_count));
      cell.surfaces.push_back(std::move(surface));
      ++surface_count_;
      begin = end;
    }

    cell.points.clear();
    cell.points.shrink_to_fit();
  }
}

TerrainBuildStatus TerrainGrid::build(const TerrainPointXYZ* points,
                                      std::size_t count) {
  clear();
  if (!validParams()) {
    return TerrainBuildStatus::kInvalidParams;
  }
  if (points == nullptr && count != 0) {
    return TerrainBuildStatus::kInvalidInput;
  }
  if (count >
      static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return TerrainBuildStatus::kTooManyPoints;
  }
  if (count == 0) {
    built_ = true;
    return TerrainBuildStatus::kOk;
  }

  for (std::size_t i = 0; i < count; ++i) {
    const TerrainPointXYZ& point = points[i];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      ++skipped_non_finite_;
      continue;
    }
    insertPointInternal(static_cast<int>(i), point);
  }

  finalizeBuild();
  built_ = true;
  return TerrainBuildStatus::kOk;
}

TerrainCell TerrainGrid::toPublicCell(const CellData& cell) {
  TerrainCell out;
  out.ix = cell.ix;
  out.iy = cell.iy;
  out.point_count = cell.point_count;
  out.z_min = cell.z_min;
  out.z_max = cell.z_max;
  out.z_mean = cell.z_mean;
  out.surfaces.reserve(cell.surfaces.size());
  for (const SurfaceData& surface : cell.surfaces) {
    TerrainSurface public_surface;
    public_surface.z_min = surface.z_min;
    public_surface.z_max = surface.z_max;
    public_surface.z_mean = surface.z_mean;
    public_surface.point_count = surface.point_count;
    public_surface.point_indices.reserve(surface.points.size());
    for (const StoredPoint& point : surface.points) {
      public_surface.point_indices.push_back(point.index);
    }
    out.surfaces.push_back(std::move(public_surface));
  }
  return out;
}

bool TerrainGrid::getCell(int ix, int iy, TerrainCell* cell) const {
  if (cell == nullptr) {
    return false;
  }
  const auto it = cells_.find(CellKey{ix, iy});
  if (it == cells_.end()) {
    return false;
  }
  *cell = toPublicCell(it->second);
  return true;
}

bool TerrainGrid::getCellAt(float x, float y, TerrainCell* cell) const {
  if (cell == nullptr || !built_) {
    return false;
  }
  std::int64_t ix = 0;
  std::int64_t iy = 0;
  if (!cellCoordinate(x, &ix) || !cellCoordinate(y, &iy)) {
    return false;
  }
  return getCell(static_cast<int>(ix), static_cast<int>(iy), cell);
}

std::vector<TerrainCell> TerrainGrid::cells() const {
  std::vector<TerrainCell> out;
  out.reserve(cells_.size());
  for (const auto& entry : cells_) {
    out.push_back(toPublicCell(entry.second));
  }
  std::sort(out.begin(), out.end(),
            [](const TerrainCell& a, const TerrainCell& b) {
              if (a.ix != b.ix) {
                return a.ix < b.ix;
              }
              return a.iy < b.iy;
            });
  return out;
}

TerrainQueryResult TerrainGrid::radiusSearch(const TerrainPointXYZ& query,
                                             float radius) const {
  TerrainQueryResult result;
  result.status = TerrainQueryStatus::kNotBuilt;
  radiusSearch(query, radius, result.indices, result.squared_distances,
               &result.status);
  return result;
}

int TerrainGrid::radiusSearch(const TerrainPointXYZ& query, float radius,
                              std::vector<int>& indices,
                              std::vector<float>& squared_distances,
                              TerrainQueryStatus* status) const {
  indices.clear();
  squared_distances.clear();
  if (status != nullptr) {
    *status = TerrainQueryStatus::kNotBuilt;
  }
  if (!built_) {
    return -1;
  }

  if (!std::isfinite(query.x) || !std::isfinite(query.y) ||
      !std::isfinite(query.z) || !std::isfinite(radius) || radius < 0.0f) {
    if (status != nullptr) {
      *status = TerrainQueryStatus::kInvalidQuery;
    }
    return -1;
  }

  if (radius > params_.max_query_radius) {
    if (status != nullptr) {
      *status = TerrainQueryStatus::kQueryTooLarge;
    }
    return -1;
  }

  // Match the existing nanoflann/PCL wrapper exactly: the squared search
  // radius is a float and points with dist_sq == radius_sq are excluded.
  const float radius_squared = radius * radius;
  if (!std::isfinite(radius_squared)) {
    if (status != nullptr) {
      *status = TerrainQueryStatus::kQueryTooLarge;
    }
    return -1;
  }

  const double cell_size = static_cast<double>(params_.cell_size);
  const double radius_d = static_cast<double>(radius);
  const double low_x =
      (static_cast<double>(query.x) - radius_d) / cell_size;
  const double high_x =
      (static_cast<double>(query.x) + radius_d) / cell_size;
  const double low_y =
      (static_cast<double>(query.y) - radius_d) / cell_size;
  const double high_y =
      (static_cast<double>(query.y) + radius_d) / cell_size;

  double cell_x0 = std::floor(low_x);
  double cell_x1 = std::floor(high_x);
  double cell_y0 = std::floor(low_y);
  double cell_y1 = std::floor(high_y);

  if (!std::isfinite(cell_x0) || !std::isfinite(cell_x1) ||
      !std::isfinite(cell_y0) || !std::isfinite(cell_y1)) {
    if (status != nullptr) {
      *status = TerrainQueryStatus::kQueryTooLarge;
    }
    return -1;
  }

  const double limit = static_cast<double>(kCellCoordLimit);
  // The entire query disk lies outside the storable cell domain: this is a
  // legal empty result, not an oversized traversal.
  if (cell_x1 < -limit || cell_x0 > limit || cell_y1 < -limit ||
      cell_y0 > limit) {
    if (status != nullptr) {
      *status = TerrainQueryStatus::kOk;
    }
    return 0;
  }

  cell_x0 = std::max(cell_x0, -limit);
  cell_x1 = std::min(cell_x1, limit);
  cell_y0 = std::max(cell_y0, -limit);
  cell_y1 = std::min(cell_y1, limit);

  const double span_x = cell_x1 - cell_x0 + 1.0;
  const double span_y = cell_y1 - cell_y0 + 1.0;
  if (!(span_x > 0.0) || !(span_y > 0.0)) {
    if (status != nullptr) {
      *status = TerrainQueryStatus::kQueryTooLarge;
    }
    return -1;
  }
  const double max_cells = static_cast<double>(params_.max_query_cells);
  if (span_x > max_cells || span_y > max_cells ||
      span_x * span_y > max_cells) {
    if (status != nullptr) {
      *status = TerrainQueryStatus::kQueryTooLarge;
    }
    return -1;
  }

  const std::int64_t ix_begin = static_cast<std::int64_t>(cell_x0);
  const std::int64_t ix_end = static_cast<std::int64_t>(cell_x1);
  const std::int64_t iy_begin = static_cast<std::int64_t>(cell_y0);
  const std::int64_t iy_end = static_cast<std::int64_t>(cell_y1);

  std::vector<QueryHit> hits;
  for (std::int64_t ix = ix_begin; ix <= ix_end; ++ix) {
    for (std::int64_t iy = iy_begin; iy <= iy_end; ++iy) {
      const auto it = cells_.find(CellKey{ix, iy});
      if (it == cells_.end()) {
        continue;
      }
      for (const SurfaceData& surface : it->second.surfaces) {
        for (const StoredPoint& point : surface.points) {
          const float dx = point.x - query.x;
          const float dy = point.y - query.y;
          const float dz = point.z - query.z;
          const float distance_squared = dx * dx + dy * dy + dz * dz;
          if (distance_squared < radius_squared) {
            hits.push_back(QueryHit{distance_squared, point.index});
          }
        }
      }
    }
  }

  std::sort(hits.begin(), hits.end(),
            [](const QueryHit& a, const QueryHit& b) {
              if (a.squared_distance != b.squared_distance) {
                return a.squared_distance < b.squared_distance;
              }
              return a.index < b.index;
            });

  indices.reserve(hits.size());
  squared_distances.reserve(hits.size());
  for (const QueryHit& hit : hits) {
    indices.push_back(hit.index);
    squared_distances.push_back(hit.squared_distance);
  }

  if (status != nullptr) {
    *status = TerrainQueryStatus::kOk;
  }
  return static_cast<int>(hits.size());
}

const char* TerrainGrid::toString(TerrainBuildStatus status) {
  switch (status) {
    case TerrainBuildStatus::kOk:
      return "kOk";
    case TerrainBuildStatus::kInvalidParams:
      return "kInvalidParams";
    case TerrainBuildStatus::kInvalidInput:
      return "kInvalidInput";
    case TerrainBuildStatus::kTooManyPoints:
      return "kTooManyPoints";
  }
  return "unknown";
}

const char* TerrainGrid::toString(TerrainQueryStatus status) {
  switch (status) {
    case TerrainQueryStatus::kOk:
      return "kOk";
    case TerrainQueryStatus::kNotBuilt:
      return "kNotBuilt";
    case TerrainQueryStatus::kInvalidQuery:
      return "kInvalidQuery";
    case TerrainQueryStatus::kQueryTooLarge:
      return "kQueryTooLarge";
  }
  return "unknown";
}

}  // namespace global_planner
