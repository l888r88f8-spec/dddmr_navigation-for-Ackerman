#ifndef GLOBAL_PLANNER_TERRAIN_GRID_H
#define GLOBAL_PLANNER_TERRAIN_GRID_H

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace global_planner {

// Lightweight point adapter.  PCL clouds (or any type exposing x/y/z) can be
// fed through the iterator build overload and are converted to this type.
struct TerrainPointXYZ {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;

  bool operator==(const TerrainPointXYZ& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
  bool operator!=(const TerrainPointXYZ& other) const {
    return !(*this == other);
  }
};

enum class TerrainBuildStatus {
  kOk = 0,
  kInvalidParams,
  kInvalidInput,
  kTooManyPoints,
};

enum class TerrainQueryStatus {
  kOk = 0,
  kNotBuilt,
  kInvalidQuery,
  kQueryTooLarge,
};

// A single ground surface inside one XY cell.  point_count is a raw sample
// count, not a calibrated confidence value.
struct TerrainSurface {
  float z_min = 0.0f;
  float z_max = 0.0f;
  float z_mean = 0.0f;
  int point_count = 0;
  std::vector<int> point_indices;  // indices into the original input sequence
};

// Metadata for one occupied sparse XY cell.  Unknown cells are simply absent.
struct TerrainCell {
  int ix = 0;
  int iy = 0;
  int point_count = 0;
  float z_min = 0.0f;
  float z_max = 0.0f;
  float z_mean = 0.0f;
  std::vector<TerrainSurface> surfaces;
};

struct TerrainQueryResult {
  TerrainQueryStatus status = TerrainQueryStatus::kNotBuilt;
  std::vector<int> indices;
  std::vector<float> squared_distances;

  bool ok() const { return status == TerrainQueryStatus::kOk; }
  std::size_t size() const { return indices.size(); }
};

// Sparse multi-surface XY hash grid.
//
// - cell_size: XY cell edge in metres (default 0.3).
// - z_tolerance: maximum z span of one surface in a cell (default 0.2).  A
//   surface is never formed by a transitive height chain, so two points whose
//   heights differ by more than z_tolerance are never in the same surface.
// - max_query_radius / max_query_cells: hard service limits for radiusSearch.
//   Queries beyond either limit fail with kQueryTooLarge instead of silently
//   returning an empty result.
//
// The class is intentionally standard-C++17 only and does not depend on PCL,
// nanoflann, KD trees, meshes, or a global 3D voxel map.  It is not thread-safe.
class TerrainGrid {
 public:
  struct Params {
    float cell_size = 0.3f;
    float z_tolerance = 0.2f;
    float max_query_radius = 100.0f;
    std::size_t max_query_cells = 262144;
  };

  TerrainGrid() = default;
  explicit TerrainGrid(const Params& params);

  // Validates params.  On success the grid is cleared and the new params are
  // installed; on failure params and existing data are unchanged.
  bool setParams(const Params& params);
  const Params& params() const { return params_; }

  // Build from any input range whose elements expose x/y/z.  Every element
  // consumes one original index; non-finite points are skipped without
  // shifting the indices of later points.
  template <typename InputIt>
  TerrainBuildStatus build(InputIt first, InputIt last) {
    std::vector<TerrainPointXYZ> points;
    for (; first != last; ++first) {
      const auto& p = *first;
      points.push_back(TerrainPointXYZ{static_cast<float>(p.x),
                                       static_cast<float>(p.y),
                                       static_cast<float>(p.z)});
    }
    return build(points.data(), points.size());
  }

  template <typename PointT>
  TerrainBuildStatus build(const std::vector<PointT>& points) {
    return build(points.begin(), points.end());
  }

  // Index i in points maps to original index i in every result.  A null
  // pointer with count > 0 is kInvalidInput; count > INT_MAX is
  // kTooManyPoints because the query interface returns vector<int>.
  TerrainBuildStatus build(const TerrainPointXYZ* points, std::size_t count);

  void clear();

  bool built() const { return built_; }
  bool empty() const { return point_count_ == 0; }
  std::size_t pointCount() const { return point_count_; }
  std::size_t cellCount() const { return cells_.size(); }
  std::size_t surfaceCount() const { return surface_count_; }
  std::size_t skippedNonFiniteCount() const { return skipped_non_finite_; }
  std::size_t skippedOutOfRangeCount() const { return skipped_out_of_range_; }

  // Metadata access.  getCell/getCellAt return false for unknown/empty cells
  // (or a null output pointer) and leave *cell untouched.
  bool getCell(int ix, int iy, TerrainCell* cell) const;
  bool getCellAt(float x, float y, TerrainCell* cell) const;
  std::vector<TerrainCell> cells() const;  // sorted by (ix, iy)

  // Exact 3D radius query.  Radius semantics match the existing nanoflann/PCL
  // wrapper: squared distance must be strictly less than radius*radius, so a
  // point exactly on the radius boundary is excluded.  Results are sorted by
  // (squared distance, original index).
  //
  // The struct overload always reports a status.  The int overload returns the
  // number of hits (>= 0, including a legal empty result) or -1 on failure;
  // status, when non-null, distinguishes kNotBuilt / kInvalidQuery /
  // kQueryTooLarge from a successful empty result.  Both clear the output
  // vectors on every call.
  TerrainQueryResult radiusSearch(const TerrainPointXYZ& query,
                                  float radius) const;

  int radiusSearch(const TerrainPointXYZ& query, float radius,
                   std::vector<int>& indices,
                   std::vector<float>& squared_distances,
                   TerrainQueryStatus* status = nullptr) const;

  template <typename PointT>
  int radiusSearch(const PointT& query, float radius,
                   std::vector<int>& indices,
                   std::vector<float>& squared_distances,
                   TerrainQueryStatus* status = nullptr) const {
    return radiusSearch(
        TerrainPointXYZ{static_cast<float>(query.x),
                        static_cast<float>(query.y),
                        static_cast<float>(query.z)},
        radius, indices, squared_distances, status);
  }

  static const char* toString(TerrainBuildStatus status);
  static const char* toString(TerrainQueryStatus status);

 private:
  struct CellKey {
    std::int64_t ix = 0;
    std::int64_t iy = 0;

    bool operator==(const CellKey& other) const {
      return ix == other.ix && iy == other.iy;
    }
  };

  struct CellKeyHash {
    std::size_t operator()(const CellKey& key) const;
  };

  struct StoredPoint {
    int index = 0;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
  };

  struct SurfaceData {
    float z_min = 0.0f;
    float z_max = 0.0f;
    float z_mean = 0.0f;
    int point_count = 0;
    std::vector<StoredPoint> points;
  };

  struct CellData {
    int ix = 0;
    int iy = 0;
    int point_count = 0;
    float z_min = 0.0f;
    float z_max = 0.0f;
    float z_mean = 0.0f;
    std::vector<StoredPoint> points;  // scratch used during build finalisation
    std::vector<SurfaceData> surfaces;
  };

  bool validParams() const;
  static bool validParams(const Params& params);
  bool cellCoordinate(float value, std::int64_t* out) const;
  void insertPointInternal(int index, const TerrainPointXYZ& point);
  void finalizeBuild();
  static TerrainCell toPublicCell(const CellData& cell);

  Params params_;
  bool built_ = false;
  std::size_t point_count_ = 0;
  std::size_t surface_count_ = 0;
  std::size_t skipped_non_finite_ = 0;
  std::size_t skipped_out_of_range_ = 0;
  std::unordered_map<CellKey, CellData, CellKeyHash> cells_;
};

}  // namespace global_planner

#endif  // GLOBAL_PLANNER_TERRAIN_GRID_H
