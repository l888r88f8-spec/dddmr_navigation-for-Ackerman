#include "dddmr_local_map/voxel_map.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace dddmr_local_map {
namespace {

uint64_t splitmix64(uint64_t value) {
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31U);
}

int64_t floorMod(int64_t value, int64_t modulus) {
  int64_t result = value % modulus;
  if (result < 0) {
    result += modulus;
  }
  return result;
}

constexpr double kMaxAbsVoxelCoordinate = 9.0e15;

// Converts a finite world coordinate to a voxel index without invoking
// undefined behavior for huge (but finite) doubles.
bool safeFloorVoxel(double coordinate, double voxel_size, int64_t* out) {
  if (!std::isfinite(coordinate) || !std::isfinite(voxel_size) ||
      voxel_size <= 0.0) {
    return false;
  }
  const double scaled = std::floor(coordinate / voxel_size);
  if (!std::isfinite(scaled) || std::abs(scaled) > kMaxAbsVoxelCoordinate) {
    return false;
  }
  *out = static_cast<int64_t>(scaled);
  return true;
}

bool finite(const Vec3& value) {
  return value.allFinite();
}

using AxisRange = std::pair<int64_t, int64_t>;  // [first, second)
using AxisRanges = std::array<std::vector<AxisRange>, 3>;

template <typename Function>
void forEachCellInRanges(const AxisRanges& ranges, Function&& function) {
  for (const auto& xr : ranges[0]) {
    for (const auto& yr : ranges[1]) {
      for (const auto& zr : ranges[2]) {
        for (int64_t x = xr.first; x < xr.second; ++x) {
          for (int64_t y = yr.first; y < yr.second; ++y) {
            for (int64_t z = zr.first; z < zr.second; ++z) {
              function(x, y, z);
            }
          }
        }
      }
    }
  }
}

bool clipSegmentToBounds(const Vec3& origin, const Vec3& delta,
                         const Vec3& lower, const Vec3& upper, double* t0,
                         double* t1) {
  double enter = 0.0;
  double exit = 1.0;
  for (int axis = 0; axis < 3; ++axis) {
    const double d = delta[axis];
    const double p = origin[axis];
    if (std::abs(d) < 1e-12) {
      if (p < lower[axis] || p > upper[axis]) {
        return false;
      }
      continue;
    }
    const double inv = 1.0 / d;
    double a = (lower[axis] - p) * inv;
    double b = (upper[axis] - p) * inv;
    if (!std::isfinite(a) || !std::isfinite(b)) {
      return false;
    }
    if (a > b) {
      std::swap(a, b);
    }
    enter = std::max(enter, a);
    exit = std::min(exit, b);
    if (enter > exit) {
      return false;
    }
  }
  *t0 = enter;
  *t1 = exit;
  return true;
}

template <typename AddCell>
void traverseSegment(const Vec3& start, const Vec3& end, double voxel_size,
                     const std::array<int64_t, 3>& lower,
                     const std::array<int64_t, 3>& upper, AddCell&& add_cell) {
  const Vec3 delta = end - start;
  const std::array<int64_t, 3> dims{upper[0] - lower[0], upper[1] - lower[1],
                                    upper[2] - lower[2]};
  std::array<int64_t, 3> index{};
  std::array<int, 3> step{0, 0, 0};
  std::array<double, 3> t_max{std::numeric_limits<double>::infinity(),
                              std::numeric_limits<double>::infinity(),
                              std::numeric_limits<double>::infinity()};
  std::array<double, 3> t_delta{std::numeric_limits<double>::infinity(),
                                std::numeric_limits<double>::infinity(),
                                std::numeric_limits<double>::infinity()};
  for (int axis = 0; axis < 3; ++axis) {
    int64_t cell = 0;
    if (!safeFloorVoxel(start[axis], voxel_size, &cell)) {
      return;
    }
    cell = std::clamp(cell, lower[axis], upper[axis] - 1);
    index[axis] = cell;
    if (std::abs(delta[axis]) < 1e-12) {
      continue;
    }
    if (delta[axis] > 0.0) {
      step[axis] = 1;
      t_max[axis] =
          ((static_cast<double>(cell) + 1.0) * voxel_size - start[axis]) /
          delta[axis];
      t_delta[axis] = voxel_size / delta[axis];
    } else {
      step[axis] = -1;
      t_max[axis] = (static_cast<double>(cell) * voxel_size - start[axis]) /
                    delta[axis];
      t_delta[axis] = -voxel_size / delta[axis];
    }
  }

  // The circular storage index is (x mod nx, y mod ny, z mod nz). Compute the
  // per-axis residues once for the start cell, then advance each DDA step in
  // its own axis. Wrapping is per axis: a z step at the end of a row returns to
  // z=0 of the SAME (x,y) row; it must not carry into y (or x).
  std::array<std::size_t, 3> circular{
      static_cast<std::size_t>(floorMod(index[0], dims[0])),
      static_cast<std::size_t>(floorMod(index[1], dims[1])),
      static_cast<std::size_t>(floorMod(index[2], dims[2]))};
  const auto linearIndex = [&]() {
    return (circular[0] * static_cast<std::size_t>(dims[1]) + circular[1]) *
               static_cast<std::size_t>(dims[2]) +
           circular[2];
  };
  add_cell(index, linearIndex());
  constexpr int kMaxSteps = 100000;
  for (int iteration = 0; iteration < kMaxSteps; ++iteration) {
    int axis = 0;
    if (t_max[1] < t_max[axis]) {
      axis = 1;
    }
    if (t_max[2] < t_max[axis]) {
      axis = 2;
    }
    if (!std::isfinite(t_max[axis]) || t_max[axis] > 1.0 + 1e-12) {
      break;
    }
    index[axis] += step[axis];
    if (index[axis] < lower[axis] || index[axis] >= upper[axis]) {
      break;
    }
    t_max[axis] += t_delta[axis];
    const std::size_t dim = static_cast<std::size_t>(dims[axis]);
    if (step[axis] > 0) {
      circular[axis] = circular[axis] + 1U == dim ? 0U : circular[axis] + 1U;
    } else {
      circular[axis] = circular[axis] == 0U ? dim - 1U : circular[axis] - 1U;
    }
    add_cell(index, linearIndex());
  }
}

bool checkedVoxelProduct(const MapConfig& config, std::size_t* out) {
  std::size_t product = 1;
  const std::array<int, 3> dims{config.window_x, config.window_y,
                                config.window_z};
  for (const int dim : dims) {
    if (dim <= 0) {
      return false;
    }
    const std::size_t d = static_cast<std::size_t>(dim);
    if (config.max_voxels == 0 || d > config.max_voxels / product) {
      return false;
    }
    product *= d;
  }
  *out = product;
  return true;
}

}  // namespace

std::size_t VoxelKeyHash::operator()(const VoxelKey& key) const noexcept {
  uint64_t seed = splitmix64(static_cast<uint64_t>(key.x));
  seed ^= splitmix64(static_cast<uint64_t>(key.y) + 0x9e3779b9U) +
          (seed << 6U) + (seed >> 2U);
  seed ^= splitmix64(static_cast<uint64_t>(key.z) + 0x85ebca6bU) +
          (seed << 6U) + (seed >> 2U);
  return static_cast<std::size_t>(seed);
}

std::size_t ColumnKeyHash::operator()(const ColumnKey& key) const noexcept {
  uint64_t seed = splitmix64(static_cast<uint64_t>(key.x));
  seed ^= splitmix64(static_cast<uint64_t>(key.y) + 0xc2b2ae3dU) +
          (seed << 6U) + (seed >> 2U);
  return static_cast<std::size_t>(seed);
}

const char* toString(Occupancy value) {
  switch (value) {
    case Occupancy::kUnknown:
      return "unknown";
    case Occupancy::kFree:
      return "free";
    case Occupancy::kOccupied:
      return "occupied";
  }
  return "invalid";
}

Validation MapConfig::validate() const {
  if (!std::isfinite(voxel_size) || voxel_size <= 0.0) {
    return Validation::failure("voxel_size must be finite and > 0");
  }
  if (voxel_size > 10.0) {
    return Validation::failure("voxel_size is unreasonably large");
  }
  std::size_t cell_count = 0;
  if (!checkedVoxelProduct(*this, &cell_count)) {
    return Validation::failure(
        "window dimensions must be positive and their product must not "
        "exceed max_voxels");
  }
  constexpr int kMaxWindowDimension = 4096;
  if (window_x > kMaxWindowDimension || window_y > kMaxWindowDimension ||
      window_z > kMaxWindowDimension) {
    return Validation::failure(
        "window dimensions exceed the physically meaningful limit");
  }
  if (max_voxels > 50'000'000U) {
    return Validation::failure("max_voxels exceeds the hard safety limit");
  }
  if (!std::isfinite(max_range) || max_range <= 0.0) {
    return Validation::failure("max_range must be finite and > 0");
  }
  constexpr double kMaxPhysicalRange = 1000.0;
  if (max_range > kMaxPhysicalRange) {
    return Validation::failure("max_range exceeds the physical safety limit");
  }
  if (!std::isfinite(hit_log_odds) || hit_log_odds <= 0.0f ||
      !std::isfinite(miss_log_odds) || miss_log_odds >= 0.0f) {
    return Validation::failure(
        "hit_log_odds must be > 0 and miss_log_odds must be < 0");
  }
  if (!std::isfinite(occupied_threshold) || occupied_threshold <= 0.0f ||
      !std::isfinite(free_threshold) || free_threshold >= 0.0f) {
    return Validation::failure(
        "occupied_threshold must be > 0 and free_threshold must be < 0");
  }
  if (hit_log_odds <= occupied_threshold) {
    return Validation::failure(
        "a single hit must be able to cross occupied_threshold");
  }
  if (miss_log_odds >= free_threshold) {
    return Validation::failure(
        "a single miss must be able to cross free_threshold");
  }
  if (!std::isfinite(min_log_odds) || !std::isfinite(max_log_odds) ||
      min_log_odds >= free_threshold || max_log_odds <= occupied_threshold ||
      min_log_odds >= 0.0f || max_log_odds <= 0.0f) {
    return Validation::failure(
        "log-odds clamp bounds must straddle both thresholds and zero");
  }
  if (!std::isfinite(inflation_radius) || inflation_radius < 0.0) {
    return Validation::failure(
        "inflation_radius must be finite and non-negative");
  }
  constexpr double kMaxPhysicalInflationRadius = 10.0;
  if (inflation_radius > kMaxPhysicalInflationRadius) {
    return Validation::failure(
        "inflation_radius exceeds the physical safety limit");
  }
  return Validation::success();
}

// One circular-buffer cell is the public flat POD so a snapshot copy and a
// caller-owned export share exactly one layout with no conversion.
using VoxelCell = MapCellExport;

struct VoxelMap::Impl {
  using Cell = VoxelCell;

  // Per-frame collection records. They replace the former hash containers so
  // the hot path never allocates per cell and teardown is O(1). Records are
  // reused across calls; only the epoch tags are reset.
  struct FreeRecord {
    int64_t x{0};
    int64_t y{0};
    int64_t z{0};
    std::size_t index{0};
  };
  struct HitRecord {
    int64_t x{0};
    int64_t y{0};
    int64_t z{0};
    std::size_t index{0};
    double sum_x{0.0};
    double sum_y{0.0};
    double sum_z{0.0};
    double max_z{-std::numeric_limits<double>::infinity()};
    std::size_t count{0};
  };

  MapConfig config;
  std::string frame;
  std::array<int64_t, 3> origin{0, 0, 0};
  std::vector<Cell> cells;
  std::shared_ptr<const MapSnapshot> snapshot;
  double last_stamp{kNaN};
  uint64_t version{0};
  bool initialized{false};

  // Dense epoch tags make unique-cell collection a single array test instead
  // of a hash lookup. Epoch wraps after 2^32 frames (decades at 9 Hz); on wrap
  // the tag arrays are cleared, so uniqueness is never ambiguous.
  uint32_t evidence_epoch{0};
  std::vector<uint32_t> free_epoch;
  std::vector<uint32_t> hit_epoch;
  // Index into hit_records, valid only when hit_epoch[index] == evidence_epoch.
  std::vector<int32_t> hit_slot;
  std::vector<FreeRecord> free_records;
  std::vector<HitRecord> hit_records;
  // Maintained exactly like the old full-window recount: every stored cell,
  // including stale out-of-window cells still present in the circular buffer,
  // counts when observed && log_odds >= occupied_threshold.
  std::size_t occupied_count{0};

  uint32_t nextEvidenceEpoch() {
    ++evidence_epoch;
    if (evidence_epoch == 0U) {
      std::fill(free_epoch.begin(), free_epoch.end(), 0U);
      std::fill(hit_epoch.begin(), hit_epoch.end(), 0U);
      evidence_epoch = 1U;
    }
    return evidence_epoch;
  }

  int nx() const { return config.window_x; }
  int ny() const { return config.window_y; }
  int nz() const { return config.window_z; }

  bool inBoundsAbs(int64_t x, int64_t y, int64_t z) const {
    return x >= origin[0] && x < origin[0] + nx() && y >= origin[1] &&
           y < origin[1] + ny() && z >= origin[2] && z < origin[2] + nz();
  }

  // Circular storage uses the absolute world voxel index modulo the window
  // dimensions. Changing origin therefore never remaps an overlapping cell.
  std::size_t linearAbs(int64_t x, int64_t y, int64_t z) const {
    const int64_t rx = floorMod(x, nx());
    const int64_t ry = floorMod(y, ny());
    const int64_t rz = floorMod(z, nz());
    return static_cast<std::size_t>((rx * ny() + ry) * nz() + rz);
  }

  Cell& cellAbs(int64_t x, int64_t y, int64_t z) {
    return cells[linearAbs(x, y, z)];
  }

  const Cell& cellAbs(int64_t x, int64_t y, int64_t z) const {
    return cells[linearAbs(x, y, z)];
  }

  bool baseOccupied(const Cell& cell) const {
    return cell.observed != 0 &&
           cell.log_odds >= config.occupied_threshold;
  }

  // Bound the stencil so a pathological (but validated finite) radius cannot
  // create an enormous loop. 256 voxels is already much wider than the local
  // planner body and remains a conservative superset inside that reach.
  int inflationReach() const {
    const int cap = std::min({nx(), ny(), nz(), 256});
    const double radius_vox = config.inflation_radius / config.voxel_size;
    if (!std::isfinite(radius_vox) || radius_vox <= 0.0) {
      return radius_vox > 0.0 ? cap : 0;
    }
    return std::min(static_cast<int>(std::ceil(radius_vox)), cap);
  }

  void changeInflationAbs(int64_t x, int64_t y, int64_t z, bool add) {
    const int reach = inflationReach();
    if (reach <= 0) {
      return;
    }
    const double radius_vox = config.inflation_radius / config.voxel_size;
    const double limited_radius =
        std::min(radius_vox, static_cast<double>(reach));
    const double radius_squared = limited_radius * limited_radius;
    for (int dx = -reach; dx <= reach; ++dx) {
      for (int dy = -reach; dy <= reach; ++dy) {
        for (int dz = -reach; dz <= reach; ++dz) {
          const double distance_squared =
              static_cast<double>(dx) * dx + static_cast<double>(dy) * dy +
              static_cast<double>(dz) * dz;
          if (distance_squared > radius_squared + 1e-9) {
            continue;
          }
          const int64_t ix = x + dx;
          const int64_t iy = y + dy;
          const int64_t iz = z + dz;
          if (!inBoundsAbs(ix, iy, iz)) {
            continue;
          }
          Cell& target = cellAbs(ix, iy, iz);
          if (add) {
            if (target.inflation < std::numeric_limits<uint32_t>::max()) {
              ++target.inflation;
            }
          } else if (target.inflation > 0) {
            --target.inflation;
          }
        }
      }
    }
  }

  void applyEvidenceCell(Cell& cell, int64_t x, int64_t y, int64_t z,
                         float delta, double stamp, bool hit,
                         const Vec3& hit_position, double hit_max_z) {
    const bool was_occupied = baseOccupied(cell);
    if (cell.observed == 0) {
      cell.observed = 1;
      cell.log_odds = 0.0f;
    }
    float updated = cell.log_odds + delta;
    updated = std::max(config.min_log_odds,
                       std::min(config.max_log_odds, updated));
    cell.log_odds = updated;
    // Evidence is real even when the log-odds value is clamped. Keep HIT and
    // MISS times separate: a later miss invalidates support even while the
    // clamped log-odds still reads occupied.
    cell.last_observed_stamp = stamp;
    if (hit) {
      cell.last_hit_stamp = stamp;
      cell.hit_x = static_cast<float>(hit_position.x());
      cell.hit_y = static_cast<float>(hit_position.y());
      cell.hit_z = static_cast<float>(hit_position.z());
      cell.hit_max_z = static_cast<float>(hit_max_z);
    } else {
      cell.last_miss_stamp = stamp;
    }
    const bool now_occupied = baseOccupied(cell);
    if (now_occupied != was_occupied) {
      if (now_occupied) {
        ++occupied_count;
      } else {
        --occupied_count;
      }
      changeInflationAbs(x, y, z, now_occupied);
    }
  }

  // Resetting a cell is only used for cells newly entering the window. Their
  // circular slots can still hold a stale departed cell, which the old
  // full-window recount would have counted, so keep the counter exact.
  void resetCellAbs(int64_t x, int64_t y, int64_t z) {
    Cell& cell = cellAbs(x, y, z);
    if (baseOccupied(cell) && occupied_count > 0) {
      --occupied_count;
    }
    cell = Cell{};
  }

  bool recenter(const Vec3& center, std::string* error, bool* changed) {
    *changed = false;
    if (!finite(center)) {
      if (error) {
        *error = "window center must be finite";
      }
      return false;
    }
    std::array<int64_t, 3> target{};
    const std::array<int, 3> dims{nx(), ny(), nz()};
    for (int axis = 0; axis < 3; ++axis) {
      const double quotient = center[axis] / config.voxel_size;
      if (!std::isfinite(quotient) ||
          std::abs(quotient) > kMaxAbsVoxelCoordinate) {
        if (error) {
          *error = "window center overflows voxel coordinates";
        }
        return false;
      }
      const int64_t center_voxel =
          static_cast<int64_t>(std::floor(quotient));
      target[axis] = center_voxel - dims[axis] / 2;
    }

    if (initialized && target == origin) {
      return true;
    }

    if (!initialized) {
      origin = target;
      initialized = true;
      *changed = true;
      return true;
    }

    const auto old_origin = origin;
    std::array<AxisRange, 3> overlap{};
    bool disjoint = false;
    for (int axis = 0; axis < 3; ++axis) {
      const int64_t overlap_lo =
          std::max(old_origin[axis], target[axis]);
      const int64_t overlap_hi =
          std::min(old_origin[axis] + dims[axis], target[axis] + dims[axis]);
      overlap[axis] = AxisRange{overlap_lo, overlap_hi};
      if (overlap_lo >= overlap_hi) {
        disjoint = true;
      }
    }

    // A genuinely disjoint slide shares no world cell, so a full reset is the
    // only correct action and cannot discard retained evidence.
    if (disjoint) {
      for (auto& cell : cells) {
        cell = Cell{};
      }
      occupied_count = 0;
      origin = target;
      *changed = true;
      return true;
    }

    std::array<std::vector<AxisRange>, 3> new_diff;
    std::array<std::vector<AxisRange>, 3> old_diff;
    for (int axis = 0; axis < 3; ++axis) {
      const int64_t new_lo = target[axis];
      const int64_t new_hi = target[axis] + dims[axis];
      const int64_t old_lo = old_origin[axis];
      const int64_t old_hi = old_origin[axis] + dims[axis];
      if (new_lo < overlap[axis].first) {
        new_diff[axis].emplace_back(new_lo, overlap[axis].first);
      }
      if (overlap[axis].second < new_hi) {
        new_diff[axis].emplace_back(overlap[axis].second, new_hi);
      }
      if (old_lo < overlap[axis].first) {
        old_diff[axis].emplace_back(old_lo, overlap[axis].first);
      }
      if (overlap[axis].second < old_hi) {
        old_diff[axis].emplace_back(overlap[axis].second, old_hi);
      }
    }

    // The box difference is a union of per-axis slabs. Ordering the axes makes
    // the slabs disjoint, so every entering/leaving cell is visited once.
    std::vector<AxisRanges> entering_slabs;
    std::vector<AxisRanges> outgoing_slabs;
    for (int axis = 0; axis < 3; ++axis) {
      if (new_diff[axis].empty()) {
        continue;
      }
      AxisRanges new_slab;
      AxisRanges old_slab;
      for (int a = 0; a < axis; ++a) {
        new_slab[a].push_back(overlap[a]);
        old_slab[a].push_back(overlap[a]);
      }
      new_slab[axis] = new_diff[axis];
      old_slab[axis] = old_diff[axis];
      for (int a = axis + 1; a < 3; ++a) {
        new_slab[a].emplace_back(target[a], target[a] + dims[a]);
        old_slab[a].emplace_back(old_origin[a], old_origin[a] + dims[a]);
      }
      entering_slabs.push_back(std::move(new_slab));
      outgoing_slabs.push_back(std::move(old_slab));
    }

    // Remove the influence of obstacles that leave the window while the old
    // origin is still active for bounds checks. Retained cells then keep only
    // the contributions of retained obstacles.
    for (const auto& slab : outgoing_slabs) {
      forEachCellInRanges(slab, [&](int64_t x, int64_t y, int64_t z) {
        if (baseOccupied(cellAbs(x, y, z))) {
          changeInflationAbs(x, y, z, false);
        }
      });
    }

    // Recycle exactly the entering slabs; overlapping cells are never copied
    // or remapped because storage is indexed by world index modulo dimensions.
    for (const auto& slab : entering_slabs) {
      forEachCellInRanges(slab, [&](int64_t x, int64_t y, int64_t z) {
        resetCellAbs(x, y, z);
      });
    }

    origin = target;

    // Restore inflation into the freshly recycled slabs from retained
    // obstacles. Incoming slabs have no evidence yet, so the count can be set
    // directly from the retained occupied neighbours.
    const int reach = inflationReach();
    if (reach > 0) {
      const double radius_vox = config.inflation_radius / config.voxel_size;
      const double limited_radius =
          std::min(radius_vox, static_cast<double>(reach));
      const double radius_squared = limited_radius * limited_radius;
      for (const auto& slab : entering_slabs) {
        forEachCellInRanges(slab, [&](int64_t x, int64_t y, int64_t z) {
          uint32_t count = 0;
          for (int dx = -reach; dx <= reach; ++dx) {
            for (int dy = -reach; dy <= reach; ++dy) {
              for (int dz = -reach; dz <= reach; ++dz) {
                const double distance_squared =
                    static_cast<double>(dx) * dx +
                    static_cast<double>(dy) * dy +
                    static_cast<double>(dz) * dz;
                if (distance_squared > radius_squared + 1e-9) {
                  continue;
                }
                const int64_t ix = x + dx;
                const int64_t iy = y + dy;
                const int64_t iz = z + dz;
                if (!inBoundsAbs(ix, iy, iz)) {
                  continue;
                }
                if (baseOccupied(cellAbs(ix, iy, iz))) {
                  ++count;
                }
              }
            }
          }
          cellAbs(x, y, z).inflation = count;
        });
      }
    }

    *changed = true;
    return true;
  }
};

struct MapSnapshot::Impl {
  MapConfig config;
  std::string frame;
  double stamp{kNaN};
  uint64_t version{0};
  std::array<int64_t, 3> origin{0, 0, 0};
  Vec3i dims{0, 0, 0};
  // One immutable copy of the mutable cell buffer. Same fields, same circular
  // indexing, so every public accessor keeps its exact prior semantics.
  std::vector<VoxelCell> cells;
  std::size_t occupied_count{0};

  bool inBoundsVoxel(int64_t x, int64_t y, int64_t z) const {
    return x >= origin[0] && x < origin[0] + dims.x() && y >= origin[1] &&
           y < origin[1] + dims.y() && z >= origin[2] &&
           z < origin[2] + dims.z();
  }

  // Must exactly match VoxelMap::Impl: absolute world voxel index modulo
  // dimensions, independent of the current origin.
  std::size_t linearAbs(int64_t x, int64_t y, int64_t z) const {
    const int64_t rx = floorMod(x, dims.x());
    const int64_t ry = floorMod(y, dims.y());
    const int64_t rz = floorMod(z, dims.z());
    return static_cast<std::size_t>((rx * dims.y() + ry) * dims.z() + rz);
  }
};

MapSnapshot::MapSnapshot(std::unique_ptr<Impl> impl)
    : impl_(std::move(impl)) {}
MapSnapshot::~MapSnapshot() = default;

const std::string& MapSnapshot::frame() const { return impl_->frame; }
double MapSnapshot::stamp() const { return impl_->stamp; }
uint64_t MapSnapshot::version() const { return impl_->version; }
const MapConfig& MapSnapshot::config() const { return impl_->config; }
std::array<int64_t, 3> MapSnapshot::originVoxel() const {
  return impl_->origin;
}
Vec3 MapSnapshot::originWorld() const {
  return Vec3(static_cast<double>(impl_->origin[0]),
              static_cast<double>(impl_->origin[1]),
              static_cast<double>(impl_->origin[2])) *
         impl_->config.voxel_size;
}
Vec3i MapSnapshot::windowDims() const { return impl_->dims; }

std::vector<MapCellExport> MapSnapshot::exportCells() const {
  return impl_->cells;
}

bool MapSnapshot::inBounds(const Vec3& point) const {
  if (!finite(point)) {
    return false;
  }
  const Vec3 lower = originWorld();
  const Vec3 upper =
      lower + impl_->dims.cast<double>() * impl_->config.voxel_size;
  return point.x() >= lower.x() && point.x() < upper.x() &&
         point.y() >= lower.y() && point.y() < upper.y() &&
         point.z() >= lower.z() && point.z() < upper.z();
}

MapLookup MapSnapshot::lookupVoxel(int64_t x, int64_t y, int64_t z,
                                   bool include_inflation) const {
  MapLookup result;
  result.last_observed_stamp = kNaN;
  result.last_hit_stamp = kNaN;
  result.last_miss_stamp = kNaN;
  result.hit_position = Vec3(kNaN, kNaN, kNaN);
  result.hit_max_z = kNaN;
  if (!impl_->inBoundsVoxel(x, y, z)) {
    return result;
  }
  result.in_bounds = true;
  const std::size_t index = impl_->linearAbs(x, y, z);
  const VoxelCell& cell = impl_->cells[index];
  result.observed = cell.observed != 0;
  if (result.observed) {
    result.last_observed_stamp = cell.last_observed_stamp;
    result.last_hit_stamp = cell.last_hit_stamp;
    result.last_miss_stamp = cell.last_miss_stamp;
    if (std::isfinite(cell.hit_x) && std::isfinite(cell.hit_y) &&
        std::isfinite(cell.hit_z)) {
      result.hit_position = Vec3(static_cast<double>(cell.hit_x),
                                 static_cast<double>(cell.hit_y),
                                 static_cast<double>(cell.hit_z));
      result.hit_max_z = static_cast<double>(cell.hit_max_z);
    }
  }
  if (cell.observed == 0) {
    result.occupancy = Occupancy::kUnknown;
  } else if (cell.log_odds >= impl_->config.occupied_threshold) {
    result.occupancy = Occupancy::kOccupied;
  } else if (cell.log_odds <= impl_->config.free_threshold) {
    result.occupancy = Occupancy::kFree;
  } else {
    result.occupancy = Occupancy::kUnknown;
  }
  if (include_inflation && cell.inflation > 0U &&
      result.occupancy != Occupancy::kOccupied) {
    result.inflated = true;
    result.occupancy = Occupancy::kOccupied;
  }
  return result;
}

MapLookup MapSnapshot::lookup(const Vec3& point,
                              bool include_inflation) const {
  if (!inBounds(point)) {
    MapLookup result;
    result.last_observed_stamp = kNaN;
    result.last_hit_stamp = kNaN;
    result.last_miss_stamp = kNaN;
    result.hit_position = Vec3(kNaN, kNaN, kNaN);
    result.hit_max_z = kNaN;
    return result;
  }
  int64_t x = 0;
  int64_t y = 0;
  int64_t z = 0;
  if (!safeFloorVoxel(point.x(), impl_->config.voxel_size, &x) ||
      !safeFloorVoxel(point.y(), impl_->config.voxel_size, &y) ||
      !safeFloorVoxel(point.z(), impl_->config.voxel_size, &z)) {
    MapLookup result;
    result.last_observed_stamp = kNaN;
    result.last_hit_stamp = kNaN;
    result.last_miss_stamp = kNaN;
    result.hit_position = Vec3(kNaN, kNaN, kNaN);
    result.hit_max_z = kNaN;
    return result;
  }
  return lookupVoxel(x, y, z, include_inflation);
}

float MapSnapshot::logOddsAt(const Vec3& point) const {
  if (!inBounds(point)) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  int64_t x = 0;
  int64_t y = 0;
  int64_t z = 0;
  if (!safeFloorVoxel(point.x(), impl_->config.voxel_size, &x) ||
      !safeFloorVoxel(point.y(), impl_->config.voxel_size, &y) ||
      !safeFloorVoxel(point.z(), impl_->config.voxel_size, &z) ||
      !impl_->inBoundsVoxel(x, y, z)) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  return impl_->cells[impl_->linearAbs(x, y, z)].log_odds;
}

int MapSnapshot::inflationRefCountAt(const Vec3& point) const {
  if (!inBounds(point)) {
    return -1;
  }
  int64_t x = 0;
  int64_t y = 0;
  int64_t z = 0;
  if (!safeFloorVoxel(point.x(), impl_->config.voxel_size, &x) ||
      !safeFloorVoxel(point.y(), impl_->config.voxel_size, &y) ||
      !safeFloorVoxel(point.z(), impl_->config.voxel_size, &z) ||
      !impl_->inBoundsVoxel(x, y, z)) {
    return -1;
  }
  return static_cast<int>(impl_->cells[impl_->linearAbs(x, y, z)].inflation);
}

std::size_t MapSnapshot::occupiedCount() const {
  return impl_->occupied_count;
}

bool MapSnapshot::columnObserved(double x, double y) const {
  int64_t ix = 0;
  int64_t iy = 0;
  if (!safeFloorVoxel(x, impl_->config.voxel_size, &ix) ||
      !safeFloorVoxel(y, impl_->config.voxel_size, &iy)) {
    return false;
  }
  if (ix < impl_->origin[0] || ix >= impl_->origin[0] + impl_->dims.x() ||
      iy < impl_->origin[1] || iy >= impl_->origin[1] + impl_->dims.y()) {
    return false;
  }
  for (int64_t iz = impl_->origin[2];
       iz < impl_->origin[2] + impl_->dims.z(); ++iz) {
    if (impl_->cells[impl_->linearAbs(ix, iy, iz)].observed != 0) {
      return true;
    }
  }
  return false;
}

VoxelMap::VoxelMap(MapConfig config, std::string frame)
    : impl_(std::make_unique<Impl>()) {
  const Validation validation = config.validate();
  if (!validation.ok) {
    throw std::invalid_argument("invalid MapConfig: " + validation.error);
  }
  impl_->config = std::move(config);
  impl_->frame = std::move(frame);
  std::size_t count = 0;
  checkedVoxelProduct(impl_->config, &count);
  impl_->cells.assign(count, Impl::Cell{});
  impl_->free_epoch.assign(count, 0U);
  impl_->hit_epoch.assign(count, 0U);
  impl_->hit_slot.assign(count, -1);
  const std::size_t record_capacity = std::min(count, std::size_t{262144});
  impl_->free_records.reserve(record_capacity);
  impl_->hit_records.reserve(std::min(count, std::size_t{65536}));
}

VoxelMap::~VoxelMap() = default;

bool VoxelMap::setWindowCenter(const Vec3& center, std::string* error) {
  bool changed = false;
  if (!impl_->recenter(center, error, &changed)) {
    return false;
  }
  // A repeated center is a no-op: do not create a duplicate snapshot or force
  // downstream terrain work.
  if (changed) {
    publishSnapshot(impl_->last_stamp);
  }
  return true;
}

std::shared_ptr<const MapSnapshot> VoxelMap::publishSnapshot(double stamp) {
  auto snapshot_impl = std::make_unique<MapSnapshot::Impl>();
  auto& target = *snapshot_impl;
  target.config = impl_->config;
  target.frame = impl_->frame;
  target.stamp = stamp;
  target.version = ++impl_->version;
  target.origin = impl_->origin;
  target.dims = Vec3i(impl_->config.window_x, impl_->config.window_y,
                      impl_->config.window_z);
  // One contiguous copy of the POD cells. occupied_count is maintained on the
  // same transitions the old full-window recount observed.
  target.cells = impl_->cells;
  target.occupied_count = impl_->occupied_count;
  auto snapshot = std::shared_ptr<MapSnapshot>(
      new MapSnapshot(std::move(snapshot_impl)));
  impl_->snapshot = snapshot;
  return snapshot;
}

MapUpdateResult VoxelMap::integrateRays(const Vec3& origin,
                                        const std::vector<Vec3>& endpoints,
                                        double stamp) {
  std::vector<Ray> rays;
  rays.reserve(endpoints.size());
  for (const auto& endpoint : endpoints) {
    rays.push_back(Ray{origin, endpoint});
  }
  return integrateRays(rays, stamp);
}

MapUpdateResult VoxelMap::integrateRays(const std::vector<Ray>& rays,
                                        double stamp) {
  MapUpdateResult result;
  result.stats.rays = rays.size();
  if (!impl_->initialized) {
    result.error = "window center has not been set";
    return result;
  }
  if (!std::isfinite(stamp)) {
    result.error = "stamp must be finite";
    return result;
  }

  const uint32_t epoch = impl_->nextEvidenceEpoch();
  impl_->free_records.clear();
  impl_->hit_records.clear();

  const auto lower = impl_->origin;
  const std::array<int64_t, 3> upper{
      impl_->origin[0] + impl_->config.window_x,
      impl_->origin[1] + impl_->config.window_y,
      impl_->origin[2] + impl_->config.window_z};
  const Vec3 lower_world =
      Vec3(static_cast<double>(lower[0]), static_cast<double>(lower[1]),
           static_cast<double>(lower[2])) *
      impl_->config.voxel_size;
  const Vec3 upper_world =
      Vec3(static_cast<double>(upper[0]), static_cast<double>(upper[1]),
           static_cast<double>(upper[2])) *
      impl_->config.voxel_size;

  for (const Ray& ray : rays) {
    if (!finite(ray.origin) || !finite(ray.endpoint)) {
      continue;
    }
    ++result.stats.valid_rays;
    Vec3 endpoint = ray.endpoint;
    Vec3 delta = endpoint - ray.origin;
    const double max_abs = delta.cwiseAbs().maxCoeff();
    bool has_hit = false;
    if (!std::isfinite(max_abs)) {
      continue;
    }
    if (max_abs > impl_->config.max_range) {
      // Normalize before comparing/clipping so huge finite endpoints cannot
      // overflow the squared norm.
      const Vec3 unit = delta / max_abs;
      const double unit_norm = unit.norm();
      if (!std::isfinite(unit_norm) || unit_norm <= 0.0) {
        continue;
      }
      endpoint = ray.origin + unit * (impl_->config.max_range / unit_norm);
      delta = endpoint - ray.origin;
    } else {
      const double distance = delta.norm();
      if (!std::isfinite(distance)) {
        continue;
      }
      if (distance <= impl_->config.max_range) {
        has_hit = true;
      } else {
        const double scale = impl_->config.max_range / distance;
        endpoint = ray.origin + delta * scale;
        delta = endpoint - ray.origin;
      }
    }

    int64_t hit_x = 0;
    int64_t hit_y = 0;
    int64_t hit_z = 0;
    if (has_hit) {
      const bool converted =
          safeFloorVoxel(ray.endpoint.x(), impl_->config.voxel_size,
                         &hit_x) &&
          safeFloorVoxel(ray.endpoint.y(), impl_->config.voxel_size,
                         &hit_y) &&
          safeFloorVoxel(ray.endpoint.z(), impl_->config.voxel_size, &hit_z);
      if (!converted || !impl_->inBoundsAbs(hit_x, hit_y, hit_z)) {
        has_hit = false;
      }
    }

    double t0 = 0.0;
    double t1 = 1.0;
    if (!clipSegmentToBounds(ray.origin, delta, lower_world, upper_world, &t0,
                             &t1)) {
      continue;
    }
    const Vec3 start = ray.origin + delta * t0;
    const Vec3 end = ray.origin + delta * t1;
    traverseSegment(
        start, end, impl_->config.voxel_size, lower, upper,
        [&](const std::array<int64_t, 3>& index, std::size_t linear) {
          if (has_hit && index[0] == hit_x && index[1] == hit_y &&
              index[2] == hit_z) {
            return;
          }
          if (impl_->free_epoch[linear] != epoch) {
            impl_->free_epoch[linear] = epoch;
            impl_->free_records.push_back(
                Impl::FreeRecord{index[0], index[1], index[2], linear});
          }
        });
    if (has_hit) {
      const std::size_t hit_linear =
          impl_->linearAbs(hit_x, hit_y, hit_z);
      if (impl_->hit_epoch[hit_linear] != epoch) {
        impl_->hit_epoch[hit_linear] = epoch;
        impl_->hit_slot[hit_linear] =
            static_cast<int32_t>(impl_->hit_records.size());
        impl_->hit_records.push_back(
            Impl::HitRecord{hit_x, hit_y, hit_z, hit_linear});
      }
      auto& accumulator =
          impl_->hit_records[static_cast<std::size_t>(
              impl_->hit_slot[hit_linear])];
      accumulator.sum_x += ray.endpoint.x();
      accumulator.sum_y += ray.endpoint.y();
      accumulator.sum_z += ray.endpoint.z();
      accumulator.max_z = std::max(accumulator.max_z, ray.endpoint.z());
      ++accumulator.count;
    }
  }

  if (result.stats.valid_rays == 0) {
    result.ok = true;
    return result;
  }

  // A cell hit by any ray in this frame receives HIT evidence only; free
  // records for it are dropped exactly like the former set erase.
  std::size_t free_kept = 0;
  for (const auto& record : impl_->free_records) {
    if (impl_->hit_epoch[record.index] == epoch) {
      continue;
    }
    impl_->free_records[free_kept++] = record;
  }
  impl_->free_records.resize(free_kept);
  for (const auto& record : impl_->free_records) {
    impl_->applyEvidenceCell(impl_->cells[record.index], record.x, record.y,
                             record.z, impl_->config.miss_log_odds, stamp,
                             false, Vec3::Zero(), kNaN);
  }
  for (const auto& record : impl_->hit_records) {
    const double count = static_cast<double>(record.count);
    const Vec3 centroid(record.sum_x / count, record.sum_y / count,
                        record.sum_z / count);
    impl_->applyEvidenceCell(impl_->cells[record.index], record.x, record.y,
                             record.z, impl_->config.hit_log_odds, stamp, true,
                             centroid, record.max_z);
  }

  result.stats.free_voxels = impl_->free_records.size();
  result.stats.occupied_voxels = impl_->hit_records.size();
  impl_->last_stamp = stamp;
  publishSnapshot(stamp);
  result.ok = true;
  return result;
}

std::shared_ptr<const MapSnapshot> VoxelMap::snapshot() const {
  return impl_->snapshot;
}

const MapConfig& VoxelMap::config() const { return impl_->config; }
const std::string& VoxelMap::frame() const { return impl_->frame; }

}  // namespace dddmr_local_map
