#pragma once

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "dddmr_local_map/snapshot_export.hpp"
#include "dddmr_local_map/types.hpp"

namespace dddmr_local_map {

struct MapConfig {
  double voxel_size{0.1};
  int window_x{128};
  int window_y{128};
  int window_z{40};
  double max_range{8.0};
  float hit_log_odds{0.85f};
  float miss_log_odds{-0.85f};
  float occupied_threshold{0.4f};
  float free_threshold{-0.4f};
  float min_log_odds{-3.0f};
  float max_log_odds{3.0f};
  double inflation_radius{0.0};
  std::size_t max_voxels{4'000'000};

  Validation validate() const;
};

struct MapUpdateStats {
  std::size_t rays{0};
  std::size_t valid_rays{0};
  std::size_t free_voxels{0};
  std::size_t occupied_voxels{0};
};

struct MapUpdateResult {
  bool ok{false};
  std::string error;
  MapUpdateStats stats;
};

class MapSnapshot {
 public:
  ~MapSnapshot();
  MapSnapshot(const MapSnapshot&) = delete;
  MapSnapshot& operator=(const MapSnapshot&) = delete;
  MapSnapshot(MapSnapshot&&) = delete;
  MapSnapshot& operator=(MapSnapshot&&) = delete;

  const std::string& frame() const;
  double stamp() const;
  uint64_t version() const;
  const MapConfig& config() const;
  std::array<int64_t, 3> originVoxel() const;
  Vec3 originWorld() const;
  Vec3i windowDims() const;
  bool inBounds(const Vec3& point) const;

  // Deep copy of the raw circular-buffer cells in their exact storage order.
  // The returned vector is owned by the caller and stays unchanged when newer
  // snapshots are published. Use originVoxel(), windowDims() and voxel_size to
  // reproduce MapSnapshot::lookupVoxel without approximation.
  std::vector<MapCellExport> exportCells() const;

  MapLookup lookup(const Vec3& point, bool include_inflation = true) const;
  MapLookup lookupVoxel(int64_t x, int64_t y, int64_t z,
                        bool include_inflation = true) const;
  float logOddsAt(const Vec3& point) const;
  int inflationRefCountAt(const Vec3& point) const;
  std::size_t occupiedCount() const;
  bool columnObserved(double x, double y) const;

 private:
  friend class VoxelMap;
  struct Impl;
  explicit MapSnapshot(std::unique_ptr<Impl> impl);

  std::unique_ptr<Impl> impl_;
};

// A bounded, world-aligned 3D log-odds map. Storage is a true circular buffer
// indexed by absolute world voxel index modulo the window dimensions. Moving
// the window keeps overlapping cells at their existing indices, recycles only
// the entering/leaving slabs, and incrementally fixes inflation: outgoing
// obstacles are removed from retained cells and retained obstacles are counted
// into entering cells. A disjoint slide clears the whole buffer.
//
// Each integrateRays call publishes one immutable snapshot by copying the
// whole window exactly once; setWindowCenter does not publish when the center
// did not move. The map owner is not thread-safe; serialize mutating calls.
class VoxelMap {
 public:
  explicit VoxelMap(MapConfig config, std::string frame);
  ~VoxelMap();
  VoxelMap(const VoxelMap&) = delete;
  VoxelMap& operator=(const VoxelMap&) = delete;
  VoxelMap(VoxelMap&&) = delete;
  VoxelMap& operator=(VoxelMap&&) = delete;

  bool setWindowCenter(const Vec3& center, std::string* error = nullptr);
  MapUpdateResult integrateRays(const Vec3& origin,
                                const std::vector<Vec3>& endpoints,
                                double stamp);
  MapUpdateResult integrateRays(const std::vector<Ray>& rays, double stamp);

  std::shared_ptr<const MapSnapshot> snapshot() const;
  const MapConfig& config() const;
  const std::string& frame() const;

 private:
  struct Impl;
  std::shared_ptr<const MapSnapshot> publishSnapshot(double stamp);
  std::unique_ptr<Impl> impl_;
};

}  // namespace dddmr_local_map
