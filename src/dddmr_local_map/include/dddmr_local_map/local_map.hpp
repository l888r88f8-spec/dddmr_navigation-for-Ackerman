#pragma once

#include <memory>
#include <string>
#include <vector>

#include "dddmr_local_map/terrain.hpp"
#include "dddmr_local_map/voxel_map.hpp"

namespace dddmr_local_map {

struct LocalMapConfig {
  MapConfig map;
  TerrainConfig terrain;
  VehicleGeometry geometry;
};

// Convenience owner that keeps one snapshot and one terrain cache per map
// update. The ROS adapter and MPPI integration should fetch both once at the
// start of a planning cycle and reuse them for every candidate.
class LocalMap {
 public:
  LocalMap(LocalMapConfig config, std::string frame);
  ~LocalMap();
  LocalMap(const LocalMap&) = delete;
  LocalMap& operator=(const LocalMap&) = delete;
  LocalMap(LocalMap&&) = delete;
  LocalMap& operator=(LocalMap&&) = delete;

  bool setWindowCenter(const Vec3& center, std::string* error = nullptr);
  MapUpdateResult integrateRays(const std::vector<Ray>& rays, double stamp);
  MapUpdateResult integrateRays(const Vec3& origin,
                                const std::vector<Vec3>& endpoints,
                                double stamp);

  std::shared_ptr<const MapSnapshot> snapshot() const;
  std::shared_ptr<const TerrainModel> terrain() const;
  TerrainResult evaluate(const TerrainQuery& query) const;
  const LocalMapConfig& config() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace dddmr_local_map
