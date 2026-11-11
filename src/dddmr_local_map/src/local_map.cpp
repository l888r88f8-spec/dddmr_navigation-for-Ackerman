#include "dddmr_local_map/local_map.hpp"

#include <memory>
#include <utility>

namespace dddmr_local_map {

struct LocalMap::Impl {
  LocalMapConfig config;
  std::unique_ptr<VoxelMap> map;
  std::shared_ptr<const MapSnapshot> snapshot;
  std::shared_ptr<const TerrainModel> terrain;

  void rebuildTerrain() {
    snapshot = map->snapshot();
    terrain = std::make_shared<TerrainModel>(snapshot, config.terrain,
                                             config.geometry);
  }
};

LocalMap::LocalMap(LocalMapConfig config, std::string frame)
    : impl_(std::make_unique<Impl>()) {
  impl_->config = std::move(config);
  impl_->map = std::make_unique<VoxelMap>(impl_->config.map, std::move(frame));
}

LocalMap::~LocalMap() = default;

bool LocalMap::setWindowCenter(const Vec3& center, std::string* error) {
  // Recentering only prepares the next update. Keep the previous complete
  // (post-integration) snapshot/terrain pair visible until integrateRays
  // produces a new one, so callers never observe a half-updated pair.
  return impl_->map->setWindowCenter(center, error);
}

MapUpdateResult LocalMap::integrateRays(const std::vector<Ray>& rays,
                                        double stamp) {
  MapUpdateResult result = impl_->map->integrateRays(rays, stamp);
  if (result.ok) {
    impl_->rebuildTerrain();
  }
  return result;
}

MapUpdateResult LocalMap::integrateRays(const Vec3& origin,
                                        const std::vector<Vec3>& endpoints,
                                        double stamp) {
  MapUpdateResult result =
      impl_->map->integrateRays(origin, endpoints, stamp);
  if (result.ok) {
    impl_->rebuildTerrain();
  }
  return result;
}

std::shared_ptr<const MapSnapshot> LocalMap::snapshot() const {
  return impl_->snapshot;
}

std::shared_ptr<const TerrainModel> LocalMap::terrain() const {
  return impl_->terrain;
}

TerrainResult LocalMap::evaluate(const TerrainQuery& query) const {
  if (!impl_->terrain) {
    TerrainResult result;
    result.status = TerrainStatus::kBuildFailure;
    result.detail = "no terrain cache has been built";
    return result;
  }
  return impl_->terrain->evaluate(query);
}

const LocalMapConfig& LocalMap::config() const { return impl_->config; }

}  // namespace dddmr_local_map
