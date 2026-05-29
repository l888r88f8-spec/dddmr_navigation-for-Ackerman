#include <perception_3d/ground_surface_filter.h>

#include <cmath>
#include <unordered_map>
#include <vector>

namespace perception_3d
{
namespace {

struct GroundLayerAccumulator {
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_z = 0.0;
  double sum_intensity = 0.0;
  std::size_t count = 0;
};

using GroundCellKey = std::pair<int, int>;

struct GroundCellKeyHash {
  std::size_t operator()(const GroundCellKey& key) const
  {
    const std::size_t h1 = std::hash<int>{}(key.first);
    const std::size_t h2 = std::hash<int>{}(key.second);
    return h1 ^ (h2 << 1);
  }
};

double MeanZ(const GroundLayerAccumulator& layer)
{
  return layer.sum_z / static_cast<double>(layer.count);
}

pcl::PointXYZI MakePoint(const GroundLayerAccumulator& layer)
{
  pcl::PointXYZI point;
  const double count = static_cast<double>(layer.count);
  point.x = static_cast<float>(layer.sum_x / count);
  point.y = static_cast<float>(layer.sum_y / count);
  point.z = static_cast<float>(layer.sum_z / count);
  point.intensity = static_cast<float>(layer.sum_intensity / count);
  return point;
}

}  // namespace

pcl::PointCloud<pcl::PointXYZI>::Ptr FilterGroundSurface(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
  const GroundSurfaceFilterConfig& config)
{
  if(!config.merge_ground_layers || !input_cloud || input_cloud->points.empty() ||
    config.merge_ground_xy_resolution <= 0.0 || config.merge_ground_z_tolerance <= 0.0)
  {
    return input_cloud;
  }

  std::unordered_map<GroundCellKey, std::vector<GroundLayerAccumulator>, GroundCellKeyHash> cells;
  cells.reserve(input_cloud->points.size());

  for(const auto& point : input_cloud->points){
    const int cell_x = static_cast<int>(std::floor(point.x / config.merge_ground_xy_resolution));
    const int cell_y = static_cast<int>(std::floor(point.y / config.merge_ground_xy_resolution));
    auto& layers = cells[GroundCellKey(cell_x, cell_y)];

    bool merged = false;
    for(auto& layer : layers){
      if(std::fabs(MeanZ(layer) - point.z) <= config.merge_ground_z_tolerance){
        layer.sum_x += point.x;
        layer.sum_y += point.y;
        layer.sum_z += point.z;
        layer.sum_intensity += point.intensity;
        layer.count++;
        merged = true;
        break;
      }
    }

    if(!merged){
      GroundLayerAccumulator layer;
      layer.sum_x = point.x;
      layer.sum_y = point.y;
      layer.sum_z = point.z;
      layer.sum_intensity = point.intensity;
      layer.count = 1;
      layers.push_back(layer);
    }
  }

  auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  filtered_cloud->header = input_cloud->header;

  filtered_cloud->points.reserve(input_cloud->points.size());
  for(const auto& cell_entry : cells){
    for(const auto& layer : cell_entry.second){
      if(layer.count == 0){
        continue;
      }
      filtered_cloud->points.push_back(MakePoint(layer));
    }
  }

  return filtered_cloud;
}

}  // namespace perception_3d
