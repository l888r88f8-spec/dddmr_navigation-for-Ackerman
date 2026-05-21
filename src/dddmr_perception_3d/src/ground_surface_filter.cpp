#include <perception_3d/ground_surface_filter.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
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
  double anchor_x = 0.0;
  double anchor_y = 0.0;
  double anchor_z = 0.0;
  double anchor_intensity = 0.0;
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

struct GroundLayerCandidate {
  GroundCellKey cell;
  pcl::PointXYZI point;
};

struct CandidateGraph {
  std::vector<GroundLayerCandidate> candidates;
  std::unordered_map<GroundCellKey, std::vector<std::size_t>, GroundCellKeyHash> indices_by_cell;
};

class DisjointSet {
public:
  explicit DisjointSet(std::size_t size)
  : parent_(size), rank_(size, 0)
  {
    std::iota(parent_.begin(), parent_.end(), 0);
  }

  std::size_t Find(std::size_t value)
  {
    if(parent_[value] != value){
      parent_[value] = Find(parent_[value]);
    }
    return parent_[value];
  }

  void Union(std::size_t lhs, std::size_t rhs)
  {
    std::size_t lhs_root = Find(lhs);
    std::size_t rhs_root = Find(rhs);
    if(lhs_root == rhs_root){
      return;
    }
    if(rank_[lhs_root] < rank_[rhs_root]){
      std::swap(lhs_root, rhs_root);
    }
    parent_[rhs_root] = lhs_root;
    if(rank_[lhs_root] == rank_[rhs_root]){
      rank_[lhs_root]++;
    }
  }

private:
  std::vector<std::size_t> parent_;
  std::vector<int> rank_;
};

double MeanZ(const GroundLayerAccumulator& layer)
{
  return layer.sum_z / static_cast<double>(layer.count);
}

pcl::PointXYZI MakePoint(
  const GroundLayerAccumulator& layer,
  double max_xy_shift)
{
  pcl::PointXYZI point;
  const double count = static_cast<double>(layer.count);
  point.x = static_cast<float>(layer.sum_x / count);
  point.y = static_cast<float>(layer.sum_y / count);
  point.z = static_cast<float>(layer.sum_z / count);
  point.intensity = static_cast<float>(layer.sum_intensity / count);

  if(max_xy_shift > 0.0 && std::isfinite(max_xy_shift)){
    const double dx = static_cast<double>(point.x) - layer.anchor_x;
    const double dy = static_cast<double>(point.y) - layer.anchor_y;
    const double xy_shift = std::hypot(dx, dy);
    if(xy_shift > max_xy_shift && xy_shift > 1e-6){
      const double ratio = max_xy_shift / xy_shift;
      point.x = static_cast<float>(layer.anchor_x + dx * ratio);
      point.y = static_cast<float>(layer.anchor_y + dy * ratio);
    }
  }

  return point;
}

bool IsPolicy(const std::string& policy, const char* expected)
{
  return policy == expected;
}

void AddCandidateForPolicy(
  const GroundCellKey& cell,
  const std::vector<GroundLayerAccumulator>& layers,
  const GroundSurfaceFilterConfig& config,
  std::vector<GroundLayerCandidate>* candidates)
{
  if(layers.empty()){
    return;
  }

  std::size_t best_index = 0;
  for(std::size_t i = 1; i < layers.size(); i++){
    if(IsPolicy(config.single_surface_select_policy, "highest")){
      if(MeanZ(layers[i]) > MeanZ(layers[best_index])){
        best_index = i;
      }
    }
    else{
      if(MeanZ(layers[i]) < MeanZ(layers[best_index])){
        best_index = i;
      }
    }
  }

  candidates->push_back({
    cell,
    MakePoint(layers[best_index], config.single_surface_max_xy_shift)});
}

bool CanConnectAsSlope(
  const pcl::PointXYZI& lhs,
  const pcl::PointXYZI& rhs,
  const GroundSurfaceFilterConfig& config)
{
  const double dx = static_cast<double>(lhs.x) - static_cast<double>(rhs.x);
  const double dy = static_cast<double>(lhs.y) - static_cast<double>(rhs.y);
  const double xy_distance = std::hypot(dx, dy);
  if(xy_distance <= 1e-6){
    return false;
  }

  constexpr double kPi = 3.14159265358979323846;
  const double max_slope_rad = std::max(0.0, config.single_surface_max_slope_deg) * kPi / 180.0;
  const double allowed_z_delta =
    std::tan(max_slope_rad) * xy_distance + std::max(0.0, config.single_surface_z_margin);
  const double z_delta =
    std::fabs(static_cast<double>(lhs.z) - static_cast<double>(rhs.z));
  return z_delta <= allowed_z_delta;
}

CandidateGraph BuildCandidateGraph(
  const std::unordered_map<GroundCellKey, std::vector<GroundLayerAccumulator>, GroundCellKeyHash>& cells,
  const GroundSurfaceFilterConfig& config)
{
  CandidateGraph graph;
  graph.candidates.reserve(cells.size());

  for(const auto& cell_entry : cells){
    for(const auto& layer : cell_entry.second){
      if(layer.count == 0){
        continue;
      }
      const std::size_t candidate_index = graph.candidates.size();
      graph.candidates.push_back({
        cell_entry.first,
        MakePoint(layer, config.single_surface_max_xy_shift)});
      graph.indices_by_cell[cell_entry.first].push_back(candidate_index);
    }
  }

  return graph;
}

std::size_t SelectLargestComponentRoot(
  DisjointSet* components,
  const std::vector<GroundLayerCandidate>& candidates)
{
  struct ComponentStats {
    std::size_t count = 0;
    double sum_z = 0.0;
  };
  std::unordered_map<std::size_t, ComponentStats> stats_by_root;
  for(std::size_t i = 0; i < candidates.size(); i++){
    auto& stats = stats_by_root[components->Find(i)];
    stats.count++;
    stats.sum_z += candidates[i].point.z;
  }

  std::size_t selected_root = components->Find(0);
  for(const auto& stats_entry : stats_by_root){
    const auto selected_it = stats_by_root.find(selected_root);
    const auto& candidate_stats = stats_entry.second;
    const auto& selected_stats = selected_it->second;
    const double candidate_mean_z = candidate_stats.sum_z / static_cast<double>(candidate_stats.count);
    const double selected_mean_z = selected_stats.sum_z / static_cast<double>(selected_stats.count);
    if(candidate_stats.count > selected_stats.count ||
      (candidate_stats.count == selected_stats.count && candidate_mean_z < selected_mean_z))
    {
      selected_root = stats_entry.first;
    }
  }

  return selected_root;
}

void AddLowestCandidateForCell(
  const GroundCellKey& cell,
  const std::vector<std::size_t>& indices,
  const std::vector<GroundLayerCandidate>& candidates,
  std::unordered_map<GroundCellKey, GroundLayerCandidate, GroundCellKeyHash>* selected_by_cell)
{
  if(indices.empty()){
    return;
  }
  std::size_t best_index = indices.front();
  for(const auto candidate_index : indices){
    if(candidates[candidate_index].point.z < candidates[best_index].point.z){
      best_index = candidate_index;
    }
  }
  (*selected_by_cell)[cell] = candidates[best_index];
}

std::vector<GroundLayerCandidate> SelectLargestConnectedSurface(
  const std::unordered_map<GroundCellKey, std::vector<GroundLayerAccumulator>, GroundCellKeyHash>& cells,
  const GroundSurfaceFilterConfig& config)
{
  const CandidateGraph graph = BuildCandidateGraph(cells, config);
  const auto& candidates = graph.candidates;

  if(candidates.empty()){
    return {};
  }

  DisjointSet components(candidates.size());
  for(const auto& cell_entry : graph.indices_by_cell){
    const auto& cell = cell_entry.first;
    const auto& indices = cell_entry.second;

    for(int dx = -1; dx <= 1; dx++){
      for(int dy = -1; dy <= 1; dy++){
        if(dx == 0 && dy == 0){
          continue;
        }
        const GroundCellKey neighbor_cell{cell.first + dx, cell.second + dy};
        if(neighbor_cell < cell){
          continue;
        }
        const auto neighbor_it = graph.indices_by_cell.find(neighbor_cell);
        if(neighbor_it == graph.indices_by_cell.end()){
          continue;
        }
        for(const auto lhs_index : indices){
          for(const auto rhs_index : neighbor_it->second){
            if(CanConnectAsSlope(
                 candidates[lhs_index].point,
                 candidates[rhs_index].point,
                 config))
            {
              components.Union(lhs_index, rhs_index);
            }
          }
        }
      }
    }
  }

  const std::size_t selected_root = SelectLargestComponentRoot(&components, candidates);

  std::unordered_map<GroundCellKey, GroundLayerCandidate, GroundCellKeyHash> selected_by_cell;
  for(std::size_t i = 0; i < candidates.size(); i++){
    if(components.Find(i) != selected_root){
      continue;
    }
    auto existing = selected_by_cell.find(candidates[i].cell);
    if(existing == selected_by_cell.end() ||
      candidates[i].point.z < existing->second.point.z)
    {
      selected_by_cell[candidates[i].cell] = candidates[i];
    }
  }

  bool made_progress = true;
  while(made_progress){
    made_progress = false;
    for(const auto& cell_entry : graph.indices_by_cell){
      const auto& cell = cell_entry.first;
      if(selected_by_cell.find(cell) != selected_by_cell.end()){
        continue;
      }

      bool found_candidate = false;
      std::size_t best_index = cell_entry.second.front();
      double best_z_delta = std::numeric_limits<double>::max();
      for(const auto candidate_index : cell_entry.second){
        const auto& candidate = candidates[candidate_index];
        for(int dx = -1; dx <= 1; dx++){
          for(int dy = -1; dy <= 1; dy++){
            if(dx == 0 && dy == 0){
              continue;
            }
            const GroundCellKey neighbor_cell{cell.first + dx, cell.second + dy};
            const auto selected_neighbor = selected_by_cell.find(neighbor_cell);
            if(selected_neighbor == selected_by_cell.end()){
              continue;
            }
            if(!CanConnectAsSlope(candidate.point, selected_neighbor->second.point, config)){
              continue;
            }
            const double z_delta =
              std::fabs(static_cast<double>(candidate.point.z) -
                static_cast<double>(selected_neighbor->second.point.z));
            if(!found_candidate || z_delta < best_z_delta){
              best_index = candidate_index;
              best_z_delta = z_delta;
              found_candidate = true;
            }
          }
        }
      }

      if(found_candidate){
        selected_by_cell[cell] = candidates[best_index];
        made_progress = true;
      }
    }
  }

  for(const auto& cell_entry : graph.indices_by_cell){
    if(selected_by_cell.find(cell_entry.first) != selected_by_cell.end()){
      continue;
    }
    AddLowestCandidateForCell(
      cell_entry.first,
      cell_entry.second,
      candidates,
      &selected_by_cell);
  }

  std::vector<GroundLayerCandidate> selected;
  selected.reserve(selected_by_cell.size());
  for(const auto& selected_entry : selected_by_cell){
    selected.push_back(selected_entry.second);
  }
  std::sort(selected.begin(), selected.end(),
    [](const GroundLayerCandidate& lhs, const GroundLayerCandidate& rhs) {
      return lhs.cell < rhs.cell;
    });
  return selected;
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
      layer.anchor_x = point.x;
      layer.anchor_y = point.y;
      layer.anchor_z = point.z;
      layer.anchor_intensity = point.intensity;
      layer.count = 1;
      layers.push_back(layer);
    }
  }

  auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  filtered_cloud->header = input_cloud->header;

  if(config.force_single_ground_surface){
    std::vector<GroundLayerCandidate> selected_candidates;
    if(IsPolicy(config.single_surface_select_policy, "lowest") ||
      IsPolicy(config.single_surface_select_policy, "highest"))
    {
      selected_candidates.reserve(cells.size());
      for(const auto& cell_entry : cells){
        AddCandidateForPolicy(cell_entry.first, cell_entry.second, config, &selected_candidates);
      }
      std::sort(selected_candidates.begin(), selected_candidates.end(),
        [](const GroundLayerCandidate& lhs, const GroundLayerCandidate& rhs) {
          return lhs.cell < rhs.cell;
        });
    }
    else{
      selected_candidates = SelectLargestConnectedSurface(cells, config);
    }

    filtered_cloud->points.reserve(selected_candidates.size());
    for(const auto& candidate : selected_candidates){
      filtered_cloud->points.push_back(candidate.point);
    }
    return filtered_cloud;
  }

  filtered_cloud->points.reserve(input_cloud->points.size());
  for(const auto& cell_entry : cells){
    for(const auto& layer : cell_entry.second){
      if(layer.count == 0){
        continue;
      }
      filtered_cloud->points.push_back(MakePoint(layer, config.single_surface_max_xy_shift));
    }
  }

  return filtered_cloud;
}

}  // namespace perception_3d
