// Copyright 2026 DDDMobileRobot
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "dddmr_grid_global_planner/continuous_ground_surface.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <utility>

#include "dddmr_grid_global_planner/offline_geometry.h"

namespace dddmr_grid_global_planner
{
namespace
{
using Key = std::pair<std::int64_t, std::int64_t>;
using Refs = std::vector<std::uint32_t>;
using Fit = offline_geometry::PlaneFit;
constexpr double kEpsilon = 1.0e-9;

Key KeyOf(const FineCellIndex & cell) {return {cell.x, cell.y};}

double FitHeight(const Fit & fit, double x, double y)
{
  return fit.centroid.z + fit.a * (x - fit.centroid.x) +
         fit.b * (y - fit.centroid.y);
}

double PlaneHeight(const PlaneEvidence & plane, double x, double y)
{
  return plane.centroid.z - (plane.normal.x * (x - plane.centroid.x) +
         plane.normal.y * (y - plane.centroid.y)) / plane.normal.z;
}

bool FitRefs(const std::vector<Point3> & points, const Refs & refs, Fit * fit)
{
  std::vector<Point3> samples;
  samples.reserve(refs.size());
  double min_x = std::numeric_limits<double>::infinity(), min_y = min_x;
  double max_x = -min_x, max_y = max_x;
  for (const auto index : refs) {
    samples.push_back(points[index]);
    min_x = std::min(min_x, points[index].x); max_x = std::max(max_x, points[index].x);
    min_y = std::min(min_y, points[index].y); max_y = std::max(max_y, points[index].y);
  }
  // Exact scan lines can acquire tiny non-zero centred covariance from mean
  // rounding. They still cannot establish a two-dimensional surface normal.
  if (max_x - min_x < 1.0e-6 || max_y - min_y < 1.0e-6) {return false;}
  return offline_geometry::FitPlane(samples, fit);
}

// Convex support is used only inside one real height layer. In particular it
// cannot connect a lower floor to an upper bridge in the same XY footprint.
bool InsideHull(const std::vector<Point3> & points, const Refs & refs, const Point3 & q)
{
  using XY = std::pair<double, double>;
  std::vector<XY> xy;
  for (const auto index : refs) {
    xy.emplace_back(points[index].x, points[index].y);
  }
  std::sort(xy.begin(), xy.end());
  xy.erase(std::unique(xy.begin(), xy.end()), xy.end());
  if (xy.size() < 3U) {return false;}
  const auto cross = [](const XY & a, const XY & b, const XY & c) {
      return (b.first - a.first) * (c.second - a.second) -
             (b.second - a.second) * (c.first - a.first);
    };
  std::vector<XY> lower, upper;
  for (const auto & p : xy) {
    while (lower.size() > 1U && cross(lower[lower.size() - 2U], lower.back(), p) <= 0.0) {
      lower.pop_back();
    }
    lower.push_back(p);
  }
  for (auto it = xy.rbegin(); it != xy.rend(); ++it) {
    while (upper.size() > 1U && cross(upper[upper.size() - 2U], upper.back(), *it) <= 0.0) {
      upper.pop_back();
    }
    upper.push_back(*it);
  }
  lower.pop_back();
  upper.pop_back();
  lower.insert(lower.end(), upper.begin(), upper.end());
  if (lower.size() < 3U) {return false;}
  const XY query{q.x, q.y};
  for (std::size_t i = 0; i < lower.size(); ++i) {
    if (cross(lower[i], lower[(i + 1U) % lower.size()], query) < -kEpsilon) {return false;}
  }
  return true;
}

// Small height gaps establish a step only when the two low-residual planes
// occupy distinct spatial sides. Interleaved noisy samples are one surface.
bool PersistentSplit(
  const std::vector<Point3> & points, const std::vector<FineCellIndex> & point_cells,
  const Refs & first, const Refs & second, const Point3 & centre,
  double gap, double step, double noise, double radius)
{
  if (gap > std::max(0.35, 2.0 * step)) {return true;}
  Fit a, b;
  if (!FitRefs(points, first, &a) || !FitRefs(points, second, &b) ||
    a.rms > noise || b.rms > noise ||
    std::abs(
      FitHeight(
        a, centre.x,
        centre.y) - FitHeight(b, centre.x, centre.y)) <= step + kEpsilon)
  {
    return false;
  }
  if (std::hypot(a.centroid.x - b.centroid.x, a.centroid.y - b.centroid.y) < 0.25 * radius) {
    return false;
  }
  std::set<Key> first_cells, second_cells;
  for (const auto index : first) {
    first_cells.insert(KeyOf(point_cells[index]));
  }
  for (const auto index : second) {
    second_cells.insert(KeyOf(point_cells[index]));
  }
  std::size_t overlap = 0;
  for (const auto & key : first_cells) {
    overlap += second_cells.count(key);
  }
  const auto minimum = std::min(first_cells.size(), second_cells.size());
  return minimum > 0U && 10U * overlap <= minimum;
}

std::vector<Refs> SeparateLayers(
  const std::vector<Point3> & points, const std::vector<FineCellIndex> & point_cells,
  Refs refs, const Point3 & centre, const ContinuousGroundSurfaceOptions & options,
  double step)
{
  std::sort(
    refs.begin(), refs.end(), [&](auto a, auto b) {
      return points[a].z != points[b].z ? points[a].z < points[b].z : a < b;
    });
  std::vector<Refs> groups{std::move(refs)};
  // Regional evidence can contain a step on one of two floors, although any
  // individual fine cell still contains at most max_layers_per_cell surfaces.
  const auto regional_limit = std::max<std::size_t>(4U, 3U * options.max_layers_per_cell);
  std::size_t group = 0;
  while (group < groups.size() && groups.size() < regional_limit) {
    const Refs source = groups[group];
    std::vector<std::pair<double, std::size_t>> gaps;
    for (std::size_t split = options.min_region_points;
      split + options.min_region_points <= source.size(); ++split)
    {
      const double gap = points[source[split]].z - points[source[split - 1U]].z;
      if (gap > step + kEpsilon) {gaps.emplace_back(gap, split);}
    }
    std::sort(gaps.rbegin(), gaps.rend());
    bool split_group = false;
    for (const auto & candidate : gaps) {
      Refs first(source.begin(), source.begin() + candidate.second);
      Refs second(source.begin() + candidate.second, source.end());
      if (!PersistentSplit(
          points, point_cells, first, second, centre,
          candidate.first, step, options.robust_noise_floor_m, options.regional_radius_m))
      {
        continue;
      }
      groups[group] = std::move(first);
      groups.push_back(std::move(second));
      split_group = true;
      break;
    }
    if (!split_group) {++group;}
  }
  return groups;
}

// Persist exactly the refs used by the fit. Own-cell observations are never
// silently lost when a dense regional neighbourhood needs downsampling.
bool SelectRefs(
  const std::vector<FineCellIndex> & point_cells, const Refs & source,
  const FineCellIndex & cell, std::size_t cap, Refs * refs)
{
  Refs remaining;
  for (const auto index : source) {
    if (KeyOf(point_cells[index]) == KeyOf(cell)) {refs->push_back(index);} else {
      remaining.push_back(index);
    }
  }
  if (refs->size() > cap) {return false;}
  const auto count = std::min(cap - refs->size(), remaining.size());
  for (std::size_t i = 0; i < count; ++i) {
    refs->push_back(remaining[i * remaining.size() / count]);
  }
  std::sort(refs->begin(), refs->end());
  return true;
}

bool RegionalFit(
  const std::vector<Point3> & points, const Refs & refs,
  const ContinuousGroundSurfaceOptions & options, Fit * fit, Refs * inliers)
{
  if (!FitRefs(points, refs, fit)) {return false;}
  *inliers = refs;
  for (int iteration = 0; iteration < 4; ++iteration) {
    Refs selected;
    for (const auto index : refs) {
      if (std::abs(points[index].z - FitHeight(*fit, points[index].x, points[index].y)) <=
        options.max_inlier_residual_m)
      {
        selected.push_back(index);
      }
    }
    if (selected.size() < options.min_region_points) {return false;}
    if (selected == *inliers) {break;}
    if (!FitRefs(points, selected, fit)) {return false;}
    *inliers = std::move(selected);
  }
  return std::isfinite(fit->rms) && fit->rms <= options.max_plane_rms_m;
}

struct DisjointSet
{
  explicit DisjointSet(std::size_t size)
  : parent(size), rank(size, 0U)
  {std::iota(parent.begin(), parent.end(), 0U);}
  std::size_t Find(std::size_t item)
  {
    while (parent[item] != item) {
      parent[item] = parent[parent[item]];
      item = parent[item];
    }
    return item;
  }
  void Join(std::size_t first, std::size_t second)
  {
    first = Find(first); second = Find(second);
    if (first == second) {return;}
    if (rank[first] < rank[second]) {std::swap(first, second);}
    parent[second] = first;
    if (rank[first] == rank[second]) {++rank[first];}
  }
  std::vector<std::size_t> parent, rank;
};

void WeldCorners(
  std::vector<ContinuousSurfaceCell> * cells, double tolerance, double normal_limit,
  ContinuousGroundSurfaceMetrics * metrics)
{
  DisjointSet vertices(cells->size() * 4U), patches(cells->size());
  std::map<Key, std::vector<std::size_t>> lookup;
  for (std::size_t i = 0; i < cells->size(); ++i) {
    lookup[KeyOf((*cells)[i].cell)].push_back(i);
  }
  for (std::size_t i = 0; i < cells->size(); ++i) {
    const auto & first = (*cells)[i];
    for (int direction = 0; direction < 2; ++direction) {
      const Key adjacent{first.cell.x + (direction == 0), first.cell.y + (direction == 1)};
      const auto found = lookup.find(adjacent);
      if (found == lookup.end()) {continue;}
      const std::array<std::size_t, 2> first_edge = direction == 0 ?
        std::array<std::size_t, 2>{1U, 2U} : std::array<std::size_t, 2>{3U, 2U};
      const std::array<std::size_t, 2> second_edge = direction == 0 ?
        std::array<std::size_t, 2>{0U, 3U} : std::array<std::size_t, 2>{0U, 1U};
      for (const auto j : found->second) {
        const auto & second = (*cells)[j];
        const double dot = first.plane.normal.x * second.plane.normal.x +
          first.plane.normal.y * second.plane.normal.y +
          first.plane.normal.z * second.plane.normal.z;
        if (std::acos(std::clamp(dot, -1.0, 1.0)) > normal_limit) {continue;}
        bool compatible = true;
        for (std::size_t end = 0; end < 2U; ++end) {
          compatible = compatible && std::abs(
            first.shared_corners[first_edge[end]].z -
            second.shared_corners[second_edge[end]].z) <= tolerance + kEpsilon;
        }
        if (!compatible) {continue;}
        patches.Join(i, j);
        for (std::size_t end = 0; end < 2U; ++end) {
          vertices.Join(4U * i + first_edge[end], 4U * j + second_edge[end]);
        }
      }
    }
  }
  std::vector<double> sums(cells->size() * 4U, 0.0), weights(sums.size(), 0.0);
  for (std::size_t i = 0; i < cells->size(); ++i) {
    for (std::size_t corner = 0; corner < 4U; ++corner) {
      const auto vertex = vertices.Find(4U * i + corner);
      const double weight = std::max(0.01, (*cells)[i].confidence);
      sums[vertex] += weight * (*cells)[i].shared_corners[corner].z;
      weights[vertex] += weight;
    }
  }
  std::set<std::size_t> patch_roots;
  for (std::size_t i = 0; i < cells->size(); ++i) {
    auto & cell = (*cells)[i];
    for (std::size_t corner = 0; corner < 4U; ++corner) {
      const auto vertex = vertices.Find(4U * i + corner);
      cell.shared_corners[corner].z = sums[vertex] / weights[vertex];
    }
    cell.center.z = 0.5 * (cell.shared_corners[0].z + cell.shared_corners[2].z);
    cell.shared_corners_valid = true;
    patch_roots.insert(patches.Find(i));
  }
  metrics->welded_cells = cells->size();
  metrics->welded_patches = patch_roots.size();
}
}  // namespace

bool BuildContinuousGroundSurface(
  const std::vector<Point3> & points, const ContinuousGroundSurfaceOptions & options,
  double resolution, double origin_x, double origin_y, double max_step_height_m,
  double max_slope_rad, std::vector<ContinuousSurfaceCell> * cells,
  ContinuousGroundSurfaceMetrics * metrics)
{
  if (cells == nullptr || metrics == nullptr) {return false;}
  cells->clear(); *metrics = ContinuousGroundSurfaceMetrics{};
  if (!options.enabled) {return true;}
  std::vector<ContinuousSurfaceCell> pending;
  auto * destination = cells;
  cells = &pending;
  if (!std::isfinite(resolution) || resolution <= 0.0 ||
    !std::isfinite(origin_x) || !std::isfinite(origin_y) ||
    !std::isfinite(options.regional_radius_m) || options.regional_radius_m <= 0.0 ||
    !std::isfinite(options.max_gap_fill_m) || options.max_gap_fill_m < 0.0 ||
    !std::isfinite(options.max_plane_rms_m) || options.max_plane_rms_m <= 0.0 ||
    !std::isfinite(options.max_inlier_residual_m) || options.max_inlier_residual_m <= 0.0 ||
    !std::isfinite(options.robust_noise_floor_m) || options.robust_noise_floor_m <= 0.0 ||
    !std::isfinite(options.min_region_confidence) || options.min_region_confidence < 0.0 ||
    options.min_region_confidence > 1.0 || !std::isfinite(options.min_angular_coverage) ||
    options.min_angular_coverage < 0.0 || options.min_angular_coverage > 1.0 ||
    !std::isfinite(options.corner_association_tolerance_m) ||
    options.corner_association_tolerance_m < 0.0 ||
    !std::isfinite(options.max_association_normal_rad) ||
    options.max_association_normal_rad < 0.0 ||
    !std::isfinite(max_step_height_m) || max_step_height_m <= 0.0 ||
    !std::isfinite(max_slope_rad) || max_slope_rad <= 0.0 || max_slope_rad >= 1.57 ||
    options.min_region_points < 3U || options.max_region_points < options.min_region_points ||
    options.max_region_points > 1'000'000U || options.max_support_refs_per_cell > 1'000'000U ||
    options.max_support_refs_per_cell < options.min_region_points ||
    options.max_layers_per_cell == 0U || options.max_layers_per_cell > 8U ||
    options.max_candidate_seeds == 0U || points.size() > std::numeric_limits<std::uint32_t>::max())
  {
    return false;
  }
  const double reach = std::ceil(options.max_gap_fill_m / resolution);
  if (!std::isfinite(reach) || reach > 512.0) {return false;}
  const auto halo = static_cast<std::int64_t>(reach);
  std::map<Key, Refs> observed, bins;
  std::vector<FineCellIndex> point_cells;
  point_cells.reserve(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (!points[i].IsFinite()) {return false;}
    const auto cell = FineCellOf(points[i], origin_x, origin_y, resolution);
    const auto bin = FineCellOf(points[i], origin_x, origin_y, options.regional_radius_m);
    if (!cell || !bin) {return false;}
    point_cells.push_back(*cell);
    observed[KeyOf(*cell)].push_back(static_cast<std::uint32_t>(i));
    bins[KeyOf(*bin)].push_back(static_cast<std::uint32_t>(i));
  }
  metrics->measured_cells = observed.size();
  std::set<Key> candidates;
  for (const auto & item : observed) {
    for (std::int64_t dx = -halo; dx <= halo; ++dx) {
      for (std::int64_t dy = -halo; dy <= halo; ++dy) {
        const FineCellIndex cell{item.first.first + dx, item.first.second + dy};
        if (!IsSafeFineCell(cell)) {return false;}
        candidates.insert(KeyOf(cell));
        if (candidates.size() > options.max_inferred_cells + observed.size()) {return false;}
      }
    }
  }
  std::set<Key> inferred_locations;
  const auto cap = std::min(options.max_region_points, options.max_support_refs_per_cell);
  for (const auto & key : candidates) {
    const FineCellIndex cell{key.first, key.second};
    const Point3 centre = FineCellCenter(cell, origin_x, origin_y, resolution);
    const auto own = observed.find(key);
    const Point3 anchor = own == observed.end() ? centre : points[own->second.front()];
    const auto anchor_bin = FineCellOf(anchor, origin_x, origin_y, options.regional_radius_m);
    if (!anchor_bin) {return false;}
    Refs region;
    for (std::int64_t dx = -1; dx <= 1; ++dx) {
      for (std::int64_t dy = -1; dy <= 1; ++dy) {
        const auto found = bins.find({anchor_bin->x + dx, anchor_bin->y + dy});
        if (found == bins.end()) {continue;}
        for (const auto index : found->second) {
          if (std::hypot(points[index].x - anchor.x, points[index].y - anchor.y) <=
            options.regional_radius_m + kEpsilon)
          {
            region.push_back(index);
            if (region.size() > 16U * options.max_region_points) {return false;}
          }
        }
      }
    }
    if (region.size() < options.min_region_points) {continue;}
    std::sort(region.begin(), region.end());
    if (region.size() > options.max_region_points) {
      Refs sampled;
      if (!SelectRefs(point_cells, region, cell, options.max_region_points, &sampled)) {
        return false;
      }
      region = std::move(sampled);
    }
    const auto layers = SeparateLayers(
      points, point_cells, region, centre, options,
      max_step_height_m);
    const std::size_t first_output = cells->size();
    for (const auto & layer : layers) {
      Refs refs, inliers;
      if (!SelectRefs(point_cells, layer, cell, cap, &refs)) {return false;}
      if (refs.size() < options.min_region_points) {continue;}
      Fit fit;
      if (!RegionalFit(points, refs, options, &fit, &inliers) ||
        std::atan(std::hypot(fit.a, fit.b)) > max_slope_rad + kEpsilon)
      {
        ++metrics->rejected_confidence; continue;
      }
      std::size_t own_count = 0;
      std::set<Key> support_cells;
      double nearest = std::numeric_limits<double>::infinity();
      unsigned int sectors = 0;
      for (const auto index : refs) {
        own_count += KeyOf(point_cells[index]) == key;
      }
      for (const auto index : inliers) {
        support_cells.insert(KeyOf(point_cells[index]));
        const double dx = points[index].x - centre.x, dy = points[index].y - centre.y;
        nearest = std::min(nearest, std::hypot(dx, dy));
        const double angle = std::atan2(dy, dx) + 3.14159265358979323846;
        sectors |= 1U << std::min(
          7U,
          static_cast<unsigned int>(angle * 8.0 / 6.28318530717958647692));
      }
      std::size_t covered = 0;
      for (unsigned int bit = 0; bit < 8U; ++bit) {
        covered += (sectors >> bit) & 1U;
      }
      const double coverage = covered / 8.0;
      const double confidence = static_cast<double>(inliers.size()) / refs.size() *
        (1.0 - 0.5 * fit.rms / options.max_plane_rms_m);
      if (support_cells.size() > options.max_candidate_seeds ||
        confidence < options.min_region_confidence || coverage < options.min_angular_coverage)
      {
        ++metrics->rejected_confidence; continue;
      }
      const bool inferred = own_count == 0U;
      if (inferred && (support_cells.size() < 2U ||
        nearest > options.max_gap_fill_m + kEpsilon || !InsideHull(points, inliers, centre)))
      {
        ++metrics->rejected_exterior; continue;
      }
      ContinuousSurfaceCell output;
      output.cell = cell;
      output.center = centre;
      output.center.z = FitHeight(fit, centre.x, centre.y);
      output.plane.valid = true;
      output.plane.centroid = fit.centroid;
      output.plane.normal = fit.normal;
      output.plane.offset = fit.normal.x * fit.centroid.x +
        fit.normal.y * fit.centroid.y + fit.normal.z * fit.centroid.z;
      output.plane.rms_residual = fit.rms;
      output.plane.inlier_count = inliers.size();
      output.plane.sample_count = refs.size();
      output.point_indices = std::move(refs);
      output.inlier_indices = std::move(inliers);
      output.inferred = inferred;
      output.confidence = confidence;
      output.regional_support_cells = support_cells.size();
      output.own_cell_point_count = own_count;
      const double x0 = origin_x + cell.x * resolution, y0 = origin_y + cell.y * resolution;
      const double x1 = origin_x + (cell.x + 1) * resolution,
        y1 = origin_y + (cell.y + 1) * resolution;
      output.shared_corners =
      {Point3{x0, y0, 0}, Point3{x1, y0, 0}, Point3{x1, y1, 0}, Point3{x0, y1, 0}};
      for (auto & corner : output.shared_corners) {
        corner.z = PlaneHeight(output.plane, corner.x, corner.y);
      }
      cells->push_back(std::move(output));
      if (inferred) {
        inferred_locations.insert(key); ++metrics->inferred_surfaces;
      } else {
        ++metrics->observed_surfaces; ++metrics->seed_layers;
      }
    }
    if (cells->size() - first_output > options.max_layers_per_cell) {cells->clear(); return false;}
    if (inferred_locations.size() > options.max_inferred_cells) {cells->clear(); return false;}
  }
  metrics->inferred_cells = inferred_locations.size();
  std::sort(
    cells->begin(), cells->end(), [](const auto & first, const auto & second) {
      if (KeyOf(first.cell) != KeyOf(second.cell)) {return KeyOf(first.cell) < KeyOf(second.cell);}
      return first.center.z < second.center.z;
    });
  WeldCorners(
    cells, std::min(max_step_height_m, options.corner_association_tolerance_m),
    options.max_association_normal_rad, metrics);
  destination->swap(pending);
  return true;
}

bool TerrainSurfaceCorners(const TerrainNode & node, std::array<Point3, 4> * corners) noexcept
{
  if (corners == nullptr || !node.surface_reconstruction.shared_corners_valid) {return false;}
  const auto & stored = node.surface_reconstruction.shared_corners;
  for (const auto & point : stored) {
    if (!point.IsFinite()) {
      return false;
    }
  }
  if (stored[1].x <= stored[0].x || stored[3].y <= stored[0].y ||
    stored[0].x != stored[3].x || stored[1].x != stored[2].x ||
    stored[0].y != stored[1].y || stored[2].y != stored[3].y)
  {
    return false;
  }
  *corners = stored;
  return true;
}

double TerrainSurfaceHeight(const TerrainNode & node, double x, double y) noexcept
{
  std::array<Point3, 4> corners;
  if (TerrainSurfaceCorners(node, &corners)) {
    const double u = (x - corners[0].x) / (corners[1].x - corners[0].x);
    const double v = (y - corners[0].y) / (corners[3].y - corners[0].y);
    // Same diagonal/triangles as RViz: (0,1,2), (0,2,3). Along a shared
    // edge both cells therefore evaluate the identical linear height.
    return v <= u ? corners[0].z + u * (corners[1].z - corners[0].z) +
           v * (corners[2].z - corners[1].z) :
           corners[0].z + u * (corners[2].z - corners[3].z) +
           v * (corners[3].z - corners[0].z);
  }
  if (node.plane.valid && std::abs(node.plane.normal.z) > 1.0e-12) {
    const double z = PlaneHeight(node.plane, x, y);
    if (std::isfinite(z)) {return z;}
  }
  return node.representative.z;
}
}  // namespace dddmr_grid_global_planner
