#ifndef LOCAL_PLANNER_MPPI_CUDA_OBSTACLES_H_
#define LOCAL_PLANNER_MPPI_CUDA_OBSTACLES_H_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "local_planner/mppi_critic_manager.h"
#include "local_planner/mppi_cuda_trajectory_batch.h"
#include "local_planner/mppi_obstacle_geometry.h"
#include "local_planner/mppi_planar_obstacles.h"

namespace local_planner
{

// Independent, POD-only configuration shared by the legacy_points and
// tilted_plane_2d obstacle rollouts. It intentionally does not reference
// MppiConfig or the critic manager so the host glue can be added without
// coupling this backend to their private state.
struct MppiCudaObstacleConfig
{
  double model_dt = 0.1;
  // Clearance is evaluated at points 0, step, 2*step, ... and always at the
  // final rollout point. Ignored when check_every_obstacle_point is true.
  std::size_t trajectory_point_step = 3;
  double inflation_radius = 1.0;
  double evaluation_distance = 6.5;
  double evaluation_step = 0.15;
  std::size_t evaluation_max_points = 64;
  bool check_every_obstacle_point = false;
};

struct MppiCudaObstaclePose
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
};

// Diagnostics for the tilted-plane (2-D grid) rollout.
//   * upload_ms: host->device copy time of the most recent upload attempt; 0
//     when that attempt was rejected or failed, -1 when uploadPlanar() has
//     never been called.
//   * rollout_ms: kernel + copy-back time of the last non-empty batch; 0
//     unless that batch completed device work (a batch that fails after
//     partial device work stays 0), -1 when no non-empty batch has run.
//     Metadata-only (empty candidate) batches leave it unchanged.
//   * grid_points/grid_cells: scene most recently uploaded, zeroed by a
//     rejected or failed upload so they can never describe a stale scene.
struct MppiCudaPlanarStats
{
  double upload_ms = -1.0;         // host->device grid copy, last attempt
  double rollout_ms = -1.0;        // kernel + copy back, last non-empty batch
  std::size_t grid_points = 0;
  std::size_t grid_cells = 0;
  // GPU event timings and host readback wall time. Zero means the stage
  // was skipped; -1 means no measurement is available.
  double pose_kernel_ms = -1.0;
  double query_kernel_ms = -1.0;
  double probe_kernel_ms = -1.0;
  double readback_ms = -1.0;
};

// GPU execution of two independent MPPI obstacle protocols:
//   * legacy_points: midpoint-integrated 3-D rollout with the exact PCL-float
//     cuboid transform and point-to-OBB distance, per-step BVH collision and
//     sampled adaptive clearance, then a terminal constant-curvature probe.
//   * tilted_plane_2d: planar rollout from the frozen plane origin using the
//     same shared 2-D rectangle geometry as the CPU query, sampled clearance
//     and the same terminal probe over the uploaded grid.
// Every trajectory point is produced on the device, and the host glue reuses
// allocations across batches. upload() and uploadPlanar() own separate device
// scenes and are queried independently: a
// successful uploadPlanar() is required before rolloutPlanarBatch(), while
// upload()/rolloutBatch() keep their legacy behavior untouched. One instance
// is not thread-safe; a single control thread must own it for the whole
// upload/rollout lifetime.
class MppiCudaObstacles
{
public:
  virtual ~MppiCudaObstacles() = default;

  // False when this build has no CUDA or no usable device. reason is always
  // populated on failure.
  virtual bool available(std::string * reason) const = 0;

  // Uploads a snapshot (finite cloud + flat BVH). Must succeed before
  // rolloutBatch(). No CPU fallback is performed on error.
  virtual bool upload(
    const MppiObstacleSnapshot & snapshot,
    std::string & error) = 0;

  // local_cuboid_vertices are the legacy robot_cuboid_vertices_ in PCL float
  // coordinates and in the legacy order. candidates[i].controls is one rollout.
  // On success output has exactly candidates.size() entries and preserves the
  // legacy point/probe diagnostics.
  virtual bool rolloutBatch(
    const MppiCudaObstaclePose & initial_pose,
    const pcl::PointCloud<pcl::PointXYZ> & local_cuboid_vertices,
    const std::vector<MppiCandidate> & candidates,
    const MppiCudaObstacleConfig & config,
    std::vector<MppiScoredTrajectory> & output,
    std::string & error) = 0;

  // Uploads the 2-D grid once per scene cycle. cell_offsets and points_xy are
  // copied verbatim; the grid must have been built by
  // buildMppiPlanarObstacleGrid.
  virtual bool uploadPlanar(
    const MppiPlanarObstacleGrid & grid, std::string & error) = 0;

  // Expands every candidate from the local zero pose (x=y=z=0, yaw=0) in the
  // frozen plane, queries the same 2-D grid geometry as the CPU path, and
  // emits one MppiScoredTrajectory per candidate with the terminal distance
  // probe. Must be preceded by a successful uploadPlanar().
  virtual bool rolloutPlanarBatch(
    const MppiPlanarBodyRect & body,
    const std::vector<MppiCandidate> & candidates,
    const MppiCudaObstacleConfig & config,
    std::vector<MppiScoredTrajectory> & output,
    std::string & error,
    MppiCudaTrajectoryBatchView * device_batch = nullptr) = 0;

  virtual MppiCudaPlanarStats lastPlanarStats() const = 0;
};

// Always returns a non-null backend. CUDA builds return the device backend;
// CPU-only builds return a fail-closed stub with the same interface.
std::unique_ptr<MppiCudaObstacles> makeMppiCudaObstacles();

}  // namespace local_planner

#endif  // LOCAL_PLANNER_MPPI_CUDA_OBSTACLES_H_
