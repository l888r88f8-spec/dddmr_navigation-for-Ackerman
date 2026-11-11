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

#ifndef DDDMR_GRID_GLOBAL_PLANNER__GRID_GLOBAL_PLANNER_H_
#define DDDMR_GRID_GLOBAL_PLANNER__GRID_GLOBAL_PLANNER_H_

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "dddmr_grid_global_planner/corridor_builder.h"
#include "dddmr_grid_global_planner/distance_field.h"
#include "dddmr_grid_global_planner/grid_search.h"
#include "dddmr_grid_global_planner/latest_validation.h"
#include "dddmr_grid_global_planner/map_snapshot.h"
#include "dddmr_grid_global_planner/map_update_worker.h"
#include "dddmr_grid_global_planner/metrics.h"
#include "dddmr_grid_global_planner/msg/source_observation.hpp"
#include "dddmr_grid_global_planner/multiresolution_map.h"
#include "dddmr_grid_global_planner/offline_static_map.h"
#include "dddmr_grid_global_planner/path_validator.h"
#include "dddmr_grid_global_planner/plan_commit_delivery.h"
#include "dddmr_grid_global_planner/snapshot_store.h"
#include "dddmr_grid_global_planner/source_observation.h"
#include "dddmr_grid_global_planner/terrain_grid_builder.h"
#include "dddmr_grid_global_planner/terrain_types.h"
#include "dddmr_grid_global_planner/wheel_support.h"
#include "dddmr_sys_core/action/get_plan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace perception_3d
{
class PerceptionSnapshot;
}  // namespace perception_3d

namespace dddmr_grid_global_planner
{

// ROS 2 action node for the immutable-snapshot Grid/Hybrid A* planner.
//
// The node intentionally keeps all planning state private to one accepted goal:
// a request pins one MapSnapshot, runs the search, then validates the complete
// trajectory against the latest snapshot under a bounded retry/deadline loop.
// Map updates never cancel a search; only an explicit action cancel, the legacy
// activate_threading=false reserve-bypass command, or shutdown sets the job
// cancellation flag.
//
// The action contract is frozen to dddmr_sys_core/action/GetPlan on /get_plan so
// p2p_move_base can switch between the legacy and grid planners without changing
// route_action_name.
class GridGlobalPlanner : public rclcpp::Node
{
public:
  // Provider for the immutable latest perception export. The provider must
  // return a frozen object that remains valid without the perception lock. The
  // node polls it on its own timer so the perception update loop never blocks on
  // planner ingestion. A weak capture in the provider is expected so shutdown
  // can detach the callback before the perception instance is destroyed.
  using PerceptionSnapshotProvider =
    std::function<std::shared_ptr<const perception_3d::PerceptionSnapshot>()>;

  // Constructs the planner without an in-process perception provider.
  explicit GridGlobalPlanner(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Constructs the planner and consumes the supplied immutable perception
  // export. The provider is invoked from a planner-owned timer.
  GridGlobalPlanner(
    const rclcpp::NodeOptions & options,
    PerceptionSnapshotProvider perception_provider);

  ~GridGlobalPlanner() override;

  GridGlobalPlanner(const GridGlobalPlanner &) = delete;
  GridGlobalPlanner & operator=(const GridGlobalPlanner &) = delete;

private:
  using GetPlan = dddmr_sys_core::action::GetPlan;
  using GoalHandle = rclcpp_action::ServerGoalHandle<GetPlan>;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using SourceObservationMsg =
    dddmr_grid_global_planner::msg::SourceObservation;

  // Complete node/configuration mapping. The core option types are kept as-is
  // so search, validation, and the worker consume exactly the same values.
  struct GridPlannerConfig
  {
    TerrainGridBuilderOptions terrain;
    CoarseTerrainMapOptions coarse;
    ObstacleDistanceOptions obstacles;
    MapUpdateWorkerOptions worker;
    WheelSupportOptions wheel;
    HybridSearchOptions search;
    // "hybrid" keeps the Ackermann Hybrid-A* pipeline; "fast_graph" searches
    // the existing terrain graph directly (ground nodes only, no kinematics).
    std::string search_backend{"hybrid"};
    PathValidationOptions validation;
    VehicleCapabilityConfig vehicle;
    bool vehicle_capability_declared_configured{false};

    // Explicit offline static-PCD input. When enabled the node loads the PCDs
    // once, publishes one immutable offline snapshot, and never waits for a
    // live sensor, perception export, localization or TF-provided pose. The
    // live pipeline is untouched because this defaults to OFF.
    struct OfflineConfig
    {
      bool enabled{false};
      // Default true: the complete global cloud is the authoritative surface
      // source. GroundMap is an optional hint and may be empty. Setting this
      // false keeps the legacy ground-only input contract for old profiles.
      bool global_cloud_mode{false};
      std::string global_pcd;
      std::string ground_pcd;
      std::string obstacle_pcd;
      OfflineStaticMapOptions map;
    } offline;

    std::size_t max_expansions{200'000};
    std::size_t max_samples{2'000'000};

    std::string global_frame{"map"};
    std::string robot_base_frame{"base_link"};

    bool enable_ground_cloud{true};
    bool enable_static_obstacle_cloud{true};
    bool enable_dynamic_obstacle_cloud{true};
    // Raw PointCloud2 source contributions are an explicit opt-in fallback.
    // They are OFF by default so a shared dynamic cloud is never duplicated
    // alongside the explicit SourceObservation evidence.
    bool enable_raw_cloud_fallback{false};
    // Explicit frozen ROS observation input (msg/SourceObservation).
    bool enable_source_observation{true};
    // Poll the in-process perception provider's immutable export.
    bool enable_perception_export{true};
    std::string ground_cloud_topic{"mapground"};
    std::string static_obstacle_cloud_topic{"mapcloud"};
    std::string dynamic_obstacle_cloud_topic{
      "navigation/preprocessed_cloud"};
    std::string source_observation_topic{"~/source_observation"};
    double perception_publish_rate_hz{10.0};
    std::size_t max_source_queue{4};
    std::size_t max_sources{64};
    std::size_t max_source_records{200'000};
    // Bounded live retired-snapshot references. A small production bound keeps
    // publication backpressure observable instead of retaining a deep history.
    std::size_t max_retired_snapshots{64};
    // Per-layer TTLs used by the node's managed expiry. Static latched map
    // contributions are immortal by default; a positive value opts into a
    // finite static TTL. Dynamic sources expire to unknown.
    double static_source_ttl_sec{-1.0};
    double dynamic_source_ttl_sec{2.0};

    double tf_timeout_sec{0.20};
    double tf_future_tolerance_sec{0.50};
    double max_sensor_age_sec{2.0};
    bool allow_latest_tf_fallback{true};
    double source_expiry_period_sec{1.0};

    double max_planning_time_sec{10.0};
    double max_latest_validation_time_sec{2.0};
    int max_latest_validation_retries{4};
    double corridor_tile_size_m{3.2};
    std::size_t corridor_max_samples{50'000};
    double body_sweep_spacing_m{0.10};

    bool publish_stage_debug{true};
    bool publish_wheel_debug{false};
    bool publish_terrain_debug{true};
    bool publish_coarse_debug{true};
    bool publish_corridor_debug{true};
    bool publish_cost_debug{true};
    std::size_t max_visualization_points{200'000};
    std::size_t max_measurement_visualization_patches{20'000};
    std::size_t max_corridor_visualization_cells{20'000};

    std::string path_topic{"~/path"};
    std::string terrain_topic{"~/terrain"};
    std::string coarse_topic{"~/coarse"};
    std::string corridor_topic{"~/corridor"};
    std::string cost_topic{"~/cost"};
    std::string diagnostics_topic{"~/diagnostics"};
    std::string version_topic{"~/version"};
    std::string fallback_topic{"~/fallback"};
    std::string wheel_debug_topic{"~/wheel_support"};
    std::string metrics_topic{"~/metrics"};
    std::string planner_stage_topic{
      "/debug/planner_route_request_stage"};
    // Per-query latency histograms are OFF by default. Counts and phase totals
    // remain available; enabling this only adds detailed timing fields.
    bool detailed_query_timing{false};
  };

  // One immutable cloud pointer plus the monotonic receipt time. The message is
  // const and owned by the shared pointer, so the callback never hands the
  // conversion thread a mutable PCL/point buffer.
  struct FrozenCloud
  {
    PointCloud2::ConstSharedPtr message;
    std::int64_t received_steady_ns{0};
    std::uint64_t sequence{0};
  };

  // Latest-wins slot for one complete source. Ground, static, and dynamic
  // snapshots are complete replacements, so keeping the newest frozen message
  // preserves the final semantics without blocking a ROS callback.
  struct IngestSlot
  {
    FrozenCloud cloud;
    bool has_value{false};
  };

  // One frozen source-observation message plus its monotonic receipt time. The
  // message is const and shared, so the bounded ingestion queue never aliases a
  // mutable ROS buffer.
  struct FrozenSourceObservation
  {
    SourceObservationMsg::ConstSharedPtr message;
    std::int64_t received_steady_ns{0};
    std::uint64_t queue_sequence{0};
  };

  // Per-source ordering/TTL bookkeeping for accepted evidence. isCurrent() is
  // not used as a substitute for this ordering.
  struct SourceRuntimeState
  {
    std::uint64_t last_sequence{0};
    bool have_sequence{false};
    std::int64_t last_received_steady_ns{0};
    SourceLayer layer{SourceLayer::kDynamic};
  };

  enum class CloudKind : std::uint8_t
  {
    kGround = 0,
    kStatic = 1,
    kDynamic = 2,
  };

  // One accepted normal plan. The cancellation token is shared with the search
  // budget and the latest-map validation loop.
  struct PlanningJob
  {
    std::shared_ptr<GoalHandle> goal_handle;
    std::shared_ptr<std::atomic<bool>> cancel;
    std::chrono::steady_clock::time_point received;
    std::uint64_t request_id{0};
    std::uint64_t goal_seq{0};
    std::uint64_t lifecycle_epoch{0};
    // Exact action identity used by the commit/delivery protocol. A stale goal
    // from a previous deactivation generation can never cancel, commit, or
    // publish for the current job.
    PlanJobIdentity identity;
  };

  // Logical corridor tile keys and immutable commit tokens are defined by the
  // ROS-independent latest_validation helper so the exact production decision
  // logic can be unit-tested without constructing a ROS node.
  // ---- Startup and configuration -----------------------------------------
  void LoadConfiguration();
  void LoadTerrainConfiguration();
  void LoadVehicleConfiguration();
  void LoadObstacleAndWorkerConfiguration();
  void LoadWheelAndSearchConfiguration();
  void LoadValidationConfiguration();
  void LoadOfflineConfiguration();
  void LoadRosAndDiagnosticConfiguration();
  void ValidateConfiguration();
  void CreateRosInterfaces();
  void StartMapWorker();
  // Loads the configured PCDs once, builds the offline static MapSnapshot and
  // publishes it into the normal snapshot store. Throws on any failure; it never
  // leaves a partial map or fabricates a surface.
  void LoadOfflineSnapshot();
  void StartThreads();
  void StopThreads();

  // Thin wrappers keep every parameter declaration/get in one place and let the
  // YAML mapping test enumerate the keys.
  double GetDoubleParameter(const std::string & name, double default_value);
  std::int64_t GetIntParameter(
    const std::string & name,
    std::int64_t default_value);
  bool GetBoolParameter(const std::string & name, bool default_value);
  std::string GetStringParameter(
    const std::string & name,
    std::string default_value);
  std::vector<double> GetDoubleArrayParameter(
    const std::string & name, std::vector<double> default_value);

  // ---- ROS action ---------------------------------------------------------
  rclcpp_action::GoalResponse HandleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GetPlan::Goal> goal);
  rclcpp_action::CancelResponse HandleCancel(
    const std::shared_ptr<GoalHandle> goal_handle);
  void HandleAccepted(const std::shared_ptr<GoalHandle> goal_handle);

  // ---- Map input ----------------------------------------------------------
  void OnGroundCloud(const PointCloud2::ConstSharedPtr message);
  void OnStaticObstacleCloud(const PointCloud2::ConstSharedPtr message);
  void OnDynamicObstacleCloud(const PointCloud2::ConstSharedPtr message);
  void OnSourceObservation(const SourceObservationMsg::ConstSharedPtr message);
  void QueueFrozenCloud(CloudKind kind, const PointCloud2::ConstSharedPtr & message);
  void IngestLoop();
  void ProcessGroundCloud(const FrozenCloud & cloud);
  void ProcessSourceCloud(CloudKind kind, const FrozenCloud & cloud);
  void ProcessFrozenSourceObservation(
    const FrozenSourceObservation & observation);
  void ProcessPerceptionSnapshot(
    const perception_3d::PerceptionSnapshot & snapshot);
  void OnPerceptionTimer();
  void SubmitSourceObservation(
    SourceObservationSnapshot observation,
    const std::string & reason);
  void MarkSourceBackpressure(
    const std::string & source_id,
    const std::string & detail);
  void ClearSourceBackpressure(const std::string & source_id);
  void PurgeExpiredSources(std::int64_t now_steady_ns);
  bool TransformCloudToMap(
    const PointCloud2 & cloud, int64_t stamp_ns,
    std::vector<Point3> * points,
    std::string * detail) const;
  void OnExpireTimer();
  int64_t ResolveSensorStampNs(
    const builtin_interfaces::msg::Time & stamp,
    bool allow_zero,
    std::string * detail) const;
  bool ValidateLatestTransformAge(
    const geometry_msgs::msg::TransformStamped & transform,
    std::string * detail) const;
  uint64_t LatestTerrainRevision() const;

  // ---- TF helpers ---------------------------------------------------------
  bool LookupTransform(
    const std::string & target_frame,
    const std::string & source_frame, int64_t stamp_ns,
    double timeout_sec,
    geometry_msgs::msg::TransformStamped * transform,
    std::string * detail) const;
  bool ResolveRequestPose(
    const geometry_msgs::msg::PoseStamped & request,
    bool empty_is_base_link, bool require_non_empty,
    Point3 * position, double * heading,
    bool * has_heading, std::string * detail) const;
  static bool QuaternionIsZero(const geometry_msgs::msg::Quaternion & q);
  static double YawFromQuaternion(const geometry_msgs::msg::Quaternion & q);
  static bool PosePositionIsFinite(const geometry_msgs::msg::Pose & pose);

  // ---- Planning job -------------------------------------------------------
  void PlanningLoop();
  void RunPlanningJob(const PlanningJob & job);
  void CancelActivePlan(const std::string & reason);
  LatestValidationResult ValidateLatest(
    const PlanningJob & job,
    const std::shared_ptr<const MapSnapshot> & searched_snapshot,
    const std::vector<PathPose> & searched_path,
    std::chrono::steady_clock::time_point overall_deadline);
  // Copies the node's search/validation/vehicle configuration into the
  // ROS-independent helper contract. It is a value copy, so the helper can
  // never observe later parameter mutation.
  LatestValidationConfig MakeLatestValidationConfig() const;

  // ---- Outputs and diagnostics -------------------------------------------
  void PublishStage(
    std::uint64_t request_id, std::uint64_t goal_seq,
    const std::string & stage, double elapsed_sec,
    std::size_t expansions,
    const std::string & result_class);
  void PublishDiagnostics(
    std::uint64_t request_id, std::uint64_t goal_seq,
    const std::string & stage,
    const std::string & result_class,
    const std::string & detail, double elapsed_sec,
    const MapSnapshot * snapshot);
  void PublishPlanOutputs(
    const MapSnapshot & snapshot,
    const SearchResult & search,
    const LatestValidationResult & validation,
    std::uint64_t request_id,
    std::uint64_t goal_seq);
  // Expensive debug/visualization outputs for one validated plan. These run
  // before the final commit guard; only the prepared path publication and the
  // terminal action transition run while the guards are held.
  void PublishPlanDebugOutputs(
    const MapSnapshot & snapshot,
    const SearchResult & search,
    const LatestValidationResult & validation,
    std::uint64_t request_id,
    std::uint64_t goal_seq);
  // Final acceptance serialized with map publication. The commit proof is built
  // outside all locks from `validation.snapshot`: relevant corridor tiles when
  // coverage completed, otherwise exact whole-snapshot object identity from the
  // bounded full validation. PlanCommitDelivery::CommitAndDeliver() runs the
  // cheap proof predicate and the short in-memory commit handoff under the store
  // handoff mutex (store -> job -> delivery state), then releases every lock
  // before the action SDK succeed()/abort() and private path DDS publication.
  // The commit acceptance hook marks the committed phase under the same job
  // guard that schedules cancellation, so there is no check-to-mark cancel gap.
  // A map change after the atomic commit may be visible during delivery; the
  // plan is valid at the commit linearization point, not at client receipt.
  CommitDeliveryResult CommitValidatedPlan(
    const PlanningJob & job, const LatestValidationResult & validation,
    const CommitDeliveryHooks & hooks);
  void PublishPath(const std::vector<PathPose> & path);
  void PublishPreparedPath(const nav_msgs::msg::Path & path);
  // Publishes the map-ready latched terrain/coarse/version evidence. Each output
  // is isolated so a visualization failure cannot fail startup or motion.
  void PublishInitialSnapshotVisualization(const MapSnapshot & snapshot);
  void PublishTerrain(const MapSnapshot & snapshot);
  void PublishCoarse(const MapSnapshot & snapshot);
  void PublishCorridor(
    const MapSnapshot & snapshot,
    const std::vector<PathPose> & path);
  void PublishCost(const std::vector<PathPose> & path);
  void PublishVersion(
    const MapSnapshot & snapshot,
    const std::string & source);
  void PublishFallback(
    const SearchResult & search,
    const LatestValidationResult & validation);
  void PublishWheelDebug(
    const MapSnapshot & snapshot,
    const std::vector<PathPose> & path);
  // Publishes one already-serialized telemetry record on ~/metrics. It is
  // called outside every job/commit/store guard and never from the search hot
  // loop.
  void PublishMetrics(const std::string & json);
  // Optional worker observer: serializes one per-event/per-publication record.
  void PublishUpdateMetrics(const MapUpdateMetrics & metrics);
  // Publishes a terminal request record for paths that never enter the planning
  // job (busy rejection, deactivation, lost reservation). It records the actual
  // outcome instead of dropping the request from telemetry.
  void PublishTerminalMetrics(
    std::uint64_t request_id,
    std::uint64_t goal_seq,
    const std::string & result_class,
    const std::string & detail);
  void PublishEmptyPath();
  void OnMapPublished(std::shared_ptr<const MapSnapshot> snapshot);

  // ---- Members ------------------------------------------------------------
  GridPlannerConfig config_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<SnapshotStore> snapshot_store_;
  std::unique_ptr<MapUpdateWorker> map_worker_;

  rclcpp_action::Server<GetPlan>::SharedPtr action_server_;
  rclcpp::Subscription<PointCloud2>::SharedPtr ground_subscription_;
  rclcpp::Subscription<PointCloud2>::SharedPtr static_subscription_;
  rclcpp::Subscription<PointCloud2>::SharedPtr dynamic_subscription_;
  rclcpp::Subscription<SourceObservationMsg>::SharedPtr
    source_observation_subscription_;
  rclcpp::TimerBase::SharedPtr expiry_timer_;
  rclcpp::TimerBase::SharedPtr perception_timer_;
  PerceptionSnapshotProvider perception_provider_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr terrain_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    coarse_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    corridor_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    cost_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr version_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fallback_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    wheel_debug_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stage_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metrics_publisher_;

  std::thread planning_thread_;
  std::thread ingest_thread_;

  mutable std::mutex job_mutex_;
  std::condition_variable job_condition_;
  std::optional<PlanningJob> pending_job_;
  bool plan_active_{false};
  bool plan_reserved_{false};
  bool plan_reserved_epoch_valid_{false};
  std::uint64_t plan_reserved_epoch_{0};
  std::uint64_t lifecycle_epoch_{0};
  std::shared_ptr<std::atomic<bool>> cancel_current_;
  // Exact identity of the admitted scheduling slot. The planning thread clears
  // this slot only on an identity match, so a late old job can never clobber a
  // newer pending job after a deactivation.
  std::optional<PlanJobIdentity> active_job_identity_;
  // A cancel request can arrive between HandleGoal reservation and
  // HandleAccepted admission. Binding it to the reserved UUID prevents an old
  // goal from canceling a different/newer job.
  std::string reserved_goal_uuid_;
  bool reserved_cancel_requested_{false};
  // Phase/identity/owner bookkeeping and path publication serialization for the
  // commit -> delivering -> terminal protocol. The helper retains the real goal
  // handle as an opaque keepalive while terminality is unproven, so the Humble
  // handle cannot be destroyed (and implicitly auto-cancel) after PlanningLoop
  // drops its local PlanningJob. Never held across store/job locks.
  PlanCommitDelivery delivery_;

  mutable std::mutex ingest_mutex_;
  std::condition_variable ingest_condition_;
  std::array<IngestSlot, 3> ingest_slots_{};
  std::deque<FrozenSourceObservation> source_observation_queue_;

  mutable std::mutex source_state_mutex_;
  std::map<std::string, SourceRuntimeState> source_states_;
  std::map<std::string, std::string> source_backpressure_;

  std::atomic<bool> shutting_down_{false};
  std::atomic<std::uint64_t> request_counter_{0};
  std::atomic<std::uint64_t> goal_counter_{0};
  std::atomic<std::uint64_t> source_sequence_{0};
  std::atomic<std::uint64_t> source_observation_sequence_{0};
  std::atomic<std::uint64_t> last_perception_sequence_{0};
  std::atomic<std::uint64_t> published_obstacle_version_{0};
};

}  // namespace dddmr_grid_global_planner

#endif  // DDDMR_GRID_GLOBAL_PLANNER__GRID_GLOBAL_PLANNER_H_
