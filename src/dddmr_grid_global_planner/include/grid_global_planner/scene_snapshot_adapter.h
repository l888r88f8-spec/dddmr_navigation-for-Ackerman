// Copyright 2026 DDDMobileRobot
//
// Grid-only adapter between the unified perception SceneSnapshot and the
// planner's frozen source observations.
//
// The adapter deliberately does not contain or compare Grid terrain revisions
// with producer source revisions. It only answers:
//   1. Does this root scene satisfy the strict live-readiness contract?
//   2. Was the producer static source identity derived from the exact raw
//      ground/map pair that Grid used to build its own terrain?
//   3. After binding, has that source identity changed?
//
// A rejected scene never yields evidence, so a missing/incomplete/stale/wrong-
// frame/source-changed scene can never be converted into clearing evidence.

#ifndef GRID_GLOBAL_PLANNER__SCENE_SNAPSHOT_ADAPTER_H_
#define GRID_GLOBAL_PLANNER__SCENE_SNAPSHOT_ADAPTER_H_

#include <cstdint>
#include <memory>
#include <string>

#include <perception_3d/scene_snapshot.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace grid_global_planner
{

// Ground-merge parameters used by the producer's static source layer. The live
// perception entry copies the producer's actual runtime parameter values here,
// so Grid can derive the same source fingerprint from its raw mapground input
// without duplicating a separate algorithm or configuration guess.
struct StaticSourceInputOptions
{
  bool merge_ground_layers{false};
  double merge_ground_xy_resolution{0.1};
  double merge_ground_z_tolerance{0.12};
};

// Computes the producer-compatible identity of one raw static map/ground pair.
// The exported producer algorithms are reused:
//   - perception_3d::FilterGroundSurface (same runtime merge parameters)
//   - perception_3d::ComputeStaticContentHash
// This runs once per accepted static event, never per lidar cycle.
bool ComputeStaticSourceInputIdentity(
  const sensor_msgs::msg::PointCloud2 & ground,
  const sensor_msgs::msg::PointCloud2 & map,
  const std::string & frame_id,
  const StaticSourceInputOptions & options,
  perception_3d::SourceMapIdentity * identity,
  std::string * detail);

// Compares the producer-visible source content with Grid's independently
// derived raw-input identity. Producer revision is intentionally ignored: Grid
// never treats a producer revision as a Grid terrain revision. Frame, count and
// order-sensitive content hash must match.
bool SameStaticInputContent(
  const perception_3d::SourceMapIdentity & expected_input,
  const perception_3d::SourceMapIdentity & producer_scene);

enum class SceneAdmissionStatus : std::uint8_t
{
  kAccepted = 0,
  kRootMissing,
  kExpectedSourceUnavailable,
  kSourceIdentityUnknown,
  kIncomplete,
  kSensorsNotCurrent,
  kWrongFrame,
  kStale,
  kEvidenceUnavailable,
  kSourceChanged,
};

const char * SceneAdmissionStatusToString(
  SceneAdmissionStatus status) noexcept;

struct SceneAdmission
{
  SceneAdmissionStatus status{SceneAdmissionStatus::kRootMissing};
  std::string detail;
  // Set only for kAccepted. A rejected scene must never be used as clear data.
  std::shared_ptr<const perception_3d::PerceptionSnapshot> evidence;
};

// Stateful strict gate. One instance belongs to the planner's perception timer
// thread; callers that expose invalid() to other threads must synchronize
// externally.
class SceneSnapshotGate
{
public:
  // Expected identity is derived once from the raw mapground/mapcloud pair
  // actually passed to Grid's terrain builder.
  void SetExpectedInputIdentity(
    const perception_3d::SourceMapIdentity & identity);

  bool has_expected_input_identity() const noexcept
  {
    return expected_input_known_;
  }

  // True after a producer source identity has matched the expected Grid input.
  bool bound() const noexcept {return state_ == State::kBound;}

  // True on a source identity change after binding. This latch is deliberately
  // not cleared automatically: a hot map switch/rebuild is out of scope.
  bool invalid() const noexcept {return state_ == State::kInvalid;}

  const perception_3d::SourceMapIdentity & bound_source_map() const noexcept
  {
    return bound_source_map_;
  }

  const std::string & invalid_detail() const noexcept
  {
    return invalid_detail_;
  }

  // Fast metadata-only readiness check for the live planning start and the
  // final commit hook. `latest` is the provider's current immutable root scene
  // (nullptr when no root exists). No hash/rebuild, no evidence ingestion and
  // no accepted-scene cursor update happens here.
  //
  // Returns true only after an initial verified binding AND while the latest
  // known root still carries exactly the bound source identity. A known
  // identity/frame change latches invalid even if that root is incomplete,
  // has stale sensors/receipt, or has no evidence. A missing root or unknown
  // identity fails closed without latching: only a real source change is a
  // permanent restart/rebuild condition.
  bool CheckBoundSource(
    const perception_3d::SceneSnapshot * latest,
    std::string * detail);

  // Strict evaluation order:
  // root -> expected identity -> invalid latch -> known source ->
  // bound-source change -> source frame -> completeness -> sensor current ->
  // receipt freshness -> first-bind pending mismatch -> accept.
  //
  // A source mismatch before the first successful bind keeps the gate pending
  // (it rejects evidence but does not latch invalid), because the producer may
  // still be publishing its previous static pair. A change after a successful
  // bind latches invalid immediately, before any dynamic evidence checks, so a
  // stale/incomplete/non-current scene carrying a new source can never leave
  // the old terrain plannable or committable.
  SceneAdmission Evaluate(
    const std::shared_ptr<const perception_3d::SceneSnapshot> & scene,
    const std::string & expected_frame,
    std::int64_t now_steady_ns,
    std::int64_t max_receipt_age_ns);

private:
  enum class State : std::uint8_t
  {
    kAwaitingExpected = 0,
    kAwaitingScene,
    kBound,
    kInvalid,
  };

  perception_3d::SourceMapIdentity expected_input_;
  bool expected_input_known_{false};
  perception_3d::SourceMapIdentity bound_source_map_;
  State state_{State::kAwaitingExpected};
  std::string invalid_detail_;
};

}  // namespace grid_global_planner

#endif  // GRID_GLOBAL_PLANNER__SCENE_SNAPSHOT_ADAPTER_H_
