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

#ifndef GRID_GLOBAL_PLANNER__WHEEL_SUPPORT_H_
#define GRID_GLOBAL_PLANNER__WHEEL_SUPPORT_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

#include "grid_global_planner/distance_field.h"
#include "grid_global_planner/map_snapshot.h"
#include "grid_global_planner/metrics.h"
#include "grid_global_planner/terrain_types.h"

namespace grid_global_planner
{

  inline constexpr std::size_t kWheelCount = 4;

// Fixed wheel order used by every branch signature and support record.
  enum class WheelIndex: std::uint8_t
  {
    kFrontLeft = 0,
    kFrontRight = 1,
    kRearLeft = 2,
    kRearRight = 3,
  };

// Stable spelling for diagnostics and tests.
  const char * WheelIndexToString(WheelIndex wheel) noexcept;

// Configuration state. A missing measured physical parameter is reported
// explicitly and must fail closed; it is never replaced by an assumed value.
  enum class VehicleConfigurationStatus: std::uint8_t
  {
    kConfigured = 0,
    kVehicleCapabilityUnconfigured,
    kInvalidGeometry,
    kInconsistentSteeringGeometry,
    kInvalidThresholds,
  };

  const char * VehicleConfigurationStatusToString(
    VehicleConfigurationStatus status) noexcept;

// Vehicle geometry and capability configuration.
//
// The base-frame wheel contact offsets, wheelbase, track width, base contact
// height and steering limits below are the values confirmed from the project
// input manifest. They are geometry evidence, not a complete vehicle
// capability. Tire radius/width, suspension/tire compliance and the full body
// maximum envelope are deliberately NaN/unset until measured; Validate() then
// returns kVehicleCapabilityUnconfigured and the planner must refuse to plan.
  struct VehicleCapabilityConfig
  {
    // Confirmed wheel-contact geometry in base_link at rest (front-left,
    // front-right, rear-left, rear-right). Z is -base_contact_height.
    double wheelbase_m {0.55};

    // Legacy aggregate track width retained for source compatibility. It is not
    // used to orient the front wheels; front_track_width_m is authoritative there.
    double track_width_m {0.52};

    // Independent front and rear track widths. Both are confirmed as 0.52 m for
    // this vehicle, but they are stored separately so a future measured axle can
    // be configured without editing the wheel contact offsets by hand.
    double front_track_width_m {0.52};
    double rear_track_width_m {0.52};

    std::array < Point3, kWheelCount > wheel_contact_offsets {{
        Point3 {0.275, 0.26, -0.22},
        Point3 {0.275, -0.26, -0.22},
        Point3 {-0.275, 0.26, -0.22},
        Point3 {-0.275, -0.26, -0.22},
      }};
    double base_contact_height_m {0.22};

    // Confirmed steering limits (central and inner front wheel).
    double max_steer_central_rad {0.3270520925102273};
    double max_steer_inner_rad {0.3839724354387525};

    // Measured physical parameters. NaN means "not measured in this checkout".
    double wheel_radius_m {std::numeric_limits < double > ::quiet_NaN()};
    double tire_width_m {std::numeric_limits < double > ::quiet_NaN()};
    double suspension_travel_m {std::numeric_limits < double > ::quiet_NaN()};
    double tire_compliance_m {std::numeric_limits < double > ::quiet_NaN()};

    // Optional explicit contact-patch length. When unset, 2 * wheel_radius is used
    // as a conservative tire projection; the sample footprint is still finite.
    // An explicit patch length permits validation without a measured radius, but
    // a missing tire width still leaves the capability unconfigured.
    double contact_patch_length_m {std::numeric_limits < double > ::quiet_NaN()};

    // Lower body envelope is known only approximately. The full upper envelope is
    // unset, so the capability remains unconfigured until it is measured.
    Vector3 body_min {-0.41, -0.32, -0.10};
    bool body_min_configured {true};
    bool body_min_approximate {true};
    Vector3 body_max {};
    bool body_max_configured {false};

    // Terrain/vehicle limits already present in the project's local-map config.
    // They are configured acceptance limits, not measured suspension/tire
    // compliance.
    double min_contact_coverage {0.60};
    double max_pitch_rad {0.40};
    double max_roll_rad {0.30};
    double max_step_m {0.12};
    double max_warp_m {0.05};
    double max_pose_residual_m {0.05};
    double body_clearance_m {0.12};
    double max_branch_jump_m {0.12};

    // Returns the front axle midpoint X coordinate from the FL/FR contacts.
    double FrontAxleMidpointX() const noexcept;

    // Returns the rear axle midpoint X coordinate from the RL/RR contacts.
    double RearAxleMidpointX() const noexcept;

    // Returns the wheelbase measured between the front and rear axle midpoints.
    // A non-finite or non-positive result means the contact offsets are invalid.
    double MeasuredWheelbaseM() const noexcept;

    // Returns the front track measured from the FL/FR contact offsets.
    double MeasuredFrontTrackWidthM() const noexcept;

    // Returns the rear track measured from the RL/RR contact offsets.
    double MeasuredRearTrackWidthM() const noexcept;

    // Returns wheel contact offsets composed from the configured wheelbase,
    // independent front/rear tracks and base contact height. The base origin is
    // midway between the axles. All-zero offsets are returned when the composing
    // geometry is invalid.
    std::array < Point3, kWheelCount > ComposeWheelContactOffsets() const noexcept;

    // Replaces wheel_contact_offsets with ComposeWheelContactOffsets(). Returns
    // false and leaves the offsets unchanged when the composing geometry is
    // invalid; a stable explanation is written to detail when supplied.
    bool DeriveWheelContactOffsets(std::string * detail = nullptr);

    // Validates only the frame/geometry, steering and acceptance-threshold
    // inputs that both traversability models consume. It deliberately does not
    // require the measured tire/suspension/body_max capability, so the explicit
    // offline model can reuse this validation without inventing physical values.
    // The full measured validation remains Validate() below.
    VehicleConfigurationStatus ValidateGeometryAndThresholds(
      std::string * detail = nullptr) const;

    // Validates ranges and required measured parameters. The optional detail
    // string receives a stable explanation on failure.
    VehicleConfigurationStatus Validate(std::string * detail = nullptr) const;

    // True only when calibration parameters permit a support/collision decision.
    bool IsConfigured() const
    {
      return Validate(nullptr) == VehicleConfigurationStatus::kConfigured;
    }
  };

// Exact Ackermann front-wheel orientation result. Front-left/front-right are
// expressed relative to the base heading. A left turn has a positive central
// angle and the left wheel is the inner wheel.
  struct AckermannSteeringAngles
  {
    bool valid {false};
    double front_left_rad {0.0};
    double front_right_rad {0.0};
    double inner_rad {0.0};
    double outer_rad {0.0};
    std::string detail;
  };

// Solves the Ackermann inner/outer angles from one central steering angle. The
// result is rejected rather than clamped when the turn radius is degenerate or
// the configured inner limit is inconsistent.
  AckermannSteeringAngles SolveAckermannSteering(
    double central_rad, const VehicleCapabilityConfig & vehicle) noexcept;

// One solved wheel support branch.
  struct WheelSupport
  {
    bool valid {false};
    TerrainNodeId node_id {kInvalidTerrainNodeId};
    FineCellIndex cell {};
    int local_surface_index {-1};
    Point3 contact {};
    // True only when this node's coverage was measured from a finite tire
    // contact patch. The offline kinematic/static-map model does not simulate
    // that patch, so it leaves coverage unknown (false/NaN) instead of claiming
    // full physical support.
    bool coverage_known {false};
    double coverage {std::numeric_limits < double > ::quiet_NaN()};
    double contact_residual_m {0.0};
    double sample_min_z {0.0};
    double sample_max_z {0.0};
  };

// Why a support/collision evaluation was rejected.
  enum class SupportRejection: std::uint8_t
  {
    kNone = 0,
    kVehicleCapabilityUnconfigured,
    kInvalidQuery,
    kSnapshotMismatch,
    kNoTerrain,
    kInsufficientFootprintCoverage,
    kUnknownTerrain,
    kProhibitedTerrain,
    kHole,
    kHeightStep,
    kTwistExceeded,
    kResidualExceeded,
    kTravelExceeded,
    kPitchExceeded,
    kRollExceeded,
    kInconsistentDecks,
    kBranchJump,
    kChassisCollision,
    kObstacleCollision,
    kOverheadCollision,
    kUnknownObstacle,
    kPoseSolveBudget,
    kUnsupportedWheel,
  };

// Explicit traversability model. The production default remains the full
// measured four-wheel/tire/suspension/body model. The offline static-map model
// is a separate evaluation policy for PCD-only algorithm tests: it checks
// Ackermann contact geometry, measured map connectivity, slope/step and
// static obstacles, but it must never be reported as real-vehicle feasibility.
  enum class TraversabilityModel: std::uint8_t
  {
    kFullWheel = 0,
    kOfflineKinematicStaticMap,
  };

  const char * TraversabilityModelToString(
    TraversabilityModel model) noexcept;

  const char * SupportRejectionToString(SupportRejection rejection) noexcept;

// Solved full vehicle attitude. base_origin is the base_link origin in world.
  struct VehiclePose
  {
    Point3 base_origin {};
    double yaw {0.0};
    double roll {0.0};
    double pitch {0.0};
  };

// One four-wheel support and body-clearance evaluation. The same evaluator is
// used for search primitives, goal connection and final path validation.
  struct WheelSupportEvaluation
  {
    // The named traversability model that produced this result. This is evidence
    // provenance, not a success flag.
    TraversabilityModel model {TraversabilityModel::kFullWheel};
    // True only when every selected wheel coverage is backed by a finite
    // measured tire contact patch. The offline kinematic model deliberately
    // reports false and leaves WheelSupport::coverage unknown.
    bool physical_coverage_known {false};
    SupportRejection rejection {SupportRejection::kNone};
    bool feasible {false};
    std::string detail;
    VehiclePose pose {};
    std::array < WheelSupport, kWheelCount > wheels {};
    std::array < TerrainNodeId, kWheelCount > branch_nodes {};
    double roll_rad {0.0};
    double pitch_rad {0.0};
    double residual_rms_m {0.0};
    double twist_m {0.0};
    double max_travel_m {0.0};
    double chassis_min_clearance_m {std::numeric_limits < double > ::quiet_NaN()};
    double overhead_min_clearance_m {
      std::numeric_limits < double > ::quiet_NaN()
    };
    double cost {0.0};
    // Actual measured evaluator sections and instrumented lookup counts. The
    // counts are lower bounds: body-envelope internals are not individually
    // counted.
    WheelSupportTiming timing {};

    // True only for the full measured model when all four wheel branches, the
    // solved attitude and their measured contact-patch coverage are valid. It is
    // intentionally false for the offline kinematic/static-map model, whose
    // evidence does not include physical tire-patch coverage.
    bool HasFourWheelSupport() const noexcept;

    // True only for the explicit offline kinematic/static-map model when all four
    // Ackermann contact branches are valid. Search and validation consume
    // `feasible` directly; this predicate names the narrower evidence they
    // actually have. It is never a real-vehicle feasibility claim.
    bool HasOfflineKinematicSupport() const noexcept;
  };

// One support query. center.z is only a projection seed; the solver computes
// the full base origin height from the four contacts.
  struct WheelSupportQuery
  {
    Point3 center {};
    double heading {0.0};
    double steering_central_rad {0.0};
    bool has_previous {false};
    WheelSupportEvaluation previous {};
  };

// Deterministic sampling/solve budgets. These are engineering budgets, not
// vehicle capability claims, and every one is documented here.
  struct WheelSupportOptions
  {
    // Nominal finite tire footprint sample counts. The evaluator raises these
    // counts when the fine resolution requires denser evidence and always inserts
    // the exact geometric footprint center, including for even nominal counts.
    std::size_t footprint_samples_along {5};
    std::size_t footprint_samples_across {3};

    // Branch-combination and nonlinear pose-solve iteration ceilings.
    std::size_t max_branch_combinations {256};
    std::size_t max_pose_iterations {10};

    // Support identity/evidence tolerances.
    double branch_height_tolerance_m {0.03};
    double min_center_support {1.0};
    double max_contact_height_spread_m {0.05};

    // Connectivity search bound for the four selected deck nodes.
    std::size_t max_deck_search_nodes {4096};

    // Full body envelope sampling spacing (metres).
    double body_sample_spacing_m {0.10};

    // Obstacle soft-cost range and weights.
    double obstacle_penalty_distance_m {0.50};
    double obstacle_penalty_weight {1.0};
    double slope_penalty_weight {1.0};
    double residual_penalty_weight {10.0};
    double clearance_penalty_weight {1.0};

    // Safe-default headroom policy. When true (the default), every selected
    // ground surface under the body must carry explicit observed headroom
    // (clearance_known and a finite clearance_above_m) covering the body top.
    // Absence of overhead point observations is not proof of free headroom.
    // Setting this false is an explicit unsafe opt-in for synthetic maps; known
    // overhead nodes still reject when their measured headroom intersects the
    // body.
    bool require_observed_headroom {true};

    // Traversability model selection. kFullWheel keeps the production behavior
    // exactly as before. kOfflineKinematicStaticMap uses the explicit planar
    // collision outline below instead of the missing measured tire/suspension/
    // body_max values, and is only valid with the offline PCD snapshot.
    TraversabilityModel model {TraversabilityModel::kFullWheel};

    // Offline kinematic/static-map model. When both measured body XY bounds are
    // configured, the evaluator samples the actual measured rectangle and these
    // radius/spacing values are not used for the outline. Otherwise the planar
    // circle radius is a conservative fallback derived from the measured body
    // corners; the default 0.522 m covers hypot(0.410, 0.322) and is not a
    // substitute for the full measured envelope. The body sample spacing is a
    // deterministic integration budget and is clamped to the fine resolution so
    // no fine cell inside the footprint can be skipped.
    double offline_planar_collision_radius_m {0.522};
    double offline_body_sample_spacing_m {0.10};
    // Contact heights from different wheel cells must lie within this band when
    // fitting the offline Ackermann attitude. This is a map-consistency gate,
    // not a measured suspension travel.
    double offline_contact_height_tolerance_m {0.10};

    // Temporary, explicitly named opt-out of whole-vehicle ground support for
    // the offline kinematic/static-map model only. When true (the default) the
    // offline model keeps its nominal contract: four actual wheel cells, one
    // connected measured deck and an actual footprint ground cell under every
    // sampled point. When false, the pose is fitted from the actual supported
    // center plane and four virtual contact heights, so missing wheel cells and
    // absent same-deck evidence are not by themselves rejections. Obstacle/body
    // collision, prohibited terrain, real raised terrain (a height step), pitch/
    // roll limits and the center terrain requirement remain enforced. This flag
    // never claims measured wheel contact and leaves physical_coverage_known
    // false. It is ignored by the full-wheel model.
    bool offline_check_ground_support {true};

    // Offline terrain-boundary soft cost. The immutable terrain snapshot caches a
    // horizontal graph distance to the nearest missing/disconnected/prohibited
    // support boundary. Every accepted body-footprint cell contributes
    // TerrainBoundaryPenalty(distance, range, weight); the evaluator adds the
    // worst covered-cell value to the offline slope/RMS cost. This is a
    // topology-based route preference, not a collision-clearance certificate.
    // It applies only to kOfflineKinematicStaticMap; the full-wheel model keeps
    // its existing obstacle/clearance cost terms. Zero range or weight explicitly
    // disables this term. Values must be finite and non-negative.
    double boundary_penalty_distance_m {0.50};
    double boundary_penalty_weight {1.0};
  };

// Stateless four-wheel support evaluator. All scratch buffers are function-local
// so one evaluator can be reused by a planning job without hidden mutable state.
  class WheelSupportEvaluator
  {
public:
    WheelSupportEvaluator(
      VehicleCapabilityConfig vehicle,
      WheelSupportOptions options = {});

    const VehicleCapabilityConfig & vehicle() const noexcept {return vehicle_;}
    const WheelSupportOptions & options() const noexcept {return options_;}

    VehicleConfigurationStatus configuration_status() const noexcept
    {
      return configuration_status_;
    }
    const std::string & configuration_detail() const noexcept
    {
      return configuration_detail_;
    }
    bool IsConfigured() const noexcept
    {
      return configuration_status_ == VehicleConfigurationStatus::kConfigured;
    }

    // Selected traversability model. IsConfigured() means "this named model has
    // its required inputs", not "measured full-vehicle capability".
    TraversabilityModel model() const noexcept {return options_.model;}

    // Evaluates four-wheel ground contact, joint support branch, vehicle z/roll/
    // pitch, twist/travel, deck continuity, body/chassis clearance and obstacle
    // and overhead collision. It reads the supplied immutable snapshot only.
    WheelSupportEvaluation Evaluate(
      const MapSnapshot & map,
      const ObstacleDistanceField & distance_field,
      const WheelSupportQuery & query) const;

private:
    VehicleCapabilityConfig vehicle_;
    WheelSupportOptions options_;
    VehicleConfigurationStatus configuration_status_;
    std::string configuration_detail_;
  };

}  // namespace grid_global_planner

#endif  // GRID_GLOBAL_PLANNER__WHEEL_SUPPORT_H_
