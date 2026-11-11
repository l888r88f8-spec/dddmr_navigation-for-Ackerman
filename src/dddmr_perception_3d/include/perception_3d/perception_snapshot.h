// Copyright 2026 DDDMobileRobot
//
// Compatibility-only immutable perception observation export.
//
// This file deliberately depends only on the C++ standard library. It is the
// hand-off boundary between perception_3d and a downstream grid planner: the
// exporter copies every point out of PCL-owned storage, so a later plugin write
// can never alias a published snapshot.
//
// Evidence contract:
// - Occupied points prove only that the source observed an obstacle at those
//   coordinates. They never prove free volume, ground clearance or headroom.
// - Cleared records are explicit evidence that a producer observed the listed
//   location as clear.
// - Known volumes are explicit vertical evidence. A volume is free/clear
//   evidence only when it is known and declares free_space == true; a known
//   occupied extent (free_space == false) never becomes free by omission.
// - An empty complete replacement is accepted only when it is backed by such
//   explicit clearing/known-free-volume evidence; an empty observation without
//   evidence is unknown and must be rejected.
// - The legacy plugin exporter emits point-only complete replacements. That is
//   usable occupancy evidence, but it is not a complete free-space/clearance
//   export and must not be described as ready-to-plan free space. It leaves
//   cleared records and known_volumes empty and therefore never proves free
//   volume or headroom. An exporter that cannot verify the point frame or the
//   source acquisition stamp must fail closed rather than relabeling the cloud
//   or substituting the perception update-loop clock.
// - The source frame is the frame the points are already expressed in. A
//   consumer must not relabel an unknown/source-frame cloud as the configured
//   map frame without verified transform provenance.

#ifndef PERCEPTION_3D_PERCEPTION_SNAPSHOT_H_
#define PERCEPTION_3D_PERCEPTION_SNAPSHOT_H_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace perception_3d
{

// The name used by the compatibility instance that feeds the grid planner. It
// is a diagnostic/default identifier only; the ROS node name is selected at
// launch time with the node name `dddmr_grid_perception`.
inline constexpr char kGridPerceptionInstanceName[] = "dddmr_grid_perception";

// Whether a source contributes static or dynamic occupancy. Dynamic records
// must never clear static occupancy in the downstream worker.
enum class PerceptionSourceLayer : std::uint8_t
{
  kStatic = 0,
  kDynamic = 1,
};

// Update semantics for one frozen source observation.
//
// kCompleteReplacement replaces exactly this source's previous contribution:
// the occupied list is the complete current contribution. An empty complete
// replacement is accepted only when explicit clearing or known-volume evidence
// is present; it is otherwise rejected as unknown.
//
// kExactDelta applies only the listed occupied/cleared records. It must carry
// explicit evidence whenever the producer wants to remove occupancy.
enum class PerceptionUpdateKind : std::uint8_t
{
  kCompleteReplacement = 0,
  kExactDelta = 1,
};

// One frozen occupancy point in the snapshot frame. Coordinates are copied by
// value; no PCL pointer or mutable view is retained.
struct PerceptionPoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};

  // Returns true only when all three coordinates are finite.
  bool IsFinite() const noexcept
  {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
  }
};

// Explicit vertical evidence for one observed volume.
//
// known == false means "no observation covers this volume". It never means
// free. headroom_known is independent because a producer may know the observed
// occupied extent without knowing the free clearance above it.
//
// free_space is the producer's explicit declaration that the whole box was
// observed FREE. It is meaningful only when known == true and defaults to
// false so older aggregate/partial initializers remain conservative: a known
// occupied extent never becomes free by omission.
struct PerceptionObservedVolume
{
  PerceptionPoint min{};
  PerceptionPoint max{};
  bool known{false};
  double headroom_m{std::numeric_limits<double>::quiet_NaN()};
  bool headroom_known{false};
  bool free_space{false};
};

// One immutable source contribution frozen after a complete clear/mark cycle.
//
// Construction deep-copies every vector. There is intentionally no mutable
// accessor and no pcl::PointCloud member, so later plugin writes cannot alias a
// published observation.
class PerceptionSourceObservation
{
public:
  // Constructs an empty, incomplete observation. Not a ready-to-use source.
  PerceptionSourceObservation() = default;

  // Freezes one source contribution. All arguments are copied or moved into
  // this object; callers must not expect shared ownership afterwards.
  //
  // sensor_stamp_ns is the acquisition stamp of this source, in the producer
  // clock domain. Zero means unknown and is allowed only as documented static
  // unknown provenance; negative values are invalid.
  PerceptionSourceObservation(
    std::string source_id, std::string frame_id,
    std::uint64_t source_sequence, std::uint64_t terrain_revision,
    bool terrain_revision_known, std::int64_t sensor_stamp_ns,
    std::int64_t received_steady_ns, PerceptionSourceLayer layer,
    PerceptionUpdateKind kind, bool complete,
    std::vector<PerceptionPoint> occupied,
    std::vector<PerceptionPoint> cleared,
    std::vector<PerceptionObservedVolume> known_volumes,
    PerceptionObservedVolume headroom);

  // Source identity. Empty is invalid.
  const std::string & source_id() const noexcept {return source_id_;}
  // Frame of the frozen points. Empty is invalid.
  const std::string & frame_id() const noexcept {return frame_id_;}
  // Producer sequence for ordering/duplicate rejection.
  std::uint64_t source_sequence() const noexcept {return source_sequence_;}
  // Terrain revision this observation was aligned to, or 0 when unknown.
  std::uint64_t terrain_revision() const noexcept {return terrain_revision_;}
  // True only when terrain_revision_ is meaningful for this observation.
  bool terrain_revision_known() const noexcept
  {
    return terrain_revision_known_;
  }
  // Sensor acquisition stamp in nanoseconds. Zero means unknown.
  std::int64_t sensor_stamp_ns() const noexcept {return sensor_stamp_ns_;}
  // True when a genuine source acquisition stamp (not unknown zero) is present.
  bool has_sensor_stamp() const noexcept {return sensor_stamp_ns_ > 0;}
  // Monotonic receipt time in nanoseconds; freshness must use this clock.
  std::int64_t received_steady_ns() const noexcept
  {
    return received_steady_ns_;
  }
  // Static versus dynamic contribution.
  PerceptionSourceLayer layer() const noexcept {return layer_;}
  // Complete replacement versus exact delta.
  PerceptionUpdateKind kind() const noexcept {return kind_;}
  // False when the producer could not freeze a complete observation.
  bool complete() const noexcept {return complete_;}
  // Frozen occupied points. Never a mutable PCL view.
  const std::vector<PerceptionPoint> & occupied() const noexcept
  {
    return occupied_;
  }
  // Frozen explicit clearing points. Empty is not evidence of free space.
  const std::vector<PerceptionPoint> & cleared() const noexcept
  {
    return cleared_;
  }
  // Explicit known volumes. Empty means no vertical free-extent evidence.
  const std::vector<PerceptionObservedVolume> & known_volumes() const noexcept
  {
    return known_volumes_;
  }
  // Overhead/headroom evidence for this source. Unknown by default.
  const PerceptionObservedVolume & headroom() const noexcept
  {
    return headroom_;
  }

  // Number of frozen occupied plus clearing points.
  std::size_t point_count() const noexcept
  {
    return occupied_.size() + cleared_.size();
  }

  // True when no occupied and no clearing evidence exists.
  bool empty() const noexcept
  {
    return occupied_.empty() && cleared_.empty();
  }

  // True when at least one known volume or known headroom carries explicit
  // free/occupied vertical evidence.
  bool HasKnownVolumeEvidence() const noexcept;

  // Structural validation. Rejects an empty observation that has neither
  // records nor explicit volume/headroom evidence, non-finite points, missing
  // identity/frame, invalid stamps and structurally invalid volumes. It does
  // not check wall-clock age; that is the snapshot's IsFresh() responsibility.
  bool IsUsable(std::string * detail = nullptr) const;

private:
  std::string source_id_;
  std::string frame_id_;
  std::uint64_t source_sequence_{0};
  std::uint64_t terrain_revision_{0};
  bool terrain_revision_known_{false};
  std::int64_t sensor_stamp_ns_{0};
  std::int64_t received_steady_ns_{0};
  PerceptionSourceLayer layer_{PerceptionSourceLayer::kDynamic};
  PerceptionUpdateKind kind_{PerceptionUpdateKind::kCompleteReplacement};
  bool complete_{false};
  std::vector<PerceptionPoint> occupied_;
  std::vector<PerceptionPoint> cleared_;
  std::vector<PerceptionObservedVolume> known_volumes_;
  PerceptionObservedVolume headroom_;
};

// One immutable perception export. It owns frozen source observations by value
// and has const accessors only. A snapshot that fails IsUsable() must be
// reported as unknown/insufficient by the consumer; it must never be converted
// into clear space.
class PerceptionSnapshot
{
public:
  // Constructs an empty, incomplete snapshot.
  PerceptionSnapshot() = default;

  // Compatibility constructor. The supplied stamp is used for both the export
  // cycle and each source's acquisition stamp. New exporters should use the
  // cycle/source-separated constructor below so per-source provenance is kept.
  PerceptionSnapshot(
    std::string frame_id, std::uint64_t sequence,
    std::uint64_t terrain_revision, bool terrain_revision_known,
    std::int64_t sensor_stamp_ns, std::int64_t received_steady_ns,
    bool sensors_current, bool complete,
    std::vector<PerceptionSourceObservation> sources,
    std::string detail = std::string());

  // Freezes one export with an explicit cycle stamp. sensor_stamp_ns is the
  // newest per-source acquisition stamp; cycle_stamp_ns is the perception
  // update-cycle stamp and must not be substituted for source acquisition
  // stamps.
  PerceptionSnapshot(
    std::string frame_id, std::uint64_t sequence,
    std::uint64_t terrain_revision, bool terrain_revision_known,
    std::int64_t sensor_stamp_ns, std::int64_t cycle_stamp_ns,
    std::int64_t received_steady_ns, bool sensors_current, bool complete,
    std::vector<PerceptionSourceObservation> sources,
    std::string detail = std::string());

  // Frame shared by all frozen points.
  const std::string & frame_id() const noexcept {return frame_id_;}
  // Monotonic export sequence for this perception instance.
  std::uint64_t sequence() const noexcept {return sequence_;}
  // Terrain revision aligned by the caller, or 0 when unknown.
  std::uint64_t terrain_revision() const noexcept {return terrain_revision_;}
  // True only when terrain_revision_ is meaningful.
  bool terrain_revision_known() const noexcept
  {
    return terrain_revision_known_;
  }
  // Newest per-source acquisition stamp carried by the export. Zero means
  // unknown when no source supplied a genuine stamp.
  std::int64_t sensor_stamp_ns() const noexcept {return sensor_stamp_ns_;}
  // Perception update-cycle stamp. This is separate from source acquisition
  // stamps and must not be used as a sensor ordering/TTL substitute.
  std::int64_t cycle_stamp_ns() const noexcept {return cycle_stamp_ns_;}
  // Monotonic receipt/export time.
  std::int64_t received_steady_ns() const noexcept
  {
    return received_steady_ns_;
  }
  // True only when every loaded sensor reported a current observation.
  bool sensors_current() const noexcept {return sensors_current_;}
  // Producer completeness flag. False forces IsUsable() to fail.
  bool complete() const noexcept {return complete_;}
  // Frozen per-source contributions.
  const std::vector<PerceptionSourceObservation> & sources() const noexcept
  {
    return sources_;
  }
  // Human-readable exporter status/diagnostic detail.
  const std::string & detail() const noexcept {return detail_;}

  // True when there is no source evidence at all.
  bool empty() const noexcept {return sources_.empty();}

  // Number of frozen occupancy points across all sources.
  std::size_t occupied_point_count() const noexcept;

  // Number of frozen clearing points across all sources.
  std::size_t cleared_point_count() const noexcept;

  // True when any source carries explicit known-volume or headroom evidence.
  bool HasKnownVolumeEvidence() const noexcept;

  // Structural and provenance validation. Rejects incomplete/expired sensor
  // state, missing frame, empty sources, unusable sources and snapshots with no
  // occupancy, clearing or explicit volume evidence. A caller that needs a
  // maximum age must also call IsFresh() with the same monotonic clock used for
  // received_steady_ns.
  bool IsUsable(std::string * detail = nullptr) const;

  // True only when the snapshot is structurally usable and its receipt age is
  // within max_receipt_age_ns. Negative age (future receipt) is not fresh.
  bool IsFresh(
    std::int64_t now_steady_ns, std::int64_t max_receipt_age_ns,
    std::string * detail = nullptr) const;

private:
  std::string frame_id_;
  std::uint64_t sequence_{0};
  std::uint64_t terrain_revision_{0};
  bool terrain_revision_known_{false};
  std::int64_t sensor_stamp_ns_{0};
  std::int64_t cycle_stamp_ns_{0};
  std::int64_t received_steady_ns_{0};
  bool sensors_current_{false};
  bool complete_{false};
  std::vector<PerceptionSourceObservation> sources_;
  std::string detail_;
};

// Stable diagnostic spellings used in logs/tests.
const char * PerceptionSourceLayerToString(PerceptionSourceLayer layer) noexcept;
const char * PerceptionUpdateKindToString(PerceptionUpdateKind kind) noexcept;

}  // namespace perception_3d

#endif  // PERCEPTION_3D_PERCEPTION_SNAPSHOT_H_
