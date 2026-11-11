// Copyright 2026 DDDMobileRobot
//
// Implementation of the compatibility-only immutable perception observation
// export. Everything here is standard-library only.

#include <perception_3d/perception_snapshot.h>

#include <utility>

namespace perception_3d
{
namespace
{

// Sets an optional diagnostic string without requiring a non-null pointer.
void SetDetail(std::string * detail, const std::string & value)
{
  if (detail != nullptr) {
    *detail = value;
  }
}

// Returns true when a volume carries explicit known bounds.
bool VolumeIsKnown(const PerceptionObservedVolume & volume) noexcept
{
  return volume.known && volume.min.IsFinite() && volume.max.IsFinite() &&
         volume.min.x <= volume.max.x && volume.min.y <= volume.max.y &&
         volume.min.z <= volume.max.z;
}

// Returns true only for a known volume that explicitly declares free space.
// A known occupied extent (free_space == false) is never free/clear evidence.
bool VolumeIsFree(const PerceptionObservedVolume & volume) noexcept
{
  return VolumeIsKnown(volume) && volume.free_space;
}

// Validates one explicit volume. An unknown volume cannot claim free space or
// headroom. A known volume must be finite and ordered; a known headroom must be
// finite, non-negative and backed by a known volume.
bool IsVolumeValid(const PerceptionObservedVolume & volume) noexcept
{
  if (!volume.known) {
    return !volume.free_space && !volume.headroom_known;
  }
  if (!volume.min.IsFinite() || !volume.max.IsFinite()) {
    return false;
  }
  if (volume.min.x > volume.max.x || volume.min.y > volume.max.y ||
    volume.min.z > volume.max.z)
  {
    return false;
  }
  if (volume.headroom_known) {
    return std::isfinite(volume.headroom_m) && volume.headroom_m >= 0.0;
  }
  return true;
}

// Returns true when a source has explicit clearing or explicitly-free vertical
// evidence that can justify an empty occupied list. A known occupied extent is
// not removal evidence. Occupied points are handled separately.
bool HasExplicitRemovalEvidence(
  const std::vector<PerceptionPoint> & cleared,
  const std::vector<PerceptionObservedVolume> & known_volumes,
  const PerceptionObservedVolume & headroom) noexcept
{
  if (!cleared.empty()) {
    return true;
  }
  for (const PerceptionObservedVolume & volume : known_volumes) {
    if (VolumeIsFree(volume)) {
      return true;
    }
  }
  return VolumeIsFree(headroom);
}

// Returns true when at least one known volume explicitly declares free space.
bool HasFreeVolumeEvidence(
  const std::vector<PerceptionObservedVolume> & known_volumes,
  const PerceptionObservedVolume & headroom) noexcept
{
  for (const PerceptionObservedVolume & volume : known_volumes) {
    if (VolumeIsFree(volume)) {
      return true;
    }
  }
  return VolumeIsFree(headroom);
}

}  // namespace

PerceptionSourceObservation::PerceptionSourceObservation(
  std::string source_id, std::string frame_id,
  std::uint64_t source_sequence, std::uint64_t terrain_revision,
  bool terrain_revision_known, std::int64_t sensor_stamp_ns,
  std::int64_t received_steady_ns, PerceptionSourceLayer layer,
  PerceptionUpdateKind kind, bool complete,
  std::vector<PerceptionPoint> occupied,
  std::vector<PerceptionPoint> cleared,
  std::vector<PerceptionObservedVolume> known_volumes,
  PerceptionObservedVolume headroom)
: source_id_(std::move(source_id)),
  frame_id_(std::move(frame_id)),
  source_sequence_(source_sequence),
  terrain_revision_(terrain_revision),
  terrain_revision_known_(terrain_revision_known),
  sensor_stamp_ns_(sensor_stamp_ns),
  received_steady_ns_(received_steady_ns),
  layer_(layer),
  kind_(kind),
  complete_(complete),
  occupied_(std::move(occupied)),
  cleared_(std::move(cleared)),
  known_volumes_(std::move(known_volumes)),
  headroom_(headroom) {}

bool PerceptionSourceObservation::HasKnownVolumeEvidence() const noexcept
{
  for (const PerceptionObservedVolume & volume : known_volumes_) {
    if (VolumeIsKnown(volume)) {
      return true;
    }
  }
  return VolumeIsKnown(headroom_);
}

bool PerceptionSourceObservation::IsUsable(std::string * detail) const
{
  if (!complete_) {
    SetDetail(detail, "source observation is incomplete");
    return false;
  }
  if (source_id_.empty()) {
    SetDetail(detail, "source observation has an empty source_id");
    return false;
  }
  if (frame_id_.empty()) {
    SetDetail(detail, "source observation has an empty frame_id");
    return false;
  }
  // A zero sensor stamp is documented static-unknown provenance; negative is
  // invalid. A receipt stamp is required because freshness/TTL use the steady
  // clock rather than the sensor clock.
  if (sensor_stamp_ns_ < 0 || received_steady_ns_ <= 0) {
    SetDetail(detail, "source observation has a missing stamp");
    return false;
  }
  if (terrain_revision_known_ && terrain_revision_ == 0) {
    SetDetail(detail, "source observation claims a zero terrain revision");
    return false;
  }

  const bool has_occupied = !occupied_.empty();
  const bool has_cleared = !cleared_.empty();
  const bool has_free_volume_evidence =
    HasFreeVolumeEvidence(known_volumes_, headroom_);
  // An empty complete replacement is accepted only with explicit clearing or
  // explicitly-free-volume evidence; otherwise it would erase occupancy without
  // proof. A known occupied extent is not free/clear evidence. An empty exact
  // delta with no evidence carries no information at all.
  if (kind_ == PerceptionUpdateKind::kCompleteReplacement && !has_occupied &&
    !HasExplicitRemovalEvidence(cleared_, known_volumes_, headroom_))
  {
    SetDetail(detail, "empty complete replacement has no removal evidence");
    return false;
  }
  if (kind_ == PerceptionUpdateKind::kExactDelta && !has_occupied &&
    !has_cleared && !has_free_volume_evidence)
  {
    SetDetail(detail, "empty exact delta has no clearing or volume evidence");
    return false;
  }

  // Every frozen point must be finite. The exporter skips non-finite input
  // instead of publishing a partially valid observation.
  for (const PerceptionPoint & point : occupied_) {
    if (!point.IsFinite()) {
      SetDetail(detail, "source observation contains a non-finite occupied point");
      return false;
    }
  }
  for (const PerceptionPoint & point : cleared_) {
    if (!point.IsFinite()) {
      SetDetail(detail, "source observation contains a non-finite clearing point");
      return false;
    }
  }

  for (const PerceptionObservedVolume & volume : known_volumes_) {
    if (!IsVolumeValid(volume)) {
      SetDetail(detail, "source observation contains an invalid known volume");
      return false;
    }
  }
  if (!IsVolumeValid(headroom_)) {
    SetDetail(detail, "source observation contains invalid headroom evidence");
    return false;
  }

  if (detail != nullptr) {
    detail->clear();
  }
  return true;
}

PerceptionSnapshot::PerceptionSnapshot(
  std::string frame_id, std::uint64_t sequence,
  std::uint64_t terrain_revision, bool terrain_revision_known,
  std::int64_t sensor_stamp_ns, std::int64_t received_steady_ns,
  bool sensors_current, bool complete,
  std::vector<PerceptionSourceObservation> sources, std::string detail)
: PerceptionSnapshot(std::move(frame_id), sequence, terrain_revision,
    terrain_revision_known, sensor_stamp_ns,
    sensor_stamp_ns, received_steady_ns, sensors_current,
    complete, std::move(sources), std::move(detail)) {}

PerceptionSnapshot::PerceptionSnapshot(
  std::string frame_id, std::uint64_t sequence,
  std::uint64_t terrain_revision, bool terrain_revision_known,
  std::int64_t sensor_stamp_ns, std::int64_t cycle_stamp_ns,
  std::int64_t received_steady_ns, bool sensors_current, bool complete,
  std::vector<PerceptionSourceObservation> sources, std::string detail)
: frame_id_(std::move(frame_id)),
  sequence_(sequence),
  terrain_revision_(terrain_revision),
  terrain_revision_known_(terrain_revision_known),
  sensor_stamp_ns_(sensor_stamp_ns),
  cycle_stamp_ns_(cycle_stamp_ns),
  received_steady_ns_(received_steady_ns),
  sensors_current_(sensors_current),
  complete_(complete),
  sources_(std::move(sources)),
  detail_(std::move(detail)) {}

std::size_t PerceptionSnapshot::occupied_point_count() const noexcept
{
  std::size_t count = 0;
  for (const PerceptionSourceObservation & source : sources_) {
    count += source.occupied().size();
  }
  return count;
}

std::size_t PerceptionSnapshot::cleared_point_count() const noexcept
{
  std::size_t count = 0;
  for (const PerceptionSourceObservation & source : sources_) {
    count += source.cleared().size();
  }
  return count;
}

bool PerceptionSnapshot::HasKnownVolumeEvidence() const noexcept
{
  for (const PerceptionSourceObservation & source : sources_) {
    if (source.HasKnownVolumeEvidence()) {
      return true;
    }
  }
  return false;
}

bool PerceptionSnapshot::IsUsable(std::string * detail) const
{
  if (!complete_) {
    SetDetail(
      detail, detail_.empty() ? "perception snapshot is incomplete" :
      detail_);
    return false;
  }
  if (!sensors_current_) {
    SetDetail(detail, "perception snapshot has an expired or missing sensor");
    return false;
  }
  if (frame_id_.empty()) {
    SetDetail(detail, "perception snapshot has an empty frame_id");
    return false;
  }
  // isCurrent() above is only a liveness gate. It is not a substitute for
  // per-source sequence ordering, TTL or receipt freshness.
  if (sensor_stamp_ns_ < 0 || received_steady_ns_ <= 0) {
    SetDetail(detail, "perception snapshot has a missing stamp");
    return false;
  }
  if (terrain_revision_known_ && terrain_revision_ == 0) {
    SetDetail(detail, "perception snapshot claims a zero terrain revision");
    return false;
  }
  if (sources_.empty()) {
    SetDetail(detail, "perception snapshot has no source contributions");
    return false;
  }

  for (const PerceptionSourceObservation & source : sources_) {
    std::string source_detail;
    if (!source.IsUsable(&source_detail)) {
      SetDetail(
        detail, "perception source '" + source.source_id() +
        "' is unusable: " + source_detail);
      return false;
    }
    if (source.frame_id() != frame_id_) {
      SetDetail(
        detail, "perception source '" + source.source_id() +
        "' frame does not match the snapshot frame");
      return false;
    }
  }

  // A usable snapshot must contain occupancy, clearing, or explicit known-volume
  // evidence. An export that only reports an empty cloud is unknown, not free.
  if (occupied_point_count() == 0 && cleared_point_count() == 0 &&
    !HasKnownVolumeEvidence())
  {
    SetDetail(
      detail,
      "perception snapshot has no occupancy, clearing or volume evidence");
    return false;
  }

  if (detail != nullptr) {
    detail->clear();
  }
  return true;
}

bool PerceptionSnapshot::IsFresh(
  std::int64_t now_steady_ns,
  std::int64_t max_receipt_age_ns,
  std::string * detail) const
{
  if (!IsUsable(detail)) {
    return false;
  }
  if (max_receipt_age_ns < 0) {
    SetDetail(detail, "negative max receipt age is invalid");
    return false;
  }
  if (now_steady_ns < received_steady_ns_) {
    SetDetail(detail, "perception snapshot receipt time is in the future");
    return false;
  }
  if (now_steady_ns - received_steady_ns_ > max_receipt_age_ns) {
    SetDetail(detail, "perception snapshot is stale");
    return false;
  }
  if (detail != nullptr) {
    detail->clear();
  }
  return true;
}

const char * PerceptionSourceLayerToString(PerceptionSourceLayer layer) noexcept
{
  switch (layer) {
    case PerceptionSourceLayer::kStatic:
      return "static";
    case PerceptionSourceLayer::kDynamic:
      return "dynamic";
  }
  return "unknown";
}

const char * PerceptionUpdateKindToString(PerceptionUpdateKind kind) noexcept
{
  switch (kind) {
    case PerceptionUpdateKind::kCompleteReplacement:
      return "complete_replacement";
    case PerceptionUpdateKind::kExactDelta:
      return "exact_delta";
  }
  return "unknown";
}

}  // namespace perception_3d
