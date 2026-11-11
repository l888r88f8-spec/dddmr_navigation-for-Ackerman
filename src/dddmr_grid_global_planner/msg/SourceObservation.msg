# Explicit source occupancy evidence for the grid global planner.
#
# header.stamp is the true per-source acquisition stamp supplied by the
# producer. header.frame_id is the frame the records are already expressed in;
# this planner accepts only its configured map frame and rejects frames that
# would require an unsupported transform. Empty/no stamp is not silently
# replaced with the receipt time or with the perception update-loop clock.
#
# occupied records are obstacles observed by this source. cleared records are
# explicit evidence that the listed location was observed clear. known_volumes
# carry explicit vertical free/extent evidence; point-only occupancy never
# proves free volume or headroom.
std_msgs/Header header

# Stable producer source identity and ordering.
string source_id
uint64 source_sequence

# Terrain revision this evidence was aligned to. terrain_revision_known must be
# true for the revision to participate in ordering/validation.
uint64 terrain_revision
bool terrain_revision_known

# Static or dynamic contribution.
uint8 layer
uint8 LAYER_STATIC=0
uint8 LAYER_DYNAMIC=1

# Complete replacement or exact delta semantics.
uint8 update_kind
uint8 UPDATE_COMPLETE_REPLACEMENT=0
uint8 UPDATE_EXACT_DELTA=1

# False means the producer could not freeze a complete observation.
bool complete

SourceOccupancyRecord[] occupied
SourceOccupancyRecord[] cleared
ObservedVolume[] known_volumes
ObservedVolume headroom
