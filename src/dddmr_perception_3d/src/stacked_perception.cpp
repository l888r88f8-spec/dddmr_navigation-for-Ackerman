/*
* BSD 3-Clause License

* Copyright (c) 2024, DDDMobileRobot

* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:

* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.

* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.

* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.

* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <perception_3d/stacked_perception.h>

#include <atomic>
#include <cmath>
#include <limits>
#include <utility>

namespace perception_3d
{


StackedPerception::StackedPerception(const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& m_logger){
  shared_data_ = std::make_shared<perception_3d::SharedData>();
  access_ = new mutex_t();
  logger_ = m_logger;
  return;
}

StackedPerception::~StackedPerception()
{
  while (plugins_.size() > 0)
  {
    plugins_.pop_back();
  }
  shared_data_.reset();
  delete access_;
}

/*
Plugin operations
*/
void StackedPerception::addPlugin(std::shared_ptr<Sensor> plugin)
{
  plugins_.push_back(plugin);
  plugin->setSharedData(shared_data_);
}

std::vector<std::shared_ptr<Sensor>>* StackedPerception::getPlugins()
{
  return &plugins_;
}

/*
Collect marked pcl from all plugins.
*/

void StackedPerception::doClear_then_Mark(){
  std::unique_lock<mutex_t> lock(*access_);

  /*
  Say we want a velodyne plus realsense, we want realsense to clear what velodyne saw or vise versa.
  Three step for this perception 3d:
  1. aggregated all observed point clouds
  2. clear marking using kd-tree and observation space, that is why we can clearing from other sensors
  3. marking observations
  */

  for (std::vector<std::shared_ptr<Sensor> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin)
  {
    if(!(*plugin)){
      continue;
    }
    (*plugin)->selfClear();
    (*plugin)->selfMark();
    (*plugin)->updateLethalPointCloud();
  }

}

void StackedPerception::resetdGraph(){
  std::unique_lock<mutex_t> lock(*access_);

  for (std::vector<std::shared_ptr<Sensor> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin)
  {
    if(!(*plugin)){
      continue;
    }

    (*plugin)->resetdGraph();

  }

}

unsigned long StackedPerception::getdGraphSize(){
  unsigned long tmp_val = 1000000;
  for (std::vector<std::shared_ptr<Sensor> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin)
  {
    if(!(*plugin)){
      continue;
    }
    tmp_val = std::min(tmp_val, (*plugin)->get_dGraphSize());
  }
  return tmp_val;
}

double StackedPerception::get_min_dGraphValue(const unsigned int index){
  
  //@ Everything should be less that 100 km?
  double tmp_val = 99999.9;

  for (std::vector<std::shared_ptr<Sensor> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin)
  {
    if(!(*plugin)){
      continue;
    }
    RCLCPP_DEBUG(logger_->get_logger(), "%s with dGraph value of: %.1f",(*plugin)->getName().c_str(),(*plugin)->get_dGraphValue(index));
    tmp_val = std::min(tmp_val, (*plugin)->get_dGraphValue(index));
  }
  return tmp_val;
}

void StackedPerception::aggregateObservations(){
  refreshObservationCache();
}

void StackedPerception::refreshObservationCache(){
  if(!shared_data_){
    return;
  }

  static rclcpp::Clock refresh_timing_clock(RCL_STEADY_TIME);
  const auto refresh_started_at = std::chrono::steady_clock::now();
  std::unique_lock<mutex_t> lock(*access_);
  const auto refresh_access_acquired_at = std::chrono::steady_clock::now();
  const auto refresh_ground_before = std::chrono::steady_clock::now();
  std::unique_lock<std::recursive_mutex> sensor_lock(shared_data_->ground_kdtree_cb_mutex_);
  const auto refresh_ground_acquired_at = std::chrono::steady_clock::now();

  auto aggregate_observation = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  auto aggregate_observation_kdtree = std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZI>>();
  std::string frame_id;
  bool stale = plugins_.empty();
  bool has_source_stamp = false;
  //@ PCL header stamps are microseconds. Keep the oldest non-zero observation
  //@ stamp; the rebuild itself never contributes a timestamp.
  std::uint64_t source_stamp_us = 0;

  for (std::vector<std::shared_ptr<Sensor> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin)
  {
    if(!(*plugin)){
      //@ A null plugin cannot prove freshness.
      stale = true;
      continue;
    }

    if(!(*plugin)->isCurrent()){
      stale = true;
    }

    auto observation = (*plugin)->getObservation();
    if(observation){
      //@ aggregate observation for perception
      (*aggregate_observation) += (*observation);
      //@ A genuine acquisition stamp is valid even when the cloud contains
      //@ zero obstacle points. An empty static observation with stamp 0 is
      //@ ignored instead.
      const std::uint64_t observation_stamp_us =
        static_cast<std::uint64_t>(observation->header.stamp);
      if(observation_stamp_us != 0){
        if(!has_source_stamp || observation_stamp_us < source_stamp_us){
          source_stamp_us = observation_stamp_us;
          has_source_stamp = true;
        }
      }else if(!observation->points.empty()){
        //@ A non-empty contribution without acquisition time cannot be fresh,
        //@ even when another source has a valid stamp.
        stale = true;
      }
    }

    auto gbl_utils = (*plugin)->getGlobalUtils();
    if(frame_id.empty() && gbl_utils){
      frame_id = gbl_utils->getGblFrame();
    }
  }

  //@ No acquisition time means there is nothing that can make the cache fresh.
  if(!has_source_stamp){
    stale = true;
  }

  aggregate_observation->header.frame_id = frame_id;
  aggregate_observation->header.stamp = has_source_stamp ? source_stamp_us : 0;

  //@ Replace the cache every round; stale metadata must not preserve an old
  //@ aggregate and make it look like a fresh acquisition.
  shared_data_->aggregate_observation_ = aggregate_observation;
  if(aggregate_observation->points.size() >= 5){
    aggregate_observation_kdtree->setInputCloud(aggregate_observation);
  }
  shared_data_->aggregate_observation_kdtree_ = aggregate_observation_kdtree;
  shared_data_->aggregate_observation_stale_ = stale;

  {
    const auto finished_at = std::chrono::steady_clock::now();
    const double total_ms = std::chrono::duration<double, std::milli>(finished_at - refresh_started_at).count();
    const std::size_t cached_points = shared_data_->aggregate_observation_ ?
      shared_data_->aggregate_observation_->points.size() : 0U;
    if(stale){
      RCLCPP_INFO_THROTTLE(logger_->get_logger(), refresh_timing_clock, 2000,
        "refreshCache stale=%d source_stamp_us=%llu cached_points=%zu",
        stale ? 1 : 0,
        static_cast<unsigned long long>(source_stamp_us),
        cached_points);
    }
    if(total_ms > 2.0){
      RCLCPP_INFO_THROTTLE(logger_->get_logger(), refresh_timing_clock, 2000,
        "writer-timing refreshCache stale=%d source_stamp_us=%llu total=%.3fms access_wait=%.3fms ground_wait=%.3fms held=%.3fms points=%zu",
        stale ? 1 : 0,
        static_cast<unsigned long long>(source_stamp_us),
        total_ms,
        std::chrono::duration<double, std::milli>(refresh_access_acquired_at - refresh_started_at).count(),
        std::chrono::duration<double, std::milli>(refresh_ground_acquired_at - refresh_ground_before).count(),
        std::chrono::duration<double, std::milli>(finished_at - refresh_ground_acquired_at).count(),
        cached_points);
    }
  }
}

void StackedPerception::aggregateLethal(){
  
  //@ before calling this function, make sure we mutex lock, i.e.
  if(!shared_data_){
    return;
  }

  std::unique_lock<mutex_t> lock(*access_);
  std::unique_lock<std::recursive_mutex> sensor_lock(shared_data_->ground_kdtree_cb_mutex_);

  shared_data_->aggregate_lethal_.reset(new pcl::PointCloud<pcl::PointXYZI>);

  for (std::vector<std::shared_ptr<Sensor> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin)
  {
    if(!(*plugin)){
      continue;
    }

    //@aggregate observation for perception
    auto lethal = (*plugin)->getLethal();
    if(lethal){
      (*shared_data_->aggregate_lethal_) += (*lethal);
    }

    auto gbl_utils = (*plugin)->getGlobalUtils();
    if(gbl_utils){
      shared_data_->aggregate_lethal_->header.frame_id = gbl_utils->getGblFrame();
    }
  }

}

std::vector<perception_3d::PerceptionOpinion> StackedPerception::getOpinions(){
  
  std::vector<perception_3d::PerceptionOpinion> tmp_vector;
  for (std::vector<std::shared_ptr<Sensor> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin)
  {
    tmp_vector.push_back((*plugin)->getOpinion());
  }
  return tmp_vector;
}

bool StackedPerception::isSensorOK(){

  bool result = true; //@ it is safe to initialize true here, since current_ is initialized as false in sensor.h
  for (std::vector<std::shared_ptr<Sensor> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin)
  {
    result = result && (*plugin)->isCurrent();
  }
  return result;
}

void StackedPerception::configureSnapshotExport(
  bool enabled, std::string static_source_name)
{
  //@ Configuration happens before the update loop starts. The lock keeps the
  //@ static-source classification consistent with a concurrent reader.
  std::unique_lock<mutex_t> lock(*access_);
  snapshot_export_enabled_.store(enabled, std::memory_order_release);
  if(!static_source_name.empty()){
    snapshot_static_source_name_ = std::move(static_source_name);
  }
  if(!enabled){
    //@ Disabling the compatibility export removes the last ready snapshot so a
    //@ consumer cannot keep using data that is no longer refreshed.
    std::shared_ptr<const PerceptionSnapshot> empty_snapshot;
    std::atomic_store_explicit(
      &latest_snapshot_, empty_snapshot, std::memory_order_release);
  }
}

bool StackedPerception::snapshotExportEnabled() const noexcept
{
  return snapshot_export_enabled_.load(std::memory_order_acquire);
}

std::shared_ptr<const PerceptionSnapshot> StackedPerception::latestSnapshot()
const noexcept
{
  return std::atomic_load_explicit(&latest_snapshot_, std::memory_order_acquire);
}

std::shared_ptr<const PerceptionSnapshot> StackedPerception::exportSnapshot(
  const std::string & frame_id,
  std::int64_t cycle_stamp_ns,
  std::int64_t received_steady_ns,
  std::uint64_t terrain_revision,
  bool terrain_revision_known)
{
  //@ Old nodes never call this path: the compiled-out-at-runtime early return
  //@ keeps the default behavior and cost identical when the parameter is OFF.
  if(!snapshotExportEnabled()){
    return latestSnapshot();
  }

  //@ The two locks are the ones the clear/mark writers already use. They are
  //@ held only while current observations are deep-copied; no terrain building,
  //@ graph scan, distance-field update or search runs inside this critical
  //@ section. The copy is bounded by the current sensor observation sizes.
  std::unique_lock<mutex_t> lock(*access_);
  std::lock_guard<std::recursive_mutex> sensor_lock(
    shared_data_->ground_kdtree_cb_mutex_);

  const bool sensors_current = isSensorOK();
  std::string resolved_frame = frame_id;
  std::vector<PerceptionSourceObservation> sources;
  std::size_t dropped_nonfinite_sources = 0;
  std::size_t dropped_frame_sources = 0;
  //@ Newest genuine per-source acquisition stamp. A missing PCL header stamp is
  //@ exported as unknown zero, never replaced by the loop/cycle clock.
  std::int64_t newest_source_stamp_ns = 0;
  //@ One export gets one sequence number. Every source carries that same
  //@ sequence so a consumer can group contributions without another lookup.
  const std::uint64_t snapshot_sequence = snapshot_sequence_ + 1;

  for(std::vector<std::shared_ptr<Sensor> >::iterator plugin = plugins_.begin();
    plugin != plugins_.end(); ++plugin)
  {
    if(!(*plugin)){
      continue;
    }

    //@ Fill the frame from the plugin if the caller did not provide one. The
    //@ first non-empty global frame wins; disagreement is reported by the
    //@ downstream consumer because the point list is not re-projected here.
    auto gbl_utils = (*plugin)->getGlobalUtils();
    if(resolved_frame.empty() && gbl_utils){
      resolved_frame = gbl_utils->getGblFrame();
    }

    std::string source_id = (*plugin)->getName();
    if(source_id.empty()){
      source_id = "unnamed_plugin";
    }
    //@ getObservation() returns the plugin's mutable cache. Copy every finite
    //@ point immediately and never retain the PCL pointer in the snapshot.
    //@ StaticLayer contributes through the legacy shared map/dGraph path, not
    //@ through sensor_current_observation_. This compatibility export therefore
    //@ keeps the legacy semantics: an empty per-plugin observation is not
    //@ synthesized from shared_data_->pcl_map_. The latched map contains ground
    //@ points as well, so exporting it as occupied would turn every ground cell
    //@ into a static obstacle for wheel support. A map-only producer must supply
    //@ a real non-ground occupancy observation (or the offline static-map path
    //@ must be used) instead of relabeling the shared map here.
    auto observation = (*plugin)->getObservation();
    if(!observation || observation->points.empty()){
      //@ An empty plugin contribution is skipped, never exported as a clear.
      continue;
    }

    //@ PCL header stamps are microseconds. Only a genuine non-zero acquisition
    //@ stamp becomes provenance; otherwise the source stamp stays zero/unknown.
    std::int64_t source_stamp_ns = 0;
    const std::uint64_t header_stamp_us =
      static_cast<std::uint64_t>(observation->header.stamp);
    if(header_stamp_us > 0 &&
      header_stamp_us <= static_cast<std::uint64_t>(
        std::numeric_limits<std::int64_t>::max() / 1000))
    {
      source_stamp_ns = static_cast<std::int64_t>(header_stamp_us) * 1000;
    }
    //@ Never relabel a cloud whose own header names another frame. Empty header
    //@ frame is treated as the verified plugin map frame only when the plugin
    //@ transformed the points into that configured global frame.
    const std::string observation_frame = observation->header.frame_id;
    if(!observation_frame.empty() && !resolved_frame.empty() &&
      observation_frame != resolved_frame)
    {
      ++dropped_frame_sources;
      continue;
    }
    newest_source_stamp_ns =
      std::max(newest_source_stamp_ns, source_stamp_ns);

    std::vector<PerceptionPoint> occupied;
    occupied.reserve(observation->points.size());
    bool all_finite = true;
    for(const pcl::PointXYZI & point : observation->points){
      if(!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z))
      {
        all_finite = false;
        break;
      }
      occupied.push_back(PerceptionPoint{
        static_cast<double>(point.x),
        static_cast<double>(point.y),
        static_cast<double>(point.z)});
    }
    if(!all_finite){
      //@ A partially valid cloud is unknown, not a smaller complete
      //@ contribution. Drop the whole source for this export.
      ++dropped_nonfinite_sources;
      continue;
    }
    if(occupied.empty()){
      continue;
    }

    const PerceptionSourceLayer layer =
      (source_id == snapshot_static_source_name_) ?
      PerceptionSourceLayer::kStatic : PerceptionSourceLayer::kDynamic;

    //@ The current plugin observation is a complete replacement for this
    //@ source. Clearing points and vertical free-extent evidence are not
    //@ exposed by the existing Sensor interface, so they stay explicitly empty
    //@ and unknown rather than being invented.
    sources.emplace_back(
      source_id, resolved_frame, snapshot_sequence, terrain_revision,
      terrain_revision_known, source_stamp_ns, received_steady_ns, layer,
      PerceptionUpdateKind::kCompleteReplacement, true, std::move(occupied),
      std::vector<PerceptionPoint>(), std::vector<PerceptionObservedVolume>(),
      PerceptionObservedVolume());
  }

  snapshot_sequence_ = snapshot_sequence;

  std::string detail;
  if(!sensors_current){
    detail = "one or more sensors are expired or missing";
  }else if(sources.empty()){
    detail = "no finite occupancy points were available";
  }else if(dropped_nonfinite_sources > 0){
    detail = "one or more source clouds contained non-finite points";
  }else if(dropped_frame_sources > 0){
    detail = "one or more source clouds named a frame different from the "
      "verified map frame";
  }
  const bool complete = sensors_current && !sources.empty() &&
    dropped_nonfinite_sources == 0 && dropped_frame_sources == 0;

  auto snapshot = std::make_shared<PerceptionSnapshot>(
    std::move(resolved_frame), snapshot_sequence, terrain_revision,
    terrain_revision_known, newest_source_stamp_ns, cycle_stamp_ns,
    received_steady_ns, sensors_current, complete, std::move(sources),
    std::move(detail));

  //@ Publish the frozen object with an atomic shared_ptr store. Consumers only
  //@ perform an atomic load; they never take the perception lock to read it.
  std::shared_ptr<const PerceptionSnapshot> const_snapshot = snapshot;
  std::atomic_store_explicit(
    &latest_snapshot_, const_snapshot, std::memory_order_release);
  return const_snapshot;
}

}//end of name space
