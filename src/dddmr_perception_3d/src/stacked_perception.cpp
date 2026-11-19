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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

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

  //@ With the scene producer enabled, hold the shared recursive sensor lock
  //@ across the WHOLE clear/mark/lethal cycle. Input callbacks take the same
  //@ lock, so they cannot replace the accepted scan or reset pending clear
  //@ records between an individual selfClear() and selfMark(). Keep the
  //@ established lock order access_ -> sensor. Producer OFF keeps the original
  //@ per-plugin short locks and adds no long-held lock.
  const bool producer_enabled = shared_data_ &&
    shared_data_->scene_producer_enabled_.load(std::memory_order_acquire);
  std::unique_lock<std::recursive_mutex> sensor_lock;
  if(producer_enabled){
    sensor_lock = std::unique_lock<std::recursive_mutex>(
      shared_data_->ground_kdtree_cb_mutex_);
  }

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
  //@ Completed producer section generation. It advances only after all plugins
  //@ finished clear/mark/lethal for this loop; on-demand captures cannot see a
  //@ half-applied section while they hold the same producer locks.
  ++producer_loop_revision_;

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
  //@ Old nodes never call this path: the runtime early return keeps the default
  //@ behavior and cost identical when the producer is OFF.
  if(!snapshotExportEnabled()){
    return latestSnapshot();
  }

  std::unique_lock<mutex_t> lock(*access_);
  std::lock_guard<std::recursive_mutex> sensor_lock(
    shared_data_->ground_kdtree_cb_mutex_);
  auto evidence = freezeEvidenceLocked(
    frame_id, cycle_stamp_ns, received_steady_ns,
    terrain_revision, terrain_revision_known);

  //@ Ordinary ROS evidence flow publishes the unified scene here, from the
  //@ SAME frozen evidence object that compatibility latestSnapshot() exposes.
  //@ This does no second freeze, no static traversal, no dynamic dGraph scan and
  //@ no legacy-cost build; a Grid evidence-only deployment therefore gets a
  //@ root SceneSnapshot automatically after each completed export.
  if(evidence){
    std::string resolved_frame = frame_id;
    const SourceMapIdentity source_map =
      currentSourceMapIdentityLocked(resolved_frame);
    if(resolved_frame.empty()){
      resolved_frame = source_map.frame_id;
    }
    auto scene = std::make_shared<const SceneSnapshot>(
      resolved_frame, source_map,
      combinedObservationRevisionLocked(nullptr),
      newestSensorStampLocked(), cycle_stamp_ns, received_steady_ns,
      evidence->sensors_current(), evidence->complete(), evidence,
      std::shared_ptr<const LegacyStaticView>(),
      std::shared_ptr<const LegacyCostView>(), evidence->detail());
    std::atomic_store_explicit(
      &latest_scene_snapshot_, scene, std::memory_order_release);
  }
  return evidence;
}

void StackedPerception::configureSnapshotExport(
  bool enabled, std::string static_source_name)
{
  configureSceneProducer(SceneProducerConfig{
    enabled, enabled, false, std::move(static_source_name)});
}

void StackedPerception::configureSceneProducer(
  const SceneProducerConfig & config)
{
  //@ Configuration happens before the update loop starts. The lock keeps the
  //@ producer mode consistent with a concurrent capture and plugin readers.
  std::unique_lock<mutex_t> lock(*access_);
  std::string static_name = config.static_source_name;
  if(static_name.empty()){
    static_name = "map";
  }
  scene_config_ = config;
  scene_config_.static_source_name = static_name;
  snapshot_static_source_name_ = static_name;

  //@ A mode or static-source change invalidates the evidence reuse key.
  cached_evidence_snapshot_.reset();
  cached_evidence_revision_ = 0;
  cached_evidence_frame_id_.clear();
  cached_evidence_terrain_revision_ = 0;
  cached_evidence_terrain_revision_known_ = false;

  if(shared_data_){
    shared_data_->scene_producer_enabled_.store(
      config.enabled, std::memory_order_release);
    shared_data_->scene_evidence_enabled_.store(
      config.enabled && config.evidence, std::memory_order_release);
    shared_data_->scene_legacy_cost_enabled_.store(
      config.enabled && config.legacy_cost, std::memory_order_release);
  }
  snapshot_export_enabled_.store(
    config.enabled && config.evidence, std::memory_order_release);
  if(!config.enabled || !config.evidence){
    std::shared_ptr<const PerceptionSnapshot> empty_snapshot;
    std::atomic_store_explicit(
      &latest_snapshot_, empty_snapshot, std::memory_order_release);
  }
  if(!config.enabled || !config.legacy_cost){
    cached_cost_view_.reset();
    cached_cost_static_revision_ = 0;
    cached_cost_content_hash_ = 0;
    cached_cost_observation_revision_ = 0;
    cached_cost_frame_id_.clear();
  }
  if(!config.enabled){
    cached_static_view_.reset();
    cached_static_identity_ = SourceMapIdentity();
    std::shared_ptr<const SceneSnapshot> empty_scene;
    std::atomic_store_explicit(
      &latest_scene_snapshot_, empty_scene, std::memory_order_release);
  }
}

SceneProducerConfig StackedPerception::sceneProducerConfig() const
{
  std::unique_lock<mutex_t> lock(*access_);
  return scene_config_;
}

std::shared_ptr<const SceneSnapshot> StackedPerception::latestSceneSnapshot()
const noexcept
{
  return std::atomic_load_explicit(
    &latest_scene_snapshot_, std::memory_order_acquire);
}

SourceMapIdentity StackedPerception::currentSourceMapIdentityLocked(
  const std::string & fallback_frame) const
{
  SourceMapIdentity identity;
  if(!shared_data_){
    identity.frame_id = fallback_frame;
    return identity;
  }
  identity.frame_id = shared_data_->source_map_frame_id_.empty() ?
    fallback_frame : shared_data_->source_map_frame_id_;
  identity.revision = shared_data_->source_map_revision_;
  identity.content_hash = shared_data_->source_map_content_hash_;
  identity.ground_point_count = shared_data_->source_map_ground_count_;
  identity.map_point_count = shared_data_->source_map_map_count_;
  identity.known = shared_data_->source_map_identity_known_ &&
    identity.revision > 0;
  return identity;
}

std::shared_ptr<const LegacyStaticView>
StackedPerception::ensureLegacyStaticViewLocked(
  const SourceMapIdentity & identity)
{
  if(cached_static_view_ && cached_static_identity_.SameContent(identity)){
    return cached_static_view_;
  }
  if(!shared_data_ || !identity.known || !shared_data_->pcl_ground_ ||
    !shared_data_->pcl_map_ || shared_data_->pcl_ground_->empty() ||
    shared_data_->pcl_map_->empty() || !shared_data_->sGraph_ptr_)
  {
    return nullptr;
  }

  auto ground = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(
    *shared_data_->pcl_ground_);
  auto map = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(
    *shared_data_->pcl_map_);
  ground->header.frame_id = identity.frame_id;
  map->header.frame_id = identity.frame_id;

  auto ground_tree = std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZI>>();
  ground_tree->setInputCloud(ground);
  auto map_tree = std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZI>>();
  map_tree->setInputCloud(map);

  //@ Copy the original graph by index. The edge sets keep their original
  //@ ordering/precision, and the weight table keeps the old missing-value
  //@ default of 0.0. No StaticGraph shallow pointer alias is retained.
  const std::size_t ground_count = ground->points.size();
  std::vector<edges_t> edges(ground_count);
  graph_t * graph_ptr = shared_data_->sGraph_ptr_->getGraphPtr();
  if(graph_ptr){
    for(const auto & entry : *graph_ptr){
      if(entry.first < ground_count){
        edges[entry.first] = entry.second;
      }
    }
  }
  std::vector<float> weights(ground_count, 0.0F);
  for(std::size_t i = 0; i < ground_count; ++i){
    weights[i] = shared_data_->sGraph_ptr_->getNodeWeight(
      static_cast<unsigned int>(i));
  }
  LegacyStaticGraph graph(std::move(edges), std::move(weights));

  auto view = std::make_shared<const LegacyStaticView>(
    identity.frame_id, identity,
    std::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>>(ground),
    std::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>>(map),
    std::shared_ptr<const pcl::KdTreeFLANN<pcl::PointXYZI>>(ground_tree),
    std::shared_ptr<const pcl::KdTreeFLANN<pcl::PointXYZI>>(map_tree),
    std::move(graph));
  cached_static_view_ = view;
  cached_static_identity_ = identity;
  return view;
}

std::uint64_t StackedPerception::combinedObservationRevisionLocked(
  bool * unversioned) const
{
  if(unversioned){
    *unversioned = false;
  }
  bool has_dynamic = false;
  bool has_unversioned = false;
  std::uint64_t hash = 1469598103934665603ULL;
  const std::uint64_t prime = 1099511628211ULL;
  for(const auto & plugin : plugins_){
    if(!plugin || plugin->getName() == scene_config_.static_source_name){
      continue;
    }
    has_dynamic = true;
    const std::uint64_t revision = plugin->snapshotStateRevision();
    if(revision == 0){
      has_unversioned = true;
    }
    for(const char c : plugin->getName()){
      hash ^= static_cast<unsigned char>(c);
      hash *= prime;
    }
    hash ^= revision;
    hash *= prime;
  }
  if(!has_dynamic){
    return 0;
  }
  if(has_unversioned){
    if(unversioned){
      *unversioned = true;
    }
    //@ A producer without a stable state revision falls back to the completed
    //@ clear/mark loop generation. It may rebuild more often, but it never
    //@ returns a cached cost view for a newer state.
    return producer_loop_revision_ == 0 ? 1 : producer_loop_revision_;
  }
  return hash == 0 ? 1 : hash;
}

std::int64_t StackedPerception::newestSensorStampLocked() const
{
  std::int64_t stamp = 0;
  for(const auto & plugin : plugins_){
    if(!plugin || plugin->getName() == scene_config_.static_source_name){
      continue;
    }
    const std::int64_t source_stamp = plugin->snapshotSensorStampNs();
    if(source_stamp > stamp){
      stamp = source_stamp;
    }
  }
  return stamp;
}

std::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>>
StackedPerception::freezeLethalLocked(const std::string & frame_id) const
{
  auto lethal = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for(const auto & plugin : plugins_){
    if(!plugin){
      continue;
    }
    auto source_lethal = plugin->getLethal();
    if(!source_lethal){
      continue;
    }
    for(const pcl::PointXYZI & point : source_lethal->points){
      if(!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z))
      {
        continue;
      }
      lethal->push_back(point);
    }
  }
  lethal->header.frame_id = frame_id;
  return lethal;
}

std::shared_ptr<const LegacyCostView>
StackedPerception::buildLegacyCostViewLocked(
  const SourceMapIdentity & identity,
  const std::shared_ptr<const LegacyStaticView> & static_view)
{
  if(!identity.known || !static_view){
    return nullptr;
  }
  const std::size_t ground_count = static_view->groundPointCount();
  if(ground_count == 0){
    return nullptr;
  }

  bool unversioned = false;
  const std::uint64_t observation_revision =
    combinedObservationRevisionLocked(&unversioned);

  if(cached_cost_view_ && cached_cost_view_->complete() &&
    cached_cost_static_revision_ == identity.revision &&
    cached_cost_content_hash_ == identity.content_hash &&
    cached_cost_frame_id_ == identity.frame_id &&
    cached_cost_observation_revision_ == observation_revision)
  {
    return cached_cost_view_;
  }

  std::vector<double> min_dgraph(ground_count, 0.0);
  bool finite = true;
  for(std::size_t i = 0; i < ground_count; ++i){
    min_dgraph[i] = get_min_dGraphValue(static_cast<unsigned int>(i));
    if(!std::isfinite(min_dgraph[i])){
      finite = false;
      break;
    }
  }
  auto lethal = freezeLethalLocked(identity.frame_id);
  const bool sensors_current = isSensorOK();
  const bool complete = finite && lethal != nullptr && !identity.frame_id.empty();

  const std::int64_t received_steady_ns = std::chrono::duration_cast<
    std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  auto view = std::make_shared<const LegacyCostView>(
    identity.frame_id, identity, observation_revision,
    newestSensorStampLocked(), received_steady_ns, sensors_current, complete,
    static_view, std::move(min_dgraph), std::move(lethal));

  if(complete){
    cached_cost_view_ = view;
    cached_cost_static_revision_ = identity.revision;
    cached_cost_content_hash_ = identity.content_hash;
    cached_cost_observation_revision_ = observation_revision;
    cached_cost_frame_id_ = identity.frame_id;
  }
  return view;
}

std::shared_ptr<const SceneSnapshot>
StackedPerception::captureSceneSnapshot(
  const SceneSnapshotRequest & request,
  const std::string & frame_id,
  std::int64_t cycle_stamp_ns,
  std::int64_t received_steady_ns,
  std::uint64_t terrain_revision,
  bool terrain_revision_known)
{
  //@ Wait for the current complete clear/mark/reset section to finish, then
  //@ freeze one coherent view. Never waits for a new scan when a completed
  //@ valid state already exists. The producer mode is read only after taking
  //@ the same lock used by configureSceneProducer().
  std::unique_lock<mutex_t> lock(*access_);
  if(!scene_config_.enabled){
    return nullptr;
  }
  std::lock_guard<std::recursive_mutex> sensor_lock(
    shared_data_->ground_kdtree_cb_mutex_);

  if(received_steady_ns <= 0){
    received_steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  std::string resolved_frame = frame_id;
  if(resolved_frame.empty() && shared_data_){
    resolved_frame = shared_data_->source_map_frame_id_;
  }
  const SourceMapIdentity source_map =
    currentSourceMapIdentityLocked(resolved_frame);
  if(resolved_frame.empty()){
    resolved_frame = source_map.frame_id;
  }

  //@ Frame coherence is a hard gate. A caller-supplied frame that disagrees
  //@ with the accepted static source frame would relabel points if evidence or
  //@ cost were frozen against it. Return an incomplete diagnostic envelope and
  //@ deliberately keep the previous good latestSceneSnapshot().
  const bool frame_conflict = !frame_id.empty() && source_map.known &&
    !source_map.frame_id.empty() && frame_id != source_map.frame_id;
  if(frame_conflict){
    auto scene = std::make_shared<const SceneSnapshot>(
      source_map.frame_id, source_map,
      combinedObservationRevisionLocked(nullptr), newestSensorStampLocked(),
      cycle_stamp_ns, received_steady_ns, isSensorOK(), false,
      std::shared_ptr<const PerceptionSnapshot>(),
      std::shared_ptr<const LegacyStaticView>(),
      std::shared_ptr<const LegacyCostView>(),
      "capture frame '" + frame_id + "' does not match static source frame '" +
      source_map.frame_id + "'");
    return scene;
  }

  std::shared_ptr<const PerceptionSnapshot> evidence;
  bool evidence_complete = true;
  if(request.include_evidence){
    if(scene_config_.evidence){
      evidence = freezeEvidenceLocked(
        resolved_frame, cycle_stamp_ns, received_steady_ns,
        terrain_revision, terrain_revision_known);
      evidence_complete = evidence && evidence->complete() &&
        evidence->frame_id() == resolved_frame;
      if(!evidence_complete){
        evidence.reset();
      }
    }else{
      evidence_complete = false;
    }
  }

  std::shared_ptr<const LegacyStaticView> static_view;
  std::shared_ptr<const LegacyCostView> cost_view;
  bool static_complete = true;
  bool cost_complete = true;
  //@ Static-only acquisition reuses/builds the immutable static view exactly
  //@ like the cost path, but never calls get_min_dGraphValue() for N ground
  //@ cells and never freezes lethal points.
  if(request.include_legacy_static){
    if(source_map.known){
      static_view = ensureLegacyStaticViewLocked(source_map);
    }
    static_complete = static_view &&
      static_view->frame_id() == resolved_frame;
    if(!static_complete){
      static_view.reset();
    }
  }
  if(request.include_legacy_cost){
    if(scene_config_.legacy_cost && source_map.known){
      if(!static_view){
        static_view = ensureLegacyStaticViewLocked(source_map);
      }
      cost_view = buildLegacyCostViewLocked(source_map, static_view);
    }
    cost_complete = cost_view && cost_view->complete() &&
      cost_view->frame_id() == resolved_frame;
    if(!cost_complete){
      cost_view.reset();
    }
  }

  const bool requested_any =
    request.include_evidence || request.include_legacy_cost ||
    request.include_legacy_static;
  const bool sensors_current = isSensorOK();
  const bool complete = requested_any && evidence_complete && cost_complete &&
    static_complete;

  std::string detail;
  if(!requested_any){
    detail = "no scene view was requested";
  }else if(!complete){
    if(request.include_evidence && !scene_config_.evidence){
      detail = "evidence view requested but the producer evidence mode is off";
    }else if(request.include_legacy_cost && !scene_config_.legacy_cost){
      detail =
        "legacy cost view requested but the producer legacy-cost mode is off";
    }else if((request.include_legacy_cost ||
      request.include_legacy_static) && !source_map.known)
    {
      detail =
        "legacy static/cost view requested before a static source identity";
    }else if(request.include_legacy_static && !static_complete){
      detail = "legacy static view is incomplete";
    }else if(request.include_legacy_cost && !cost_complete){
      detail = "legacy cost view is incomplete";
    }else{
      detail = "evidence view is incomplete";
    }
  }

  std::int64_t sensor_stamp_ns = newestSensorStampLocked();
  if(sensor_stamp_ns == 0 && evidence){
    sensor_stamp_ns = evidence->sensor_stamp_ns();
  }
  std::uint64_t observation_revision =
    combinedObservationRevisionLocked(nullptr);

  auto scene = std::make_shared<const SceneSnapshot>(
    resolved_frame, source_map, observation_revision,
    sensor_stamp_ns, cycle_stamp_ns, received_steady_ns,
    sensors_current, complete, evidence, static_view, cost_view,
    std::move(detail));
  std::atomic_store_explicit(
    &latest_scene_snapshot_, scene, std::memory_order_release);
  return scene;
}

std::shared_ptr<const PerceptionSnapshot>
StackedPerception::freezeEvidenceLocked(
  const std::string & frame_id,
  std::int64_t cycle_stamp_ns,
  std::int64_t received_steady_ns,
  std::uint64_t terrain_revision,
  bool terrain_revision_known)
{
  //@ Reuse the already frozen complete object when nothing in the completed
  //@ producer state (or its bound terrain revision/frame) changed. This keeps
  //@ repeated polls of one scan from re-reading a mutable plugin export buffer
  //@ or advancing its source sequence/receipt.
  const std::uint64_t completed_observation_revision =
    combinedObservationRevisionLocked(nullptr);
  if(cached_evidence_snapshot_ && cached_evidence_snapshot_->complete() &&
    cached_evidence_revision_ == completed_observation_revision &&
    cached_evidence_frame_id_ == frame_id &&
    cached_evidence_terrain_revision_ == terrain_revision &&
    cached_evidence_terrain_revision_known_ == terrain_revision_known)
  {
    return cached_evidence_snapshot_;
  }

  const bool sensors_current = isSensorOK();
  std::string resolved_frame = frame_id;
  std::vector<PerceptionSourceObservation> sources;
  std::size_t dropped_nonfinite_sources = 0;
  std::size_t dropped_frame_sources = 0;
  std::size_t dropped_unready_sources = 0;
  std::size_t dropped_unprovenanced_sources = 0;
  std::size_t empty_sources = 0;
  //@ Newest genuine per-source acquisition stamp. A missing source stamp is
  //@ never replaced by the loop/cycle clock.
  std::int64_t newest_source_stamp_ns = 0;
  //@ Snapshot sequence groups one export. Producer-backed sources keep their
  //@ own sequence so a repeated exporter poll cannot replay old evidence as a
  //@ new source update.
  const std::uint64_t snapshot_sequence = snapshot_sequence_ + 1;

  for(std::vector<std::shared_ptr<Sensor> >::iterator plugin = plugins_.begin();
    plugin != plugins_.end(); ++plugin)
  {
    if(!(*plugin)){
      continue;
    }

    //@ Fill the frame from the plugin if the caller did not provide one. The
    //@ first non-empty global frame wins; disagreement is rejected below.
    auto gbl_utils = (*plugin)->getGlobalUtils();
    if(resolved_frame.empty() && gbl_utils){
      resolved_frame = gbl_utils->getGblFrame();
    }

    std::string source_id = (*plugin)->getName();
    if(source_id.empty()){
      source_id = "unnamed_plugin";
    }

    //@ Producer-backed plugins freeze their own occupied/cleared evidence and
    //@ provenance. Legacy plugins keep the original getObservation() path, so
    //@ static/map plugins are never relabelled from the shared map.
    const bool producer_backed =
      (*plugin)->supportsSnapshotObservation();
    pcl::PointCloud<pcl::PointXYZI>::Ptr occupied_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cleared_cloud;
    std::string observation_frame;
    std::int64_t source_stamp_ns = 0;
    std::int64_t source_received_ns = received_steady_ns;
    std::uint64_t source_sequence = snapshot_sequence;

    if(producer_backed){
      SensorExportObservation producer_observation;
      if(!(*plugin)->getSnapshotObservation(&producer_observation) ||
        !producer_observation.complete ||
        !producer_observation.occupied || !producer_observation.cleared)
      {
        //@ No complete fresh clear/mark cycle exists. Never fall back to a
        //@ stale mutable cache or synthesize a clear from an empty cloud.
        ++dropped_unready_sources;
        continue;
      }
      occupied_cloud = producer_observation.occupied;
      cleared_cloud = producer_observation.cleared;
      observation_frame = producer_observation.frame_id;
      if(producer_observation.sensor_stamp_ns <= 0 ||
        producer_observation.source_sequence == 0 ||
        producer_observation.received_steady_ns <= 0 ||
        observation_frame.empty())
      {
        ++dropped_unprovenanced_sources;
        continue;
      }
      source_stamp_ns = producer_observation.sensor_stamp_ns;
      source_received_ns = producer_observation.received_steady_ns;
      source_sequence = producer_observation.source_sequence;
    }else{
      //@ getObservation() returns the plugin's mutable cache. Copy every finite
      //@ point immediately and never retain the PCL pointer in the snapshot.
      auto observation = (*plugin)->getObservation();
      if(!observation || observation->points.empty()){
        //@ Legacy empty contribution is skipped, never exported as a clear.
        continue;
      }
      occupied_cloud = observation;
      cleared_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
      observation_frame = observation->header.frame_id;
      //@ PCL header stamps are microseconds. Only a genuine non-zero
      //@ acquisition stamp becomes provenance; otherwise it stays unknown.
      const std::uint64_t header_stamp_us =
        static_cast<std::uint64_t>(observation->header.stamp);
      if(header_stamp_us > 0 &&
        header_stamp_us <= static_cast<std::uint64_t>(
          std::numeric_limits<std::int64_t>::max() / 1000))
      {
        source_stamp_ns = static_cast<std::int64_t>(header_stamp_us) * 1000;
      }
    }

    //@ Never relabel a cloud whose own header names another frame. An empty
    //@ legacy header frame is treated as the verified plugin map frame only when
    //@ the plugin transformed the points into that configured global frame.
    if(!observation_frame.empty() && !resolved_frame.empty() &&
      observation_frame != resolved_frame)
    {
      ++dropped_frame_sources;
      continue;
    }
    newest_source_stamp_ns =
      std::max(newest_source_stamp_ns, source_stamp_ns);

    auto append_finite_points = [](
      const pcl::PointCloud<pcl::PointXYZI> & cloud,
      std::vector<PerceptionPoint> * points) -> bool
      {
        points->reserve(cloud.points.size());
        for(const pcl::PointXYZI & point : cloud.points){
          if(!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z))
          {
            return false;
          }
          points->push_back(PerceptionPoint{
            static_cast<double>(point.x),
            static_cast<double>(point.y),
            static_cast<double>(point.z)});
        }
        return true;
      };

    std::vector<PerceptionPoint> occupied;
    std::vector<PerceptionPoint> cleared;
    if(!append_finite_points(*occupied_cloud, &occupied) ||
      !append_finite_points(*cleared_cloud, &cleared))
    {
      //@ A partially valid cloud is unknown, not a smaller complete
      //@ contribution. Drop the whole source for this export.
      ++dropped_nonfinite_sources;
      continue;
    }
    if(occupied.empty() && cleared.empty()){
      //@ An empty snapshot with no explicit clearing evidence is unknown.
      ++empty_sources;
      continue;
    }

    const PerceptionSourceLayer layer =
      (source_id == snapshot_static_source_name_) ?
      PerceptionSourceLayer::kStatic : PerceptionSourceLayer::kDynamic;

    //@ Complete replacement: the producer's active persistent marking set is
    //@ the complete current contribution; cleared records are explicit removal
    //@ evidence. Vertical free-extent evidence is not inferred from either.
    sources.emplace_back(
      source_id, resolved_frame, source_sequence, terrain_revision,
      terrain_revision_known, source_stamp_ns, source_received_ns, layer,
      PerceptionUpdateKind::kCompleteReplacement, true, std::move(occupied),
      std::move(cleared), std::vector<PerceptionObservedVolume>(),
      PerceptionObservedVolume());
  }

  snapshot_sequence_ = snapshot_sequence;

  std::string detail;
  if(!sensors_current){
    detail = "one or more sensors are expired or missing";
  }else if(sources.empty()){
    detail = "no complete fresh source observation was available";
  }else if(dropped_unready_sources > 0){
    detail = "one or more producer sources had no complete fresh cycle";
  }else if(dropped_unprovenanced_sources > 0){
    detail = "one or more producer sources had no genuine acquisition "
      "stamp, sequence, receipt or frame";
  }else if(dropped_nonfinite_sources > 0){
    detail = "one or more source clouds contained non-finite points";
  }else if(dropped_frame_sources > 0){
    detail = "one or more source clouds named a frame different from the "
      "verified map frame";
  }else if(empty_sources > 0){
    detail = "one or more sources had no occupancy or explicit clear evidence";
  }
  const bool complete = sensors_current && !sources.empty() &&
    dropped_nonfinite_sources == 0 && dropped_frame_sources == 0 &&
    dropped_unready_sources == 0 && dropped_unprovenanced_sources == 0;

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
  if(complete){
    cached_evidence_snapshot_ = const_snapshot;
    cached_evidence_revision_ = completed_observation_revision;
    cached_evidence_frame_id_ = frame_id;
    cached_evidence_terrain_revision_ = terrain_revision;
    cached_evidence_terrain_revision_known_ = terrain_revision_known;
  }
  return const_snapshot;
}

}//end of name space
