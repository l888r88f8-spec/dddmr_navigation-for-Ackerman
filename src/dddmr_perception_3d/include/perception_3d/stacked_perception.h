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
#ifndef PERCEPTION_3D_STACKED_PERCEPTION_H_
#define PERCEPTION_3D_STACKED_PERCEPTION_H_

#include <perception_3d/perception_snapshot.h>
#include <perception_3d/scene_snapshot.h>
#include <perception_3d/sensor.h>

/*Debug*/
#include <sys/time.h>
#include <time.h>

/*Compatibility snapshot export cross-thread state.*/
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

/*To iterate the ymal for plugins and also for mutex lock*/
#include <pluginlib/class_loader.hpp>
# include <geometry_msgs/msg/pose_stamped.hpp>

namespace perception_3d
{

class StackedPerception{

  public:

    StackedPerception(const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& m_logger);
    ~StackedPerception();

    /*Plugin operations*/
    void addPlugin(std::shared_ptr<Sensor> plugin);
    std::vector<std::shared_ptr<Sensor> >* getPlugins();

    /*Collected all marked pcl*/
    void doClear_then_Mark();
    
    /*Reset the sensors*/
    void resetdGraph();

    /*Get value from d graph. Loop all plugins and return minimum one (closest to obstacles)*/
    double get_min_dGraphValue(const unsigned int index);

    /*Get value from d graph. Loop all plugins and return minimum size of dGraph one (closest to obstacles)*/
    unsigned long getdGraphSize();

    /*Aggregate observation pointers for local planner*/
    void aggregateObservations();

    /*Refresh the cached aggregate observation in perception update thread*/
    void refreshObservationCache();

    /*Aggregate lethal for line-of-sight check in global planner*/
    void aggregateLethal();

    /*Collect all opinions and return as a vector*/
    std::vector<perception_3d::PerceptionOpinion> getOpinions();
    
    std::shared_ptr<perception_3d::SharedData> getSharedDataPtr(){return shared_data_;}

    // Provide a typedef to ease future code maintenance
    typedef std::recursive_mutex mutex_t;
    mutex_t* getMutex()
    {
      return access_;
    }

    /*Check sensors are publish topics*/
    bool isSensorOK();

    /*
    Compatibility-only immutable observation export.

    configureSnapshotExport() is called once during node initialization from
    parameters whose default is OFF, so an old node keeps its original path.
    exportSnapshot() must be called only AFTER the complete clear/mark cycle; it
    briefly takes the existing locks to deep-copy current plugin observations
    into vectors, then publishes the frozen object. It performs no terrain
    building, graph scan, distance-field work or plugin callback.
    */
    void configureSnapshotExport(bool enabled, std::string static_source_name);
    bool snapshotExportEnabled() const noexcept;
    // Freezes current plugin evidence. A producer that implements the additive
    // Sensor snapshot hooks supplies occupied/cleared points plus its own
    // acquisition stamp, sequence and receipt; repeated polls of the same
    // completed cycle therefore reuse that provenance. Legacy plugins keep the
    // PCL-header path and never synthesize clearing. cycle_stamp_ns is the
    // perception update-cycle stamp and is kept separate from each source's
    // acquisition stamp. The configured map frame is used only for observations
    // whose points were already transformed into it; a source frame that
    // disagrees is dropped rather than relabelled.
    std::shared_ptr<const PerceptionSnapshot> exportSnapshot(
      const std::string & frame_id,
      std::int64_t cycle_stamp_ns,
      std::int64_t received_steady_ns,
      std::uint64_t terrain_revision,
      bool terrain_revision_known);
    std::shared_ptr<const PerceptionSnapshot> latestSnapshot() const noexcept;

    /*
    Unified immutable scene producer.

    configureSceneProducer() selects evidence and/or legacy-cost preparation.
    All-default (enabled=false) keeps old behavior. captureSceneSnapshot() takes
    the existing producer locks after a complete clear/mark/reset section, then
    freezes the requested optional views. It does not wait for a new scan. Static
    views are built once per accepted source-map identity; the legacy cost view
    is built on demand and cached by (static identity, completed source state).
    Requesting only include_legacy_static reuses that static view without
    evaluating any dynamic dGraph value or freezing lethal points. A capture
    frame that disagrees with the accepted source-map frame returns an
    incomplete scene and never replaces the last good latestSceneSnapshot().
    observationRevision() is an opaque equality token over completed state, not
    a monotonic sequence; source_sequence carries ordering.
    */
    void configureSceneProducer(const SceneProducerConfig & config);
    SceneProducerConfig sceneProducerConfig() const;
    std::shared_ptr<const SceneSnapshot> captureSceneSnapshot(
      const SceneSnapshotRequest & request,
      const std::string & frame_id,
      std::int64_t cycle_stamp_ns,
      std::int64_t received_steady_ns,
      std::uint64_t terrain_revision,
      bool terrain_revision_known);
    std::shared_ptr<const SceneSnapshot> latestSceneSnapshot() const noexcept;

  private:
    // Helpers below require access_ and ground_kdtree_cb_mutex_ to be held.
    std::shared_ptr<const PerceptionSnapshot> freezeEvidenceLocked(
      const std::string & frame_id,
      std::int64_t cycle_stamp_ns,
      std::int64_t received_steady_ns,
      std::uint64_t terrain_revision,
      bool terrain_revision_known);
    SourceMapIdentity currentSourceMapIdentityLocked(
      const std::string & fallback_frame) const;
    std::shared_ptr<const LegacyStaticView> ensureLegacyStaticViewLocked(
      const SourceMapIdentity & identity);
    std::shared_ptr<const LegacyCostView> buildLegacyCostViewLocked(
      const SourceMapIdentity & identity,
      const std::shared_ptr<const LegacyStaticView> & static_view);
    std::uint64_t combinedObservationRevisionLocked(bool * unversioned) const;
    std::int64_t newestSensorStampLocked() const;
    std::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>>
    freezeLethalLocked(const std::string & frame_id) const;

    std::vector<std::shared_ptr<Sensor>> plugins_;

    /*mutex*/
    mutex_t* access_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_prune_plan_;

    std::shared_ptr<SharedData> shared_data_;

    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_;

    /*Compatibility export state. latest_snapshot_ is read with std::atomic_load
    so consumers never take the perception lock just to inspect the snapshot.*/
    std::atomic<bool> snapshot_export_enabled_{false};
    std::string snapshot_static_source_name_{"map"};
    std::uint64_t snapshot_sequence_{0};
    std::shared_ptr<const PerceptionSnapshot> latest_snapshot_;

    //@ Unified scene producer state. Cache objects are immutable and are only
    //@ replaced under access_/ground_kdtree_cb_mutex_; consumers load the latest
    //@ envelope atomically.
    SceneProducerConfig scene_config_;
    std::uint64_t producer_loop_revision_{0};
    std::shared_ptr<const LegacyStaticView> cached_static_view_;
    SourceMapIdentity cached_static_identity_;
    std::shared_ptr<const LegacyCostView> cached_cost_view_;
    std::uint64_t cached_cost_static_revision_{0};
    std::uint64_t cached_cost_content_hash_{0};
    std::uint64_t cached_cost_observation_revision_{0};
    std::string cached_cost_frame_id_;
    std::shared_ptr<const SceneSnapshot> latest_scene_snapshot_;

    //@ Reuse one complete frozen evidence object for repeated polls of the same
    //@ completed producer state. This is the same object published to
    //@ latest_snapshot_; a repeated capture must not re-read a mutable plugin
    //@ export buffer or advance the source receipt. Appended after the existing
    //@ members so earlier offsets stay unchanged.
    std::shared_ptr<const PerceptionSnapshot> cached_evidence_snapshot_;
    std::uint64_t cached_evidence_revision_{0};
    std::string cached_evidence_frame_id_;
    std::uint64_t cached_evidence_terrain_revision_{0};
    bool cached_evidence_terrain_revision_known_{false};
};

}//end of name space

#endif
