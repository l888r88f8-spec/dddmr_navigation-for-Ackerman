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
#ifndef PERCEPTION_3D_SENSOR_H_
#define PERCEPTION_3D_SENSOR_H_

#include "pluginlib/class_list_macros.hpp"

/*Global setting from top level*/
#include <perception_3d/global_utils.h>
#include <perception_3d/shared_data.h>

/*Debug*/
#include <chrono>
#include <cstdint>
#include <string>

namespace perception_3d
{

enum PerceptionOpinion {
  PASS,
  PATH_BLOCKED_WAIT,
  PATH_BLOCKED_REPLANNING
};

// Additive producer-owned export state for the frozen perception snapshot.
//
// This is deliberately separate from getObservation(): legacy plugins keep
// their existing local-planner cache semantics, while a producer that can
// freeze explicit occupied/cleared evidence and acquisition provenance
// overrides the export hooks below. Defaults describe an unsupported plugin, so
// static/map plugins are never relabelled from the shared map.
struct SensorExportObservation
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr occupied;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cleared;
  std::string frame_id;
  std::int64_t sensor_stamp_ns{0};
  std::int64_t received_steady_ns{0};
  std::uint64_t source_sequence{0};
  bool complete{false};
};

class Sensor{

  public:
    Sensor();

    void initialize(std::string name,
        const rclcpp::Node::WeakPtr& weak_node,
        const std::shared_ptr<perception_3d::GlobalUtils>& gbl_utils);

    void setSharedData(std::shared_ptr<perception_3d::SharedData> shared_data);

    virtual void selfClear(){}

    virtual void selfMark(){}

    virtual pcl::PointCloud<pcl::PointXYZI>::Ptr getObservation() = 0;
    virtual pcl::PointCloud<pcl::PointXYZI>::Ptr getLethal() = 0;

    virtual void resetdGraph(){}

    virtual double get_dGraphValue(const unsigned int index) = 0;

    virtual bool isCurrent() = 0;
    
    virtual void updateLethalPointCloud(){}

    perception_3d::PerceptionOpinion getOpinion(){return opinion_;}

    std::shared_ptr<GlobalUtils> getGlobalUtils(){return gbl_utils_;}
    
    std::string getName(){return name_;}
    
    unsigned long get_dGraphSize();

  protected:

    virtual void onInitialize() {}

    rclcpp::Node::SharedPtr node_;

    bool current_;
    double expected_sensor_time_;
    std::string name_;
    std::shared_ptr<perception_3d::GlobalUtils> gbl_utils_;
    std::shared_ptr<perception_3d::SharedData> shared_data_;
   
    //@ For TF affine3d
    geometry_msgs::msg::TransformStamped trans_b2s_, trans_gbl2b_;

    //@ Graph, basically like the costmap_2d char
    DynamicGraph dGraph_;

    //@ current observation for local planner
    pcl::PointCloud<pcl::PointXYZI>::Ptr sensor_current_observation_;

    //@ opinion of plugin, default is pass
    PerceptionOpinion opinion_;
    
    //@ lethal point cloud, that will be merged together for line-of-sight check
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_lethal_;

  public:

    // Append export hooks after existing virtual methods to preserve their
    // vtable positions for already-built downstream consumers.
    // True when this plugin provides producer-owned snapshot evidence. A
    // supported plugin must also implement getSnapshotObservation().
    virtual bool supportsSnapshotObservation() const {return false;}

    // Fills frozen occupied/cleared evidence plus producer-owned provenance.
    // Returns false when no complete, fresh clear/mark cycle is available; the
    // caller must then omit this source rather than inventing evidence.
    virtual bool getSnapshotObservation(SensorExportObservation * observation)
    {
      (void)observation;
      return false;
    }

    // Append further producer metadata hooks after the existing virtual slots.
    // A stable COMPLETED-STATE revision changes only when this source's
    // completed clear/mark/reset state changes. Merely accepting a new input
    // cloud must not advance it; repeated polls of the same completed scan keep
    // it. It is an equality token used to key the cost cache, not an ordering
    // sequence. 0 means "not reported" and makes the cost cache fall back
    // conservatively.
    virtual std::uint64_t snapshotStateRevision() const {return 0;}

    // Acquisition stamp of that completed state in the producer clock domain.
    // 0 means unknown; the loop/cycle clock must never be substituted.
    virtual std::int64_t snapshotSensorStampNs() const {return 0;}

};


}//end of name space
#endif