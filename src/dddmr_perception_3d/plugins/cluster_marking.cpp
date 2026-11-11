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
#include <perception_3d/cluster_marking.h>

#include <cmath>
#include <limits>


namespace perception_3d
{

namespace
{
//@ Per-thread dense scratch indexed by ground-point id. Writing an array slot is
//@ much cheaper than hash-inserting the same node once per obstacle point, and
//@ the materialised map keeps exactly the same min-distance values. Each calling
//@ thread has its own scratch, and touched slots are reset after each computation.
struct NodeDistanceScratch
{
  std::vector<float> distance;
  std::vector<int> touched;
};

NodeDistanceScratch & node_distance_scratch()
{
  static thread_local NodeDistanceScratch scratch;
  return scratch;
}
}// namespace

Marking::~Marking(){
  //@ loop marking_ map to reset all ptr
  for(auto ix=marking_.begin(); ix!=marking_.end();ix++){
    for(auto iy=(*ix).second.begin(); iy!=(*ix).second.end();iy++){
      for(auto iz=(*iy).second.begin(); iz!=(*iy).second.end();iz++){
        (*iz).second.pc_.reset();
        (*iz).second.mc_.reset();
      } 
    }
  }
}

void Marking::computeMinDistanceFromObstacle2GroundNodes(  
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& pcptr, 
  const pcl::ModelCoefficients::Ptr& pcplaneptr,
  std::unordered_map<int, float>& nodes_of_min_distance){

  if(!pcptr || pcptr->points.empty() || !pcplaneptr ||
    pcplaneptr->values.size() < 4 || !shared_data_ ||
    !shared_data_->kdtree_ground_ || !shared_data_->pcl_ground_)
  {
    return;
  }
  for(size_t i=0; i<4; ++i){
    if(!std::isfinite(pcplaneptr->values[i])){
      return;
    }
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr projected_cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::ProjectInliers<pcl::PointXYZI> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (pcptr);
  proj.setModelCoefficients (pcplaneptr);
  proj.filter (*projected_cloud_cluster);

  pcl::VoxelGrid<pcl::PointXYZI> sor;
  sor.setInputCloud (projected_cloud_cluster);
  sor.setLeafSize (0.1f, 0.1f, 0.1f);
  sor.filter (*projected_cloud_cluster);

  NodeDistanceScratch & scratch = node_distance_scratch();
  const std::size_t ground_size = shared_data_->pcl_ground_->points.size();
  if(scratch.distance.size() != ground_size){
    scratch.distance.assign(ground_size, std::numeric_limits<float>::infinity());
    scratch.touched.clear();
  }

  std::vector<int> id_tmp;
  std::vector<float> sqdist_tmp;
  for(auto prj_pt_it=projected_cloud_cluster->points.begin();prj_pt_it!=projected_cloud_cluster->points.end();prj_pt_it++){
    if(!std::isfinite((*prj_pt_it).x) ||
      !std::isfinite((*prj_pt_it).y) ||
      !std::isfinite((*prj_pt_it).z))
    {
      continue;
    }
    pcl::PointXYZI pt;
    pt.x = (*prj_pt_it).x;
    pt.y = (*prj_pt_it).y;
    pt.z = (*prj_pt_it).z;

    id_tmp.clear();
    sqdist_tmp.clear();
    //@ We mark lethal
    if(shared_data_->kdtree_ground_->radiusSearch(pt, inflation_radius_, id_tmp, sqdist_tmp)){
      for(int i=0;i<id_tmp.size();i++){
        const int node_id = id_tmp[i];
        if(node_id < 0 ||
          static_cast<size_t>(node_id) >= ground_size)
        {
          continue;
        }
        float dx = pt.x - shared_data_->pcl_ground_->points[node_id].x;
        float dy = pt.y - shared_data_->pcl_ground_->points[node_id].y;
        //@ Remove z value (see issue 8), we might have to review this assumption.
        float distance = sqrt(dx*dx + dy* dy);
        float & slot = scratch.distance[node_id];
        if(std::isinf(slot)){
          scratch.touched.push_back(node_id);
          slot = distance;
        }
        else if(distance < slot){
          slot = distance;
        }
      }
    }
  }

  if(!scratch.touched.empty()){
    nodes_of_min_distance.reserve(nodes_of_min_distance.size() + scratch.touched.size());
    for(int node_id : scratch.touched){
      const float distance = scratch.distance[node_id];
      auto found = nodes_of_min_distance.find(node_id);
      if(found == nodes_of_min_distance.end()){
        nodes_of_min_distance.emplace(node_id, distance);
      }
      else if(distance < found->second){
        found->second = distance;
      }
      scratch.distance[node_id] = std::numeric_limits<float>::infinity();
    }
    scratch.touched.clear();
  }

}


void Marking::addPCPtr(const double cx, const double cy, const double cz, 
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& pcptr, 
  const pcl::ModelCoefficients::Ptr& pcplaneptr){

  if(!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(cz) ||
    xy_resolution_ <= 0.0 || height_resolution_ <= 0.0 ||
    !dGraph_ || !pcptr || pcptr->empty() ||
    !pcplaneptr || pcplaneptr->values.size() < 4)
  {
    return;
  }
  
  int x = cx/xy_resolution_;
  int y = cy/xy_resolution_;
  int z = cz/height_resolution_;
  if(marking_[x][y][z].pc_!= nullptr){
    marking_[x][y][z].pc_.reset();
    marking_[x][y][z].mc_.reset();
  }
  marking_[x][y][z].pc_ = pcptr;
  marking_[x][y][z].mc_ = pcplaneptr;
  std::unordered_map<int, float> nodes_of_min_distance;
  computeMinDistanceFromObstacle2GroundNodes(pcptr, pcplaneptr, nodes_of_min_distance);
  marking_[x][y][z].nodes_of_min_distance_ = nodes_of_min_distance;
  
  for(auto id=nodes_of_min_distance.begin();id!=nodes_of_min_distance.end();id++){
    //RCLCPP_INFO(rclcpp::get_logger("ClusterMarking"), "%.2f", (*id).second);
    dGraph_->setValue((*id).first, (*id).second);
    if((*id).second<=inscribed_radius_)
      lethal_map_[(*id).first] = (*id).second;
  }
  
}

void Marking::removePCPtr(perception_3d::per_marking& per_marking){
  
  if(!dGraph_){
    per_marking.pc_.reset();
    per_marking.mc_.reset();
    return;
  }

  auto nodes_of_min_distance = per_marking.nodes_of_min_distance_;

  for(auto id=nodes_of_min_distance.begin();id!=nodes_of_min_distance.end();id++){
    dGraph_->clearValue((*id).first, 9999.0);
    if((*id).second<=inscribed_radius_)
      lethal_map_.erase((*id).first);
  }

  per_marking.pc_.reset();
  per_marking.mc_.reset();

}

void Marking::updateCleared(const std::vector<marking_voxel>& current_observation_ptr){
  /*
  for(auto it=current_observation_ptr.begin();it!=current_observation_ptr.end();it++){
    auto pcptr = marking_[(*it).x][(*it).y][(*it).z].first;
    auto pcplaneptr = marking_[(*it).x][(*it).y][(*it).z].second;

    //@Filter out empty cloud
    if(pcptr->points.empty())
      continue;

    std::unordered_map<int, float> nodes_of_min_distance;
    computeMinDistanceFromObstacle2GroundNodes(pcptr, pcplaneptr, nodes_of_min_distance);
    for(auto id=nodes_of_min_distance.begin();id!=nodes_of_min_distance.end();id++){
      dGraph_->setValue((*id).first,(*id).second);
    }
  }
  */
}

}//end of name space
