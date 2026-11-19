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
#include <global_planner/a_star_on_pc.h>

#include <chrono>
#include <mutex>
#include <vector>

namespace
{
double ElapsedMs(const std::chrono::steady_clock::time_point & started_at)
{
  return std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started_at).count();
}
}  // namespace

AstarList::AstarList(pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_original_z_up){
  pc_original_z_up_ = pc_original_z_up;
  // Snapshot-only callers pass a null live-ground placeholder, so they do not
  // pay for an unused nanoflann index over an immutable scene cloud.
  if(pc_original_z_up_){
    kdtree_ground_.reset(new nanoflann::KdTreeFLANN<pcl::PointXYZI>());
    kdtree_ground_->setInputCloud(pc_original_z_up_);
  }
}

void AstarList::Initial(){
  Initial(pc_original_z_up_ ? pc_original_z_up_->points.size() : 0);
}

void AstarList::Initial(std::size_t node_count){
  as_list_.clear(); 
  for(unsigned int it=0; it!=node_count;it++){
    Node_t new_node = {.self_index=0, .g=0, .h=0, .f=0, .parent_index=0, .is_closed=false, .is_opened=false};
    as_list_[it] = new_node;
  }
  f_priority_set_.clear();
}

Node_t AstarList::getNode(unsigned int node_index){

  return as_list_[node_index];
}

float AstarList::getGVal(Node_t& a_node){
  return as_list_[a_node.self_index].g;
}

void AstarList::closeNode(Node_t& a_node){
  as_list_[a_node.self_index].is_closed = true;
}

void AstarList::updateNode(Node_t& a_node){
  as_list_[a_node.self_index] = a_node;
  f_p_ afp;
  afp.first = a_node.f; //made minimum f to be top so we can pop it
  afp.second = a_node.self_index;
  f_priority_set_.insert(afp);
  //ROS_DEBUG("Add node ---> %u with g: %f, h: %f, f: %f",a_node.self_index, a_node.g, a_node.h, a_node.f);
}

Node_t AstarList::getNode_wi_MinimumF(){
  while(!f_priority_set_.empty()){
    auto first_it = f_priority_set_.begin();
    Node_t m_node = as_list_[(*first_it).second];
    f_priority_set_.erase(first_it);
    if(!m_node.is_closed){
      return m_node;
    }
  }

  return Node_t{.self_index=0, .g=0, .h=0, .f=0, .parent_index=0, .is_closed=true, .is_opened=false};
}

bool AstarList::isClosed(unsigned int node_index){
  return as_list_[node_index].is_closed;
}

bool AstarList::isOpened(unsigned int node_index){
  return as_list_[node_index].is_opened;
}

bool AstarList::isFrontierEmpty(){
  return f_priority_set_.empty();
}

//@----------------------------------------------------------------------------------------

A_Star_on_Graph::A_Star_on_Graph(pcl::PointCloud<pcl::PointXYZI>::Ptr pc_original_z_up, 
                                  std::shared_ptr<perception_3d::Perception3D_ROS> perception_ros,
                                  double a_star_expanding_radius){
  
  perception_ros_ = perception_ros;
  pc_original_z_up_ = pc_original_z_up;
  a_star_expanding_radius_ = a_star_expanding_radius;
  turning_weight_ = 0.0;
  ResetSceneSnapshot();
  ASLS_ = new AstarList(pc_original_z_up_);
}

A_Star_on_Graph::~A_Star_on_Graph(){
  if(ASLS_)
    delete ASLS_;
}

void A_Star_on_Graph::updateGraph(pcl::PointCloud<pcl::PointXYZI>::Ptr pc_original_z_up){
  ResetSceneSnapshot();
  ASLS_->pc_original_z_up_ = pc_original_z_up;
  ASLS_->kdtree_ground_.reset(new nanoflann::KdTreeFLANN<pcl::PointXYZI>());
  ASLS_->kdtree_ground_->setInputCloud(pc_original_z_up_);
}

void A_Star_on_Graph::ResetSceneSnapshot(){
  scene_static_view_.reset();
  scene_cost_view_.reset();
  scene_lethal_kdtree_.reset();
  scene_parameters_ = global_planner::LegacySearchParameters{};
  scene_requested_ = false;
  scene_active_ = false;
  scene_rejection_reason_.clear();
}

void A_Star_on_Graph::clearSceneSnapshot(){
  ResetSceneSnapshot();
}

bool A_Star_on_Graph::setSceneSnapshot(
  const std::shared_ptr<const perception_3d::SceneSnapshot> & snapshot,
  const global_planner::LegacySearchParameters & parameters,
  std::string * rejection_reason)
{
  // Preserve the lethal index only when the exact same immutable cost view is
  // being re-pinned. Every field is still cleared below, so an old
  // updateGraph() can never inherit a stale snapshot.
  const auto previous_static = scene_static_view_;
  const auto previous_cost = scene_cost_view_;
  const auto previous_lethal_tree = scene_lethal_kdtree_;
  ResetSceneSnapshot();
  scene_requested_ = true;

  std::shared_ptr<const perception_3d::LegacyStaticView> static_view;
  std::shared_ptr<const perception_3d::LegacyCostView> cost_view;
  std::string reason;
  if(!global_planner::BuildLegacySceneReadView(
      snapshot, parameters, &static_view, &cost_view, &reason))
  {
    scene_rejection_reason_ = reason;
    if(rejection_reason != nullptr){
      *rejection_reason = reason;
    }
    return false;
  }

  scene_static_view_ = static_view;
  scene_cost_view_ = cost_view;
  scene_lethal_kdtree_ =
    (static_view == previous_static && cost_view == previous_cost &&
    previous_lethal_tree) ?
    previous_lethal_tree :
    global_planner::BuildLegacyLethalKdTree(cost_view);
  scene_parameters_ = parameters;
  scene_active_ = true;
  scene_rejection_reason_.clear();
  if(rejection_reason != nullptr){
    rejection_reason->clear();
  }
  return true;
}

double A_Star_on_Graph::getPitchFromParent2Expanding(pcl::PointXYZI m_pcl_current_parent, pcl::PointXYZI m_pcl_current, pcl::PointXYZI m_pcl_expanding){
  //@ calculate vector: parent -> current
  float vx1, vy1, s1;
  vx1 = m_pcl_current.x - m_pcl_current_parent.x;
  vy1 = m_pcl_current.y - m_pcl_current_parent.y;
  s1 = sqrt(vx1*vx1 + vy1*vy1);
  //@ calculate vector: current -> expanding
  float vx2, vy2, s2;
  vx2 = m_pcl_expanding.x - m_pcl_current.x;
  vy2 = m_pcl_expanding.y - m_pcl_current.y;
  s2 = sqrt(vx2*vx2 + vy2*vy2);

  float pitch = fabs(m_pcl_current_parent.z - m_pcl_expanding.z)/(s1+s2);

  return pitch;
}

double A_Star_on_Graph::getThetaFromParent2Expanding(pcl::PointXYZI m_pcl_current_parent, pcl::PointXYZI m_pcl_current, pcl::PointXYZI m_pcl_expanding){
  //@ calculate vector: parent -> current
  float vx1, vy1;
  vx1 = m_pcl_current.x - m_pcl_current_parent.x;
  vy1 = m_pcl_current.y - m_pcl_current_parent.y;
  //@ calculate vector: current -> expanding
  float vx2, vy2;
  vx2 = m_pcl_expanding.x - m_pcl_current.x;
  vy2 = m_pcl_expanding.y - m_pcl_current.y;
  float cos_theta = (vx1*vx2 + vy1*vy2)/(sqrt(vx1*vx1+vy1*vy1)*sqrt(vx2*vx2+vy2*vy2));
  if(fabs(cos_theta)>1)
    cos_theta = 1.0;
  double theta_of_vector = acos(cos_theta);
  if(vx1==0 && vy1==0)
    theta_of_vector = 0;
  else if(vx2==0 && vy2==0)
    theta_of_vector = 0;
  else if(fabs(fabs(vx1)-fabs(vx2))<=0.0001)
    theta_of_vector = 0;
  
  if(fabs(theta_of_vector)<=0.345)//cap
    theta_of_vector = 0.0;

  return theta_of_vector;
}

bool A_Star_on_Graph::isLineOfSightClear(pcl::PointXYZI& pcl_current, pcl::PointXYZI& pcl_expanding, double inscribed_radius){

  const nanoflann::KdTreeFLANN<pcl::PointXYZI>::ConstPtr & lethal_tree =
    scene_active_ ? scene_lethal_kdtree_ : kdtree_lethal_;

  if(!lethal_tree || !lethal_tree->getInputCloud() ||
    lethal_tree->getInputCloud()->empty() ||
    inscribed_radius <= 1e-6 || !std::isfinite(inscribed_radius))
  {
    return true;
  }

  //@ generate line equation
  float dX =
      pcl_expanding.x - pcl_current.x;
  float dY =
      pcl_expanding.y - pcl_current.y;
  float dZ =
      pcl_expanding.z - pcl_current.z;
  
  float distance = sqrt(dX*dX + dY*dY + dZ*dZ);
  if(distance <= 1e-6 || !std::isfinite(distance)){
    return true;
  }
  distance = distance/inscribed_radius; //sample by every inscribed radius
  float dt = 1/distance;
  for(float t=0; t<=1.0+dt; t+=dt){
    float r = t;
    if(t>=1.0) //@ make sure we examine t=1.0
      r = 1.0;
    pcl::PointXYZI a_pt;
    a_pt.intensity = 0.0;
    a_pt.x = pcl_current.x + dX*r;
    a_pt.y = pcl_current.y + dY*r;
    a_pt.z = pcl_current.z + dZ*r;
    std::vector<int> pidx;
    std::vector<float> prsd;
    lethal_tree->radiusSearch(a_pt, 2*inscribed_radius, pidx, prsd);
    if(pidx.size()>1){
      return false;
    }
  }
  return true;
}

void A_Star_on_Graph::getPath(
  unsigned int start, unsigned int goal,
  std::vector<unsigned int>& path,
  GraphPlanDiagnostics * diagnostics){
  getPath(start, goal, path, diagnostics, nullptr);
}

void A_Star_on_Graph::getPath(
  unsigned int start, unsigned int goal,
  std::vector<unsigned int>& path,
  GraphPlanDiagnostics * diagnostics,
  global_planner::AStarPlanTrace * trace){

  if(diagnostics){
    diagnostics->reset();
  }
  if(trace){
    trace->reset();
  }

  // A failed setSceneSnapshot() is a hard refusal for this planner object. It
  // must not quietly fall back to live perception data.
  if(scene_requested_ && !scene_active_){
    return;
  }
  const bool frozen = scene_active_;
  const pcl::PointCloud<pcl::PointXYZI> * ground = frozen ?
    scene_static_view_->groundPoints().get() : pc_original_z_up_.get();

  if(!ground || ground->points.empty() ||
    start >= ground->points.size() ||
    goal >= ground->points.size() ||
    !ASLS_)
  {
    return;
  }
  if(frozen){
    if(!scene_cost_view_ || !scene_static_view_ ||
      !scene_static_view_->groundKdtree())
    {
      return;
    }
  }else if(!perception_ros_ || !ASLS_->kdtree_ground_){
    return;
  }

  /*
  Create the first node which is start and add into frontier
  */

  pcl::PointXYZI pcl_goal = ground->points[goal];
  pcl::PointXYZI pcl_start = ground->points[start];
  float f = sqrt(pcl::geometry::squaredDistance(pcl_start, pcl_goal));
  Node_t current_node = {.self_index=start, .g=0, .h=0, .f=f, .parent_index=start, .is_closed=false, .is_opened=true};

  if(frozen){
    ASLS_->Initial(ground->points.size());
  }else{
    ASLS_->Initial();
  }
  ASLS_->updateNode(current_node);
  
  double inscribed_radius = 0.0;
  double inflation_descending_rate = 0.0;
  double max_obstacle_distance = 0.0;
  if(frozen){
    inscribed_radius = scene_parameters_.inscribed_radius;
    inflation_descending_rate = scene_parameters_.inflation_descending_rate;
    max_obstacle_distance = scene_parameters_.max_obstacle_distance;
  }else{
    if(!perception_ros_->getGlobalUtils()){
      return;
    }
    inscribed_radius = perception_ros_->getGlobalUtils()->getInscribedRadius();
    inflation_descending_rate = perception_ros_->getGlobalUtils()->getInflationDescendingRate();
    max_obstacle_distance = perception_ros_->getGlobalUtils()->getMaxObstacleDistance();
  }
  (void)max_obstacle_distance;

  if(!frozen){
    const auto aggregate_lethal_started_at = std::chrono::steady_clock::now();
    perception_ros_->getStackedPerception()->aggregateLethal();
    if(diagnostics){
      diagnostics->aggregate_lethal_ms = ElapsedMs(aggregate_lethal_started_at);
    }
    //@ generate kd-tree and handle no point cloud edge case
    kdtree_lethal_.reset();
    auto shared_data = perception_ros_->getSharedDataPtr();
    if(shared_data && shared_data->aggregate_lethal_ &&
      shared_data->aggregate_lethal_->points.size()>0)
    {
      kdtree_lethal_ = std::make_shared<
        nanoflann::KdTreeFLANN<pcl::PointXYZI>>();
      kdtree_lethal_->setInputCloud(shared_data->aggregate_lethal_);
    }
  }

  while(!ASLS_->isFrontierEmpty()){ 
    /*Pop minimum F, we leverage prior queue, so we dont need to loop frontier everytime*/
    current_node = ASLS_->getNode_wi_MinimumF();
    if(current_node.is_closed){
      break;
    }

    //RCLCPP_DEBUG(rclcpp::get_logger("astar"), "Expand node: %u", current_node.self_index);
    /*Get successors*/
    pcl::PointXYZI pcl_now = ground->points[current_node.self_index];
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    if(frozen){
      const auto & ground_tree = scene_static_view_->groundKdtree();
      ground_tree->radiusSearch(
        pcl_now, a_star_expanding_radius_, pointIdxRadiusSearch,
        pointRadiusSquaredDistance);
      //@dealing with orphan node
      if(pointIdxRadiusSearch.size()<8){
        ground_tree->nearestKSearch(
          pcl_now, 8, pointIdxRadiusSearch, pointRadiusSquaredDistance);
      }
    }else{
      ASLS_->kdtree_ground_->radiusSearch(
        pcl_now, a_star_expanding_radius_, pointIdxRadiusSearch,
        pointRadiusSquaredDistance);
      //@dealing with orphan node
      if(pointIdxRadiusSearch.size()<8){
        ASLS_->kdtree_ground_->nearestKSearch(
          pcl_now, 8, pointIdxRadiusSearch, pointRadiusSquaredDistance);
      }
    }

    if(trace){
      trace->expanded_nodes.push_back(current_node.self_index);
      for(const int neighbor_index : pointIdxRadiusSearch){
        if(neighbor_index >= 0 &&
          static_cast<std::size_t>(neighbor_index) < ground->points.size())
        {
          trace->neighbor_order.push_back(
            static_cast<unsigned int>(neighbor_index));
        }
      }
    }

    if(pointIdxRadiusSearch.empty()){
      ASLS_->closeNode(current_node);
      continue;
    }

    //@ calculated average intensity, because we have sparse low cost orphan, and it is unlikely to have a low cost node surrounded by high cost nodes
    float avg_intensity = 0.0;
    std::size_t valid_neighbor_count = 0;
    for(unsigned int it = 0; it!=pointIdxRadiusSearch.size(); it++){
      if(pointIdxRadiusSearch[it] < 0 ||
        static_cast<std::size_t>(pointIdxRadiusSearch[it]) >= ground->points.size())
      {
        continue;
      }
      avg_intensity += ground->points[pointIdxRadiusSearch[it]].intensity;
      valid_neighbor_count++;
    }
    if(valid_neighbor_count == 0){
      ASLS_->closeNode(current_node);
      continue;
    }
    avg_intensity = avg_intensity/valid_neighbor_count;

    const std::size_t neighbor_count = pointIdxRadiusSearch.size();
    std::vector<double> neighbor_dgraph(neighbor_count, 0.0);
    std::vector<float> neighbor_weight(neighbor_count, 0.0f);
    if(frozen){
      // The pinned cost/static views are immutable, so no live guard, getter
      // call or lock participates in the frozen search.
      for(unsigned int it = 0; it!=pointIdxRadiusSearch.size(); it++){
        const int current_expanding_index = pointIdxRadiusSearch[it];
        if(current_expanding_index < 0 ||
          static_cast<std::size_t>(current_expanding_index) >= ground->points.size())
        {
          continue;
        }

        neighbor_dgraph[it] = scene_cost_view_->minDGraphValue(
          static_cast<std::size_t>(current_expanding_index));
        if(neighbor_dgraph[it] >= inscribed_radius){
          neighbor_weight[it] = scene_static_view_->graph().getNodeWeight(
            static_cast<unsigned int>(current_expanding_index));
        }
      }
    }else{
      // One short shared-map read lock for this expanded node's neighbor batch.
      // Only the dGraph and static node-weight reads need the production guard;
      // LOS, cost math and A* bookkeeping run after it is released.
      {
        auto shared_data = perception_ros_->getSharedDataPtr();
        std::unique_lock<std::recursive_mutex> batch_lock;
        if(shared_data){
          const auto batch_lock_started_at = std::chrono::steady_clock::now();
          batch_lock = std::unique_lock<std::recursive_mutex>(
            shared_data->ground_kdtree_cb_mutex_);
          if(diagnostics){
            diagnostics->recordBatchLock(ElapsedMs(batch_lock_started_at));
          }
        }

        for(unsigned int it = 0; it!=pointIdxRadiusSearch.size(); it++){
          int current_expanding_index = pointIdxRadiusSearch[it];
          if(current_expanding_index < 0 ||
            static_cast<std::size_t>(current_expanding_index) >= ground->points.size())
          {
            continue;
          }

          //@ dGraphValue is the distance to lethal
          const auto dgraph_query_started_at = std::chrono::steady_clock::now();
          neighbor_dgraph[it] = perception_ros_->get_min_dGraphValue(current_expanding_index);
          if(diagnostics){
            diagnostics->recordDGraphQuery(ElapsedMs(dgraph_query_started_at));
          }

          // Static node weights are O(1) in production. Only non-lethal
          // neighbours need a weight; LOS-rejected reads stay unused.
          if(shared_data && neighbor_dgraph[it] >= inscribed_radius){
            neighbor_weight[it] = shared_data->sGraph_ptr_->getNodeWeight(current_expanding_index);
          }
        }
      }
    }

    for(unsigned int it = 0; it!=pointIdxRadiusSearch.size(); it++){
      
      int current_expanding_index = pointIdxRadiusSearch[it];
      if(current_expanding_index < 0 ||
        static_cast<std::size_t>(current_expanding_index) >= ground->points.size())
      {
        continue;
      }
      float current_expanding_g = sqrt(pointRadiusSquaredDistance[it]);

      //@ dGraphValue is the distance to lethal
      double dGraphValue = neighbor_dgraph[it];

      /*This is for lethal*/
      if(dGraphValue<inscribed_radius){
        //RCLCPP_DEBUG(rclcpp::get_logger("astar"), "%.2f,%.2f,%.2f, v: %.2f",ground->points[(*it).first].x,ground->points[(*it).first].y,ground->points[(*it).first].z, dGraphValue);
        continue;
      }

      pcl::PointXYZI pcl_current = ground->points[current_node.self_index];
      pcl::PointXYZI pcl_current_parent = ground->points[current_node.parent_index];
      pcl::PointXYZI pcl_expanding = ground->points[current_expanding_index];

      //@ check line-of-sight when distance is 2 times larger than inscribed_radius
      if(current_expanding_g>=2*inscribed_radius){
        if(!isLineOfSightClear(pcl_current, pcl_expanding, inscribed_radius))
          continue;
      }
      
      double factor = exp(-1.0 * inflation_descending_rate * (dGraphValue - inscribed_radius));

      //@ get current_parent, current, expanding to compute theta od expanding
      double theta = getThetaFromParent2Expanding(pcl_current_parent, pcl_current, pcl_expanding);
      
      //if(getPitchFromParent2Expanding(pcl_current_parent, pcl_current, pcl_expanding)>0.2)
      //  continue;
      
      float ground_edge_weight = avg_intensity;
      float node_weight = neighbor_weight[it];
      float new_g = current_node.g + current_expanding_g + factor * 1.0 + node_weight + theta*turning_weight_ + ground_edge_weight;
      float new_h = sqrt(pcl::geometry::squaredDistance(pcl_expanding, pcl_goal));
      float new_f = new_g + new_h;

      Node_t new_node = {.self_index=static_cast<unsigned int>(current_expanding_index), .g=new_g, .h=new_h, .f=new_f, .parent_index=current_node.self_index, .is_closed=false, .is_opened=true};

      /*Check is in closed list*/
      if(ASLS_->isClosed(current_expanding_index))
        continue;
      /*Check is in opened list*/
      else if(ASLS_->isOpened(current_expanding_index)){
        if(ASLS_->getGVal(new_node)>new_g){
          ASLS_->updateNode(new_node);          
        }
      }
      /*addNode*/
      else{
        ASLS_->updateNode(new_node);
      }
        
      
    }

    /*Close this node*/
    ASLS_->closeNode(current_node);

    /*If goal is in closed list, we are done*/
    if(ASLS_->isClosed(goal)){
      Node_t trace_back = ASLS_->getNode(goal);
      if(trace){
        trace->found = true;
        trace->goal_g = trace_back.g;
      }
      while(trace_back.self_index!=trace_back.parent_index){
        path.push_back(trace_back.self_index);
        trace_back = ASLS_->getNode(trace_back.parent_index);
      }
      path.push_back(trace_back.self_index);//Push start point
      std::reverse(path.begin(),path.end()); 
      break;
    }

    /*Check if*/
  }
}
