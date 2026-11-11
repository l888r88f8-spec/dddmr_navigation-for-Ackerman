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

/*For graph*/
#include <unordered_map>
#include <set>
#include <queue> 
#include <cmath>
#include <cstddef>

/*For pcl::PointXYZ*/
#include <pcl/common/geometry.h>
#include <math.h>

/*
for graph
type edge_t is defined here
type graph_t is defined here
*/

/*For perception*/
#include <perception_3d/perception_3d_ros.h>
#include <global_planner/nanoflann_pcl.hpp>

typedef struct {
  unsigned int self_index;
  float g, h, f;
  unsigned int parent_index; 
  bool is_closed, is_opened;
} Node_t;

// Diagnostic-only per-plan accumulation. The dGraph timer wraps the actual
// get_min_dGraphValue() call, so it includes both mutex wait and lookup work.
// It is deliberately not named lock_wait_ms: it does not measure the lock directly.
struct GraphPlanDiagnostics
{
  std::size_t dgraph_calls = 0;
  double dgraph_query_ms = 0.0;
  double dgraph_max_ms = 0.0;
  std::size_t dgraph_over_1ms = 0;
  double aggregate_lethal_ms = 0.0;
  // Time to acquire the batched shared-map read guard. Kept separate from
  // dgraph_query_ms so a getter-time reduction cannot hide moved lock wait.
  std::size_t batch_lock_calls = 0;
  double batch_lock_wait_ms = 0.0;
  double batch_lock_max_ms = 0.0;

  void reset()
  {
    dgraph_calls = 0;
    dgraph_query_ms = 0.0;
    dgraph_max_ms = 0.0;
    dgraph_over_1ms = 0;
    aggregate_lethal_ms = 0.0;
    batch_lock_calls = 0;
    batch_lock_wait_ms = 0.0;
    batch_lock_max_ms = 0.0;
  }

  void recordBatchLock(double elapsed_ms)
  {
    ++batch_lock_calls;
    batch_lock_wait_ms += elapsed_ms;
    if (elapsed_ms > batch_lock_max_ms) {
      batch_lock_max_ms = elapsed_ms;
    }
  }

  void recordDGraphQuery(double elapsed_ms)
  {
    ++dgraph_calls;
    dgraph_query_ms += elapsed_ms;
    if (elapsed_ms > dgraph_max_ms) {
      dgraph_max_ms = elapsed_ms;
    }
    if (elapsed_ms > 1.0) {
      ++dgraph_over_1ms;
    }
  }
};

typedef std::pair<double, unsigned int> f_p_;

class AstarList{
  public:
    AstarList(pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_original_z_up);

    void Initial();
    void updateNode(Node_t& a_node);
    void closeNode(Node_t& a_node);
    float getGVal(Node_t& a_node);
    Node_t getNode_wi_MinimumF();
    Node_t getNode(unsigned int node_index);
    bool isClosed(unsigned int node_index);
    bool isOpened(unsigned int node_index);
    bool isFrontierEmpty();
    /*Static graph is for path planning and list*/
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_original_z_up_;
    nanoflann::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_ground_;
  private:
    /*
    We handle closed list and accessory by unordered::map to get time complexity of O(1)
    Key: node id
    content: Node
    */
    std::unordered_map<unsigned int, Node_t> as_list_;

    /*
    We push back a pair <f,node_index> by the priority, hence ,the time complexity of checking minimum f is O(1)
    */
    //std::priority_queue<f_p_> f_priority_queue_;
    std::set<f_p_> f_priority_set_;
};

class A_Star_on_Graph{

    public:
      A_Star_on_Graph(pcl::PointCloud<pcl::PointXYZI>::Ptr pc_original_z_up, 
        std::shared_ptr<perception_3d::Perception3D_ROS> perception_ros,
        double a_star_expanding_radius);
      
      ~A_Star_on_Graph();
      
      void updateGraph(pcl::PointCloud<pcl::PointXYZI>::Ptr pc_original_z_up);

      void getPath( unsigned int start, unsigned int goal, std::vector<unsigned int>& path,
        GraphPlanDiagnostics * diagnostics = nullptr);
      
      void setupTurningWeight(double m_weight){turning_weight_ = m_weight;}

    private:
      
      //@ kd-tree for line-of-sight
      nanoflann::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_lethal_;

      pcl::PointCloud<pcl::PointXYZI>::Ptr pc_original_z_up_;

      /*Provide dynamic graph for obstacle avoidance*/
      std::shared_ptr<perception_3d::Perception3D_ROS> perception_ros_;
      
      /*Create the list*/
      AstarList* ASLS_;

      //@ turning weight of the node
      double turning_weight_;
      
      //@ neighborhodd expanding radius
      double a_star_expanding_radius_;

      double getThetaFromParent2Expanding(pcl::PointXYZI m_pcl_current_parent, pcl::PointXYZI m_pcl_current, pcl::PointXYZI m_pcl_expanding);
      double getPitchFromParent2Expanding(pcl::PointXYZI m_pcl_current_parent, pcl::PointXYZI m_pcl_current, pcl::PointXYZI m_pcl_expanding);

      bool isLineOfSightClear(pcl::PointXYZI& pcl_current, pcl::PointXYZI& pcl_expanding, double inscribed_radius);
};
