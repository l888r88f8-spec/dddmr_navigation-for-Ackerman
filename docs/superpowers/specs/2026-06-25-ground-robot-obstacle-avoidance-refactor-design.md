# 地面机器人避障重构与优化设计

日期：2026-06-25

## 1. 背景

当前项目面向 Ackermann 地面机器人，导航链路大致为：

```text
LIO-SAM / localization odometry
  -> p2p_move_base
  -> global_planner
  -> local_planner
  -> perception_3d_local
  -> mpc_critics / CollisionModel
  -> cmd_vel
```

当前 `p2p_move_base_localization.yaml` 使用 `regulated_pure_pursuit` 作为局部跟踪控制器。避障主要依赖局部感知聚合点云和 `CollisionModel` 对 RPP 预测轨迹做碰撞检查。当预测轨迹碰撞时，局部规划器返回 `PATH_BLOCKED_REPLANNING`，上层状态机立即调用全局重规划。

实际运行中暴露出几个问题：

1. 避障速度慢。前方障碍出现后车辆容易停下，等待全局规划结果。
2. 全局规划器较慢。当前配置使用 `hybrid_a_star`，一次规划可能耗时较长，不适合频繁触发。
3. 存在误触发全局重规划。单次点云误检、窄通道边缘、短暂动态障碍、TF/定位抖动、路径裁剪失败都可能导致重规划。
4. RPP 更偏路径跟踪，不是强局部避障规划器。它只评估当前跟踪生成的一条预测轨迹，缺少主动左绕/右绕/减速/回归路径的候选轨迹选择。

本设计目标是重构为更符合地面机器人特性的避障架构，不以最小代码改动为优先，而以稳定性、实时性、可解释性和扩展性为优先。

## 2. 设计目标

### 2.1 功能目标

1. 临时障碍能够局部绕行或短时等待，不立即全局重规划。
2. 窄通道通过时减少误判和反复停车。
3. 静态路径被持续阻塞时，能够确认后触发全局重规划。
4. 局部避障动作更快、更平滑，避免“一碰撞就停下等全局规划”。
5. 全局规划只在局部确认无解时触发，并提供快速规划路径。
6. 定位、TF、路径裁剪异常必须和避障失败分开诊断，不能混为“障碍阻塞”。

### 2.2 性能目标

建议目标值：

1. 局部规划频率：10-20 Hz。
2. 局部避障响应延迟：小于 200 ms。
3. 单次障碍误检不触发全局重规划。
4. 动态障碍短时出现时，优先局部处理或等待 1-3 秒。
5. 全局快速规划目标耗时：常规场景小于 1-2 秒。
6. 精细全局规划可以较慢，但不能阻塞局部控制循环。
7. 必须能在 Jetson Orin NX 上与重定位、雷达驱动、IMU 驱动同时运行。
8. 避障模块不能独占 CPU、内存带宽或 ROS executor，必须提供可配置预算和降级策略。

### 2.3 平台预算目标

目标部署平台为 Jetson Orin NX。避障重构不能假设有桌面级 CPU/GPU 资源，必须为共机运行预留预算：

```text
同机常驻模块：
  - 雷达驱动
  - IMU 驱动
  - LIO-SAM / 重定位
  - TF / robot_state_publisher
  - p2p_move_base
  - global_planner
  - local_planner / obstacle layer
  - RViz 可选，实车长期运行时不作为必需模块
```

建议预算起点：

```yaml
orin_nx_budget:
  local_obstacle_layer_cpu_percent_target: 15
  local_planner_cpu_percent_target: 20
  navigation_supervisor_cpu_percent_target: 5
  global_planner_cpu_percent_target_when_active: 35
  navigation_memory_target_mb: 700
  navigation_memory_hard_limit_mb: 1200
  max_local_planning_time_ms: 50
  max_obstacle_update_time_ms: 50
  max_end_to_end_local_cycle_ms: 80
```

预算解释：

1. 以上预算是导航新增模块的目标，不包含重定位和驱动。
2. 全局规划允许短时高 CPU，但必须低频触发，且不能阻塞局部安全停车。
3. RViz、全量候选轨迹发布、全分辨率 debug 点云不应在实车常态运行中开启。
4. 如果 Orin NX 电源模式、散热或频率限制导致性能下降，系统必须自动降级采样和 debug，而不是继续超时运行。

### 2.4 安全目标

1. 硬碰撞检查必须保留，任何候选轨迹与车体 footprint 冲突时直接拒绝。
2. 近障碍区域使用连续代价，而不是只有碰撞/不碰撞二值判断。
3. 动态障碍需要引入短期预测，避免车辆朝向横穿障碍加速。
4. 局部规划无可行轨迹时必须能可靠停车。
5. 所有重规划触发必须可追踪原因，便于现场调试。

## 3. 总体架构

推荐重构后的架构：

```text
Sensors / Localization
  - lio_sam/cloud_nonground
  - static map / ground map
  - /lio_sam/localization/odometry
  - TF: map -> base_link

Perception3D
  - 原始点云处理
  - 静态地图层
  - 雷达障碍层

LocalObstacleLayer
  - 时序滤波
  - 障碍聚类
  - 动态障碍跟踪
  - 2.5D costmap
  - distance map
  - footprint collision query
  - path blockage evidence

AckermannLocalPlanner
  - global route pruning
  - state lattice trajectory generation
  - trajectory evaluation
  - speed planning
  - cmd_vel output

NavigationSupervisor
  - 状态机
  - 阻塞确认
  - 重规划防抖
  - 等待/恢复/全局重规划决策

GlobalPlannerManager
  - 快速 graph_a_star
  - 精细 hybrid_a_star
  - 请求节流
  - stale result 处理
```

核心原则：

```text
避障优先在局部解决。
局部确认无解后，才允许全局规划。
全局规划慢时不能阻塞局部安全停车和局部恢复。
```

## 4. 模块职责

### 4.1 LocalObstacleLayer

`LocalObstacleLayer` 是局部环境建模层，不是局部规划器。它不发布速度，不选择最终轨迹，也不直接触发全局重规划。

它的职责是回答局部规划器和上层状态机的问题：

1. 某个位置是否被障碍占据？
2. 某个车体 footprint 是否碰撞？
3. 轨迹上每个采样点离最近障碍多远？
4. 前方参考路径是否被持续阻塞？
5. 障碍是短暂噪声、静态障碍，还是动态障碍？
6. 动态障碍未来 1-3 秒可能出现在哪里？
7. 当前感知数据是否足够新鲜、稳定、可信？

#### 输入

1. `lio_sam/cloud_nonground`：局部非地面障碍点云。
2. 静态地图障碍层：来自 `mapcloud` / `mapground` 或现有 `StaticLayer`。
3. `map -> base_link` TF。
4. `/lio_sam/localization/odometry`。
5. 当前全局路径裁剪片段。

#### 输出

建议提供结构化接口：

```cpp
struct LocalObstacleSnapshot {
  rclcpp::Time stamp;
  bool sensor_ok;
  bool tf_ok;
  double data_age_sec;
  std::vector<ObstacleCluster> clusters;
  LocalCostmap2D costmap;
  DistanceMap distance_map;
  PathBlockageEvidence path_blockage;
};
```

查询接口：

```cpp
bool isFootprintCollision(const Footprint& footprint) const;
double minDistanceToObstacle(const Footprint& footprint) const;
double trajectoryMinClearance(const Trajectory& trajectory) const;
CollisionCheckResult checkTrajectory(const Trajectory& trajectory) const;
PathBlockageEvidence evaluatePathBlockage(const PathSegment& local_path) const;
```

#### 时序滤波

为减少点云闪烁误触发，障碍层应维护短期时序缓存。

建议：

1. 点云障碍保留时间：0.5-1.0 秒。
2. 动态障碍单独维护轨迹，不简单长期膨胀。
3. 对单帧孤立小簇加低置信度。
4. 对连续多帧出现的障碍提升置信度。
5. 对突然消失的障碍设置短时衰减，而不是立刻清除。

示例置信度：

```text
confidence =
  observation_count_weight
  - age_decay
  - isolated_noise_penalty
```

低置信度障碍可以影响减速，但不应单独触发全局重规划。

#### Costmap 与 Distance Map

建议维护以 `base_link` 或 `map` 为中心的局部 2.5D 栅格：

```text
范围：前方 8-12 m，后方 2-4 m，左右 4-6 m
分辨率：0.05-0.10 m
更新频率：10-20 Hz
```

每个栅格保存：

1. occupancy：是否占据。
2. obstacle_height：障碍高度。
3. confidence：障碍置信度。
4. source：static / lidar / dynamic_prediction。
5. last_seen_time。
6. distance_to_obstacle。

Distance Map 用于连续避障代价：

```text
distance < collision_distance: 硬拒绝
distance < safety_margin: 高代价
distance < comfort_margin: 中等代价
distance >= comfort_margin: 低代价或 0
```

### 4.2 ObstacleTracker

动态障碍跟踪可以作为 `LocalObstacleLayer` 内部子模块。

职责：

1. 对局部点云聚类。
2. 跟踪障碍中心、大小、速度。
3. 区分静态障碍和动态障碍。
4. 预测未来短时间占据区域。

建议数据结构：

```cpp
struct ObstacleCluster {
  int id;
  geometry_msgs::msg::Point center;
  geometry_msgs::msg::Vector3 size;
  geometry_msgs::msg::Vector3 velocity;
  double confidence;
  bool dynamic;
  rclcpp::Time last_seen;
};
```

动态判断：

```text
连续多帧位置变化明显 -> dynamic
速度低于阈值且位置稳定 -> static
只出现一帧或点数很少 -> uncertain
```

动态障碍处理策略：

1. 横穿路径但很快离开：优先 `YIELD_WAIT`，不全局重规划。
2. 朝机器人接近：提前减速或停车。
3. 静止超过阈值：转为静态阻塞证据。

### 4.3 AckermannLocalPlanner

这是重构的局部规划器核心。它应替代当前“RPP 单条轨迹碰撞即重规划”的逻辑。

#### 职责

1. 跟踪全局路径。
2. 生成符合 Ackermann 运动学约束的候选轨迹。
3. 使用 LocalObstacleLayer 进行碰撞和距离评价。
4. 选择可行且代价最低的轨迹。
5. 输出 `cmd_vel`。
6. 给 NavigationSupervisor 返回结构化状态，而不是直接要求全局重规划。

#### 候选轨迹

建议使用 State Lattice / Motion Primitive：

```text
速度采样：
  stop
  crawl
  nominal

曲率采样：
  hard_left
  medium_left
  slight_left
  straight
  slight_right
  medium_right
  hard_right

时间/距离 horizon：
  1.5-4.0 s 或 2-6 m
```

每条轨迹必须满足：

1. 最大转角约束。
2. 最小转弯半径约束。
3. 最大角速度约束。
4. 最大横向加速度约束。
5. 最大线加速度和减速度约束。
6. 底盘速度限制。

#### 轨迹评价

建议将评价拆成硬约束和软代价。

硬约束：

1. footprint collision。
2. 进入不可通行区域。
3. 超出车辆运动学约束。
4. 轨迹终点明显远离可恢复区域。

软代价：

```text
total_cost =
  w_clearance * obstacle_distance_cost
  + w_path * global_path_deviation_cost
  + w_heading * heading_error_cost
  + w_curvature * curvature_cost
  + w_speed_smooth * speed_change_cost
  + w_progress * progress_cost
  + w_dynamic * dynamic_obstacle_risk_cost
  + w_reverse * reverse_penalty
```

其中：

1. `obstacle_distance_cost` 来自 LocalObstacleLayer 的 distance map。
2. `global_path_deviation_cost` 防止绕障偏离太远。
3. `progress_cost` 奖励朝目标推进。
4. `dynamic_obstacle_risk_cost` 惩罚与动态障碍预测区域重叠。
5. `speed_change_cost` 降低急停急加速。

#### 输出状态

局部规划器不再只返回 `TRAJECTORY_FOUND`、`PATH_BLOCKED_REPLANNING` 这种粗粒度状态，而是返回更详细的结果：

```cpp
enum class LocalPlanStatus {
  OK,
  TEMPORARILY_BLOCKED,
  STATIC_BLOCKED,
  NO_FEASIBLE_TRAJECTORY,
  ROUTE_DEVIATION,
  TF_INVALID,
  PERCEPTION_STALE,
  GOAL_REACHED
};

struct LocalPlanResult {
  LocalPlanStatus status;
  geometry_msgs::msg::Twist cmd_vel;
  Trajectory selected_trajectory;
  double min_clearance;
  double best_cost;
  std::string primary_reject_reason;
  size_t collision_reject_count;
  size_t dynamic_risk_reject_count;
  size_t feasible_count;
};
```

这样上层可以区分：

1. 是临时动态障碍。
2. 是静态障碍持续阻塞。
3. 是定位/TF 问题。
4. 是路径偏离。
5. 是所有轨迹都撞。

### 4.4 NavigationSupervisor

当前 `p2p_move_base` 的 FSM 同时处理跟踪、等待、恢复、全局重规划。建议重构出更明确的 `NavigationSupervisor`。

#### 状态

建议状态机：

```text
IDLE
REQUEST_GLOBAL_ROUTE
TRACK_GLOBAL_PATH
LOCAL_AVOIDANCE
YIELD_WAIT
LOCAL_RECOVERY
GLOBAL_REPLAN
GOAL_ALIGNMENT
SUCCEEDED
FAILED
```

#### 状态含义

`TRACK_GLOBAL_PATH`：
正常跟踪全局路径。局部规划器可以使用 RPP 或 lattice 中的路径跟踪模式。

`LOCAL_AVOIDANCE`：
当前跟踪轨迹受阻，但局部 lattice 有可行绕障轨迹。此状态不触发全局规划。

`YIELD_WAIT`：
局部无可行轨迹，但判断障碍可能是动态障碍或短时阻塞。车辆停车等待，并持续重新评估。

`GLOBAL_REPLAN`：
确认局部无解，且障碍持续阻塞或当前全局路径不再适合，才请求全局重规划。

`LOCAL_RECOVERY`：
处理卡住、路径偏离、低速振荡等问题。对 Ackermann 车辆，恢复动作不应默认原地旋转，应根据底盘能力设计。

#### 重规划防抖

全局重规划必须满足确认条件，不能由单次碰撞触发。

建议逻辑：

```text
if local_plan_result == NO_FEASIBLE_TRAJECTORY:
  accumulate blocked_duration
  if obstacle_is_dynamic and blocked_duration < dynamic_wait_timeout:
    state = YIELD_WAIT
  else if feasible_count == 0 for N consecutive cycles
       and blocked_duration > static_blocked_confirm_time:
    state = GLOBAL_REPLAN
  else:
    state = LOCAL_AVOIDANCE or YIELD_WAIT
```

参数建议：

```yaml
navigation_supervisor:
  collision_confirm_cycles: 5
  blocked_confirm_time: 1.0
  dynamic_wait_timeout: 3.0
  static_replan_timeout: 2.0
  min_replan_interval: 3.0
  max_replan_attempts_per_goal: 5
```

`min_replan_interval` 用于防止短时间内连续请求全局规划。

#### route_deviation 防抖

当前路径裁剪失败容易触发全局重规划。重构后应区分路径偏离和避障阻塞。

建议规则：

```text
route_deviation_confirmed =
  robot_to_route_distance > threshold
  for N consecutive cycles
  and TF is valid
  and odom jump not detected
  and local planner cannot safely return to route
```

如果车辆能通过局部轨迹回归全局路径，不触发全局重规划。

### 4.5 GlobalPlannerManager

全局规划需要从“每次都精细规划”改为“快速规划优先，精细规划可选”。

#### 两级规划

建议：

```text
Fast Global Planner:
  graph_a_star
  用于障碍后快速恢复通路

Refined Global Planner:
  hybrid_a_star
  用于初始路径、长距离规划、后台优化
```

触发策略：

1. 初始导航：可以用 `hybrid_a_star`。
2. 避障失败后的紧急重规划：先用 `graph_a_star`。
3. 快速路径成功后，局部 lattice 负责 Ackermann 可行性。
4. 如需要更平滑路径，后台再用 `hybrid_a_star` 精修，但不能阻塞当前安全控制。

#### 请求管理

必须增加：

1. 重规划最小间隔。
2. 正在规划时避免重复请求。
3. 旧规划结果过期检查。
4. 规划超时取消。
5. 失败原因分类。

建议输出诊断：

```text
request_id
goal_seq
planner_backend
trigger_reason
planning_time
expansions
result_class
path_poses
```

## 5. 数据流

### 5.1 正常跟踪

```text
Global route
  -> prune local route segment
  -> LocalObstacleLayer snapshot
  -> AckermannLocalPlanner generate candidates
  -> evaluate candidates
  -> selected trajectory
  -> cmd_vel
```

### 5.2 临时障碍

```text
Obstacle appears
  -> LocalObstacleLayer marks dynamic/uncertain obstacle
  -> current best path blocked
  -> lattice searches left/right/slow trajectories
  -> if feasible: LOCAL_AVOIDANCE
  -> if not feasible but dynamic likely: YIELD_WAIT
  -> no global replan yet
```

### 5.3 静态路径阻塞

```text
Obstacle remains stable for multiple frames
  -> static blockage confidence increases
  -> local planner no feasible trajectory
  -> blocked duration exceeds threshold
  -> NavigationSupervisor enters GLOBAL_REPLAN
  -> fast graph_a_star route request
```

### 5.4 窄通道

```text
Obstacle points close to both sides
  -> distance map gives high but continuous cost
  -> footprint collision stays hard reject
  -> selected trajectory favors centerline
  -> no replan unless truly no feasible corridor
```

### 5.5 定位/TF 异常

```text
TF stale / odom jump / robot far from route
  -> LocalPlanStatus = TF_INVALID or ROUTE_DEVIATION
  -> Supervisor enters diagnostic/recovery path
  -> not classified as obstacle blockage
```

## 6. 参数设计

建议新增配置分组：

```yaml
local_obstacle_layer:
  update_frequency: 15.0
  frame_id: "map"
  robot_base_frame: "base_link"
  forward_range: 8.0
  backward_range: 2.0
  lateral_range: 4.0
  resolution: 0.10
  obstacle_persistence_sec: 0.8
  dynamic_prediction_horizon_sec: 2.0
  min_cluster_points: 5
  dynamic_velocity_threshold: 0.2
  unknown_obstacle_confidence_threshold: 0.4
  max_points_per_snapshot: 120000
  voxel_leaf_size: 0.08

ackermann_local_planner:
  controller_frequency: 15.0
  horizon_sec: 2.5
  horizon_distance: 4.0
  velocity_samples: [0.0, 0.08, 0.2, 0.3]
  curvature_samples: [-1.0, -0.5, 0.0, 0.5, 1.0]
  min_clearance: 0.25
  comfort_clearance: 0.8
  max_lateral_accel: 1.5
  max_steer: 0.238
  wheelbase: 0.46
  max_candidate_trajectories: 80

trajectory_costs:
  obstacle_distance_weight: 3.0
  path_deviation_weight: 1.0
  heading_error_weight: 0.8
  curvature_weight: 0.4
  speed_smoothness_weight: 0.6
  progress_weight: 1.5
  dynamic_obstacle_weight: 2.0

navigation_supervisor:
  collision_confirm_cycles: 5
  blocked_confirm_time: 1.0
  dynamic_wait_timeout: 3.0
  static_replan_timeout: 2.0
  min_replan_interval: 3.0
  route_deviation_confirm_cycles: 10
  odom_jump_distance_threshold: 0.5

global_planner_manager:
  initial_backend: "hybrid_a_star"
  replan_backend: "graph_a_star"
  allow_background_refine: true
  planner_timeout_sec: 5.0
  min_request_interval_sec: 3.0

runtime_limits:
  target_platform: "jetson_orin_nx"
  max_local_planning_time_ms: 50
  max_obstacle_update_time_ms: 50
  max_end_to_end_local_cycle_ms: 80
  max_candidate_trajectories: 80
  max_dynamic_obstacles: 48
  max_points_per_snapshot: 120000
  max_costmap_cells: 120000
  max_navigation_memory_mb: 1200
  debug_publish_default: false
  max_debug_publish_rate: 2.0
```

具体数值需要通过实车或 rosbag 调试确认，以上是设计起点。

## 7. 与现有代码的关系

### 7.1 保留部分

可以保留：

1. `p2p_move_base` action 接口。
2. `global_plan_manager` 的 action 通信框架。
3. `global_planner` 的 graph_a_star 和 hybrid_a_star 后端。
4. 现有 `trajectory_generators::AckermannSimpleTrajectoryGeneratorTheory` 的部分运动学逻辑。
5. 现有 `mpc_critics` 的部分代价模型思想。
6. RViz debug 发布机制。

### 7.2 重构部分

建议重构：

1. `local_planner` 内部从 RPP 主导改为 State Lattice 主导。
2. `CollisionModel` 从直接查询点云改为查询 `LocalObstacleLayer`。
3. `p2p_move_base` 的状态机加入 `LOCAL_AVOIDANCE`、`YIELD_WAIT`、`GLOBAL_REPLAN` 的明确区分。
4. `PATH_BLOCKED_REPLANNING` 不再由单次 collision 直接产生。
5. 全局规划请求增加 backend 选择和防抖。

### 7.3 兼容策略

建议保留运行时参数切换：

```yaml
local_planner:
  controller_backend: "ackermann_lattice"
  fallback_controller_backend: "regulated_pure_pursuit"
```

这样可以在调试期随时回退。

## 8. 逻辑完整性、内存与资源安全

本重构会引入局部地图、候选轨迹、动态障碍跟踪和多线程 ROS callback。设计上必须把安全性作为一等约束，避免出现越界访问、悬空引用、数据竞争、内存膨胀、旧数据误用和异常状态下继续行驶等问题。

### 8.1 逻辑完整性原则

所有模块必须遵守以下原则：

1. 单一职责：`LocalObstacleLayer` 只负责环境表达，`AckermannLocalPlanner` 只负责局部轨迹选择，`NavigationSupervisor` 只负责行为决策，`GlobalPlannerManager` 只负责全局规划请求管理。
2. 结构化状态：模块之间传递结构化结果，不使用字符串判断核心逻辑。
3. 显式时间戳：所有感知快照、轨迹、TF、odom、全局路径都必须携带时间戳。
4. 旧数据拒绝：超过时效的数据不得参与避障决策。
5. 失败优先安全：任何无法判断的状态默认停车或降级，不默认继续加速。
6. 重规划防抖：全局重规划必须经过连续确认，不能由单帧输入触发。
7. 决策可解释：每次停车、等待、绕障、重规划都必须有明确 reason code。

建议定义统一 reason code：

```cpp
enum class NavigationReason {
  None,
  LocalObstacleDetected,
  DynamicObstacleYield,
  StaticBlockageConfirmed,
  NoFeasibleLocalTrajectory,
  RouteDeviationConfirmed,
  TfStale,
  OdomJumpDetected,
  PerceptionStale,
  GlobalPlannerTimeout,
  GlobalPlannerFailed,
  RecoveryExceeded
};
```

禁止用日志字符串作为状态机输入。

### 8.2 内存安全原则

C++ 实现必须采用 RAII 和明确所有权，不允许裸指针表达对象生命周期。

要求：

1. 长生命周期对象使用值语义、`std::unique_ptr` 或 `std::shared_ptr`，所有权必须清晰。
2. callback 捕获 `this` 时必须保证对象生命周期，异步 callback 中优先使用 `std::weak_ptr` 锁定。
3. 不把局部变量地址传给异步任务、timer、action callback 或长期缓存。
4. 点云、栅格、轨迹数组访问必须做边界检查。
5. 所有 `size_t` 与 `int` 转换必须显式检查，避免负数转为巨大无符号数。
6. 对空指针、空点云、空路径、空轨迹集合必须有明确处理分支。
7. 不允许无界增长的历史缓存。所有缓存必须有最大时长和最大容量。
8. `pcl::PointCloud`、KD-tree、distance map snapshot 对外发布时使用不可变快照，避免读写同时发生。

示例约束：

```cpp
struct ObstacleHistoryConfig {
  double max_age_sec = 1.0;
  std::size_t max_clusters = 256;
  std::size_t max_points_per_snapshot = 200000;
};
```

当输入超过限制时，系统应降采样、裁剪或拒绝当前帧，并输出诊断，而不是继续分配无限内存。

### 8.3 并发与线程安全

ROS2 中点云 callback、odom callback、TF 查询、timer、action result 可能并发运行。重构后必须明确数据交换模型。

建议采用 snapshot 模型：

```text
writer callback
  -> 构建新 LocalObstacleSnapshot
  -> 原子替换 shared_ptr<const Snapshot>

planner loop
  -> 拷贝 shared_ptr<const Snapshot>
  -> 在整个控制周期只读该 snapshot
```

要求：

1. 高频规划循环不得长时间持有感知层写锁。
2. 不允许在持有多个 mutex 时调用外部 callback、ROS publish、action send。
3. mutex 加锁顺序必须固定，避免死锁。
4. 动态障碍 tracker 写入和局部规划读取必须通过快照隔离。
5. 全局规划结果回调只能请求状态切换，不能直接覆盖当前正在使用的局部规划数据。
6. 后台 hybrid refine 的结果必须带 `request_id`、`goal_seq`、`route_version`，过期结果必须丢弃。

建议统一锁顺序：

```text
route_mutex
  -> obstacle_snapshot_mutex
  -> planner_state_mutex
```

如果无法遵守固定顺序，应改为快照拷贝，避免嵌套锁。

### 8.4 边界与数值安全

轨迹评价和距离场计算必须处理异常数值。

要求：

1. 所有输入位姿、速度、曲率、距离必须检查 `std::isfinite`。
2. yaw、曲率、速度上限必须 clamp 到物理限制。
3. distance map 坐标转换必须检查栅格索引范围。
4. footprint 多边形顶点数量不足时拒绝轨迹。
5. 轨迹 horizon 为 0 或采样点为空时返回安全失败。
6. 代价函数不得产生 NaN/Inf；任何 NaN/Inf 代价直接视为不可行轨迹。
7. 时间差为负或过大时视为时钟异常，停车并输出诊断。

代价计算建议：

```cpp
if (!std::isfinite(cost)) {
  return TrajectoryScore::Rejected(RejectReason::InvalidNumericCost);
}
```

### 8.5 资源安全与实时性

避障不能因为输入异常或全局规划慢导致资源耗尽。

要求：

1. 局部 costmap 尺寸固定，不随障碍数量动态扩大。
2. 候选轨迹数量有上限。
3. 每个控制周期有最大计算预算。
4. 动态障碍数量有上限，低置信度障碍可合并或裁剪。
5. RViz debug 发布必须可关闭，且不得阻塞控制循环。
6. 全局规划 action 必须有超时和取消机制。
7. 日志必须 throttle，避免高频刷屏拖慢控制。
8. Jetson Orin NX 共机运行时，避障模块必须限制点云规模、栅格规模、候选轨迹数量和 debug 发布频率。
9. 全局规划不得与局部规划在同一高优先级 executor 中长时间争抢线程。

建议控制周期预算：

```yaml
runtime_limits:
  max_local_planning_time_ms: 50
  max_obstacle_update_time_ms: 50
  max_end_to_end_local_cycle_ms: 80
  max_candidate_trajectories: 80
  max_dynamic_obstacles: 48
  max_points_per_snapshot: 120000
  max_costmap_cells: 120000
  max_navigation_memory_mb: 1200
  max_debug_publish_rate: 2.0
```

超过预算时：

1. 优先保留低速停车轨迹。
2. 跳过非关键 debug。
3. 降低候选轨迹采样密度。
4. 必要时进入安全停车状态。

### 8.6 Jetson Orin NX 共机运行约束

Orin NX 上同时运行重定位、雷达驱动、IMU 驱动和导航时，避障设计必须采用“预算驱动”的实现方式。

#### 8.6.1 线程与 executor

建议：

1. 雷达/IMU 驱动和重定位优先级高于 debug 和后台精细规划。
2. 局部规划循环使用固定频率 timer，不允许被全局规划 action 长时间阻塞。
3. `LocalObstacleLayer` 更新和 `AckermannLocalPlanner` 控制循环可以分离 executor，但通过 snapshot 交换数据。
4. 后台 `hybrid_a_star` 精修使用低优先级线程或独立 callback group。
5. RViz debug 发布使用低频 timer，不在控制循环内直接构建大消息。

建议线程模型：

```text
executor_high_priority:
  - odom callback
  - TF update
  - local planner timer
  - safety stop timer

executor_normal:
  - local obstacle update
  - obstacle tracker
  - global planner action client

executor_low_priority:
  - debug publish
  - background hybrid refine
```

#### 8.6.2 点云与栅格预算

点云处理必须在进入避障层前做空间裁剪和体素降采样：

```text
先裁剪 ROI：
  x: [-2 m, +8 m]
  y: [-4 m, +4 m]
  z: 按车辆可碰撞高度范围

再体素降采样：
  leaf_size: 0.08-0.12 m

最后限制点数：
  max_points_per_snapshot: 120000
```

局部 costmap 不应超过固定栅格预算。例如：

```text
ROI: 10 m x 8 m
resolution: 0.10 m
cells: 100 x 80 = 8000
```

只有在确实需要更精细近场碰撞时，才对车体附近小范围使用 0.05 m 局部分辨率。禁止全局大范围使用高分辨率 costmap。

#### 8.6.3 候选轨迹预算

候选轨迹数量必须受限，并按场景自适应：

```text
正常跟踪：
  15-25 条候选轨迹

检测到近障碍：
  40-80 条候选轨迹

计算超时或 CPU 压力高：
  降回 15-25 条，并保留停车轨迹
```

禁止每周期生成数百条轨迹再全量做高精度碰撞检查。轨迹评价应分级：

1. 粗检查：快速 footprint / 栅格碰撞。
2. 中检查：distance map clearance。
3. 精检查：只对候选前几名做更密集采样。

#### 8.6.4 Debug 与日志预算

实车默认：

1. 不发布所有候选轨迹。
2. 不发布全量 rejected trajectories。
3. 不发布全量 costmap 点云。
4. 只发布 selected trajectory、状态、关键 reason code。
5. debug topic 有订阅者且显式开启时才构建消息。

日志必须 throttle。高频循环中禁止每周期输出 warning。

#### 8.6.5 运行时预算监控

建议增加轻量预算监控：

```cpp
struct RuntimeBudgetStats {
  double obstacle_update_ms;
  double local_planning_ms;
  double local_cycle_ms;
  std::size_t snapshot_points;
  std::size_t costmap_cells;
  std::size_t candidate_count;
  std::size_t dynamic_obstacle_count;
  double process_rss_mb;
  bool budget_exceeded;
};
```

当连续超预算时：

```text
level 1:
  降低 debug 发布频率

level 2:
  降低候选轨迹数量
  降低动态障碍预测频率

level 3:
  降低 obstacle layer 更新频率
  只保留硬碰撞与低速轨迹

level 4:
  安全停车并报告 resource_budget_exceeded
```

### 8.7 故障隔离与降级

任何单一模块异常不应导致车辆继续危险行驶。

降级策略：

1. `LocalObstacleLayer` 过期：局部规划器只能输出减速或停车。
2. `ObstacleTracker` 失败：退化为静态障碍 costmap，不做动态预测。
3. distance map 生成失败：保留 footprint 硬碰撞检查。
4. State Lattice 无可行轨迹：停车并交给 Supervisor 判断等待或重规划。
5. 快速全局规划失败：保持停车或尝试恢复，不直接清空当前安全路线。
6. 后台精细规划失败：不影响当前已可执行路线。
7. Orin NX 资源预算连续超限：降低采样和 debug；仍超限则安全停车。

### 8.8 安全验收标准

进入实车测试前必须满足：

1. ASan/UBSan 或等价工具下核心单元测试无内存错误。
2. ThreadSanitizer 或人工并发审查覆盖关键 snapshot/lock 路径。
3. 空点云、空路径、空轨迹、TF stale、NaN odom 输入均不会崩溃。
4. rosbag 高频点云回放不会导致内存持续增长。
5. 全局规划超时不会阻塞局部安全停车。
6. 后台规划旧结果不会覆盖新路线。
7. debug topic 无订阅者时不做昂贵序列化。
8. 在 Jetson Orin NX 上与重定位、雷达驱动、IMU 驱动同时运行时，局部规划周期不持续超预算。
9. 长时间运行时 RSS 内存稳定，不能出现持续增长。

## 9. 日志与可视化

必须增加高质量诊断，否则避障调参会非常困难。

### 9.1 日志

建议每次状态切换输出：

```text
navigation supervisor transition:
  from=TRACK_GLOBAL_PATH
  to=LOCAL_AVOIDANCE
  reason=current_track_collision
  min_clearance=0.32
  feasible_trajectories=8
  blocked_duration=0.20
```

全局重规划输出：

```text
global replan requested:
  reason=static_blocked_confirmed
  blocked_duration=2.35
  local_feasible_count=0
  backend=graph_a_star
  last_replan_age=5.20
```

误触发分析需要这些字段：

1. 障碍来源：static / lidar / dynamic_prediction。
2. 障碍置信度。
3. 最近障碍距离。
4. 候选轨迹数量。
5. 碰撞拒绝数量。
6. 动态风险拒绝数量。
7. 是否 TF/odom 异常。
8. 是否 route deviation。
9. reason code。
10. snapshot age。
11. local planning time。
12. memory/resource limit 是否触发。

### 9.2 RViz Topic

建议发布：

```text
/local_obstacle_layer/costmap
/local_obstacle_layer/distance_map
/local_obstacle_layer/obstacle_clusters
/local_obstacle_layer/predicted_obstacles
/local_planner/candidate_trajectories
/local_planner/selected_trajectory
/local_planner/rejected_trajectories
/local_planner/trajectory_scores
/navigation_supervisor/state
/navigation_supervisor/replan_events
```

候选轨迹可用颜色区分：

1. 绿色：可行轨迹。
2. 蓝色：最终选择轨迹。
3. 红色：碰撞拒绝。
4. 黄色：动态风险高。
5. 灰色：路径偏差过大。

## 10. 分阶段实施计划

### 阶段 1：诊断与防抖

目标：先减少误触发全局规划。

工作：

1. 梳理所有 `route_refresh` 入口。
2. 增加重规划原因分类。
3. 给 `PATH_BLOCKED_REPLANNING` 增加连续确认。
4. 将单次 collision 改为短时 blocked wait 或 local avoidance candidate。
5. 增加 `min_replan_interval`。
6. 增加日志字段。
7. 增加 runtime budget 统计字段。

验收：

1. 单次点云误检不触发全局规划。
2. 日志能明确区分 collision、route_deviation、oscillation、TF stale。
3. 全局规划请求频率明显下降。
4. 所有重规划入口都有结构化 reason code。
5. 空路径、空点云、TF stale 不会导致崩溃或继续危险行驶。
6. Orin NX 上与重定位和驱动共机运行时能输出局部周期耗时、点数和内存统计。

### 阶段 2：LocalObstacleLayer

目标：把障碍表达从“直接点云查询”提升为稳定环境模型。

工作：

1. 新增 LocalObstacleLayer 类。
2. 接入局部点云和静态地图层。
3. 实现障碍时序缓存。
4. 实现 footprint collision query。
5. 实现 distance map。
6. 提供 RViz 可视化。
7. 实现固定容量缓存和输入裁剪。
8. 实现不可变 snapshot 输出。
9. 实现 ROI 裁剪、体素降采样和最大点数限制。

验收：

1. 点云闪烁不会导致障碍状态剧烈跳变。
2. 车体 footprint 碰撞检测结果与 RViz 可视化一致。
3. distance map 能正确表达窄通道障碍距离。
4. 高频 rosbag 回放时内存不持续增长。
5. 并发读取 snapshot 时无数据竞争。
6. 输入大点云时不会超过 `max_points_per_snapshot` 和内存预算。

### 阶段 3：Ackermann State Lattice 局部规划

目标：让机器人具备主动局部绕障能力。

工作：

1. 设计 lattice motion primitive。
2. 生成多候选轨迹。
3. 接入 LocalObstacleLayer 碰撞查询。
4. 实现代价函数。
5. 输出最优轨迹和 cmd_vel。
6. 与现有 RPP 保持可切换。
7. 对所有候选轨迹做 NaN/Inf 和边界检查。
8. 增加每周期最大计算预算。
9. 实现候选轨迹数量自适应降级。

验收：

1. 前方局部障碍出现时，能选择左绕/右绕候选。
2. 窄通道中能选择中间安全轨迹。
3. 无可行轨迹时能稳定停车。
4. 不再依赖单条 RPP 轨迹决定是否全局重规划。
5. 任意异常轨迹输入只会被拒绝，不会导致崩溃。
6. 超过计算预算时能降级到安全停车或低速轨迹。
7. Orin NX 共机运行时，局部规划周期在目标预算内，无持续超时。

### 阶段 4：NavigationSupervisor 重构

目标：把局部避障、等待、重规划、恢复的决策从局部规划器中解耦。

工作：

1. 新增明确状态：`LOCAL_AVOIDANCE`、`YIELD_WAIT`、`GLOBAL_REPLAN`。
2. 实现动态障碍等待策略。
3. 实现静态阻塞确认策略。
4. 实现 route deviation 防抖。
5. 实现 replan request throttling。

验收：

1. 临时障碍不会立即全局重规划。
2. 静态障碍持续阻塞后会全局重规划。
3. 定位/TF 异常不被误报为障碍阻塞。
4. 所有状态切换都有合法来源和目标。
5. 旧全局规划结果不会覆盖当前路线。

### 阶段 5：全局规划两级化

目标：降低全局规划对实时性的影响。

工作：

1. 支持按 trigger reason 选择 planner backend。
2. 避障失败重规划默认使用 `graph_a_star`。
3. 初始规划或后台精修使用 `hybrid_a_star`。
4. 增加规划超时和取消策略。
5. 处理后台精修结果的安全切换。
6. 后台精修使用低优先级执行，不抢占局部控制循环。

验收：

1. 避障失败后的快速重规划时间明显下降。
2. 慢速 hybrid 规划不会阻塞局部控制。
3. stale planner result 不会覆盖当前路径。
4. 全局规划超时和取消不会泄漏 action handle 或残留活动请求。
5. Orin NX 上触发全局规划时，局部安全停车仍按周期运行。

### 阶段 6：动态障碍跟踪与预测

目标：提升人、车等动态障碍场景的行为质量。

工作：

1. 聚类局部障碍。
2. 跟踪障碍速度。
3. 预测短期占据。
4. 将预测结果加入 trajectory risk cost。
5. 区分等待和绕行策略。

验收：

1. 横穿障碍优先等待。
2. 静止障碍持续存在时触发重规划。
3. 车辆不朝预测碰撞区域加速。
4. tracker 丢失目标时能安全退化为静态/未知障碍。
5. 动态障碍缓存数量受限，不会无限增长。

## 11. 测试方案

### 11.1 单元测试

1. footprint collision query。
2. distance map 生成。
3. 障碍时序衰减。
4. trajectory cost 计算。
5. NavigationSupervisor 状态切换。
6. replan throttle。
7. 空点云、空路径、空轨迹输入。
8. NaN/Inf 位姿、速度、代价输入。
9. 栅格坐标越界访问保护。
10. snapshot 生命周期与不可变读取。
11. request_id / route_version 过期结果丢弃。
12. 缓存容量和时间裁剪。

### 11.2 安全工具测试

建议在实现后加入：

1. AddressSanitizer：检查越界、use-after-free、double free。
2. UndefinedBehaviorSanitizer：检查未定义行为、整数溢出等。
3. ThreadSanitizer 或并发压力测试：检查 snapshot、callback、planner loop 数据竞争。
4. 长时间 rosbag 回放：检查 RSS 内存是否稳定。
5. 高频 debug 开关测试：确认无订阅者时不做昂贵发布。

### 11.3 Orin NX 共机预算测试

必须在 Jetson Orin NX 上做共机测试：

运行模块：

1. 雷达驱动。
2. IMU 驱动。
3. 重定位或 LIO-SAM localization。
4. p2p_move_base。
5. global_planner。
6. LocalObstacleLayer。
7. AckermannLocalPlanner。

测试要求：

1. 连续运行不少于 30 分钟。
2. 记录 CPU、RSS 内存、局部规划周期耗时、障碍层更新时间、点云 snapshot 点数。
3. 开启一次全局快速规划，确认局部安全停车不被阻塞。
4. 开启和关闭 debug topic 各测试一次。
5. 触发预算降级，确认系统能降低候选轨迹数或 debug 频率。

验收指标：

1. 局部规划周期不持续超过 `max_end_to_end_local_cycle_ms`。
2. RSS 内存无持续线性增长。
3. 雷达/IMU/重定位 topic 频率不因导航 debug 明显掉频。
4. 预算超限时系统降级，不崩溃、不继续危险行驶。

### 11.4 仿真测试

场景：

1. 直道临时障碍。
2. 窄通道。
3. 前方静态障碍完全挡路。
4. 动态障碍横穿。
5. 点云短时闪烁。
6. 定位跳变。
7. 全局规划超时。

指标：

1. 是否触发全局重规划。
2. 触发原因是否正确。
3. 局部规划是否持续输出安全速度。
4. 最小障碍距离。
5. 到达时间。
6. 停车次数。
7. 重规划次数。
8. 局部规划周期耗时。
9. 内存占用曲线。
10. stale data 丢弃次数。
11. 预算降级触发次数。

### 11.5 实车测试

建议按风险从低到高：

1. 空场无障碍跟踪。
2. 静态纸箱绕行。
3. 窄通道通过。
4. 人缓慢横穿。
5. 静态障碍完全挡路。
6. 定位弱场景。

每次测试必须保存 rosbag 和日志，用于回放调参。

## 12. 风险与应对

### 12.1 计算量增加

风险：State Lattice + distance map 增加 CPU 开销。

应对：

1. 限制局部地图范围。
2. 控制候选轨迹数量。
3. 缓存 distance map。
4. 分离高频碰撞检查和低频动态跟踪。

### 12.2 轨迹抖动

风险：左右绕行代价接近时，轨迹可能来回切换。

应对：

1. 加 selected trajectory hysteresis。
2. 对上一周期选择轨迹加连续性奖励。
3. 对频繁换边加惩罚。

### 12.3 过度保守

风险：安全距离和障碍膨胀过大导致窄通道无法通过。

应对：

1. 区分 hard collision margin 和 comfort clearance。
2. 窄通道中允许较低速度通过较小 comfort clearance。
3. 用 footprint 精确碰撞替代过度圆形膨胀。

### 12.4 动态障碍误分类

风险：静态障碍被误判为动态，导致等待过久。

应对：

1. 设置最大动态等待时间。
2. 静止时间超过阈值转为静态阻塞。
3. 保留人工可调参数。

### 12.5 全局快速路径不满足 Ackermann

风险：`graph_a_star` 路径几何可达，但不满足车辆转弯半径。

应对：

1. 局部 lattice 负责短期 Ackermann 可行性。
2. graph path smoothing 限制曲率。
3. 必要时后台 hybrid refine。
4. 快速路径不可跟踪时再请求精细规划。

### 12.6 内存或资源泄漏

风险：点云历史、障碍 tracker、debug 消息、action 请求长期积累，导致内存持续增长。

应对：

1. 所有缓存设置最大容量和最大时间窗口。
2. action 请求完成、取消、超时后必须释放 handle。
3. debug 消息只在有订阅者或启用诊断时构建。
4. 长时间 rosbag 回放作为验收项。

### 12.7 数据竞争或死锁

风险：感知 callback、规划循环、action result 同时访问路线、障碍和状态。

应对：

1. 采用不可变 snapshot 模型。
2. 固定 mutex 加锁顺序。
3. 不在持锁状态下调用 ROS publish 或 action send。
4. 对关键路径做 ThreadSanitizer 或并发压力测试。

### 12.8 异常输入导致危险输出

风险：NaN 位姿、空路径、TF 时间跳变、栅格索引越界导致错误速度输出。

应对：

1. 所有输入先验证再进入规划。
2. 非法输入统一返回安全失败状态。
3. Supervisor 对非法输入默认停车并记录 reason code。

## 13. 推荐落地顺序

推荐不要一次性全量替换。更稳妥的顺序：

1. 先实现诊断和重规划防抖。
2. 同时建立内存、并发、边界输入的测试框架。
3. 再实现 LocalObstacleLayer。
4. 再实现 State Lattice 局部规划器。
5. 再重构 NavigationSupervisor。
6. 最后做全局规划两级化和动态障碍预测。

这样每一步都可以单独验证，不会一次性引入太多不确定性。

## 14. 最终结论

最适合该 Ackermann 地面机器人的避障重构方案是：

```text
LocalObstacleLayer
  + Ackermann State Lattice Local Planner
  + NavigationSupervisor Replan Debounce
  + Fast/Refined Two-stage Global Planner
```

该方案的重点不是让全局规划器频繁承担避障职责，而是让局部规划器真正具备地面车可行的实时绕障能力。全局规划只处理局部确认无解的场景。

预期改进：

1. 临时障碍不再频繁触发全局规划。
2. 车辆能主动局部绕障。
3. 窄通道误判减少。
4. 全局重规划频率下降。
5. 全局规划慢对驾驶体验的影响降低。
6. 每次停车、等待、重规划都有明确可追踪原因。
