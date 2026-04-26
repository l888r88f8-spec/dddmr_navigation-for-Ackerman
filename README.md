# dddmr_navigation-for-Ackerman

本仓库是基于[dddmr_navigation](https://github.com/dfl-rlab/dddmr_navigation)改造的 Ackermann 车型三维导航包，适配了hybrid_a_star+RPP。

该项目面向已经有定位/SLAM 输出的车辆：外部系统提供机器人在 `map` 下的位姿、里程计和去地点云，本导航栈读取静态 3D 地图，完成全局规划、局部避障和速度指令输出。

## 功能组成

主启动文件 `src/dddmr_p2p_move_base/launch/p2p_move_base_localization.launch.py` 会启动以下节点：

| 节点 | 包/可执行文件 | 作用 |
| --- | --- | --- |
| `pcl_publisher` | `mcl_3dl/pcl_publisher` | 读取 `GlobalMap.pcd` 和 `GroundMap.pcd`，发布静态地图点云 |
| `global_planner` | `global_planner/global_planner_node` | 加载全局感知层，提供 `/get_plan` 全局规划 action |
| `p2p_move_base` | `p2p_move_base/p2p_move_base_node` | 运行 P2P 状态机、局部规划器、轨迹生成器、MPC critics 和恢复行为 |
| `clicked2p2p` | `p2p_move_base/clicked2goal.py` | 将 RViz 的目标点/目标位姿转成 `/p2p_move_base` action |
| `rviz2` | `rviz2` | 使用包内 RViz 配置进行可视化和目标下发 |

## 环境要求

推荐环境：

- Ubuntu 22.04
- ROS 2 Humble
- C++17 编译器
- `colcon`、`ament_cmake`
- PCL、OpenCV、Eigen
- `gtsam`、`cloud_msgs`、`cv_bridge`、`image_geometry`、`libpcl-all` 等依赖。

## 地图要求

导航启动时需要两份 PCD 地图：

| 文件 | 默认路径 | 用途 |
| --- | --- | --- |
| 全局地图 | `/home/robot/map/hesai_sim/GlobalMap.pcd` | 静态障碍/环境点云 |
| 地面地图 | `/home/robot/map/hesai_sim/GroundMap.pcd` | 可通行地面点云，全局规划主要在该点云上建图搜索 |

启动时可以通过launch参数替换。

## TF 树要求

- 全局坐标系：`map`
- 机器人底盘坐标系：`base_link`
- 传感器坐标系：由点云 `header.frame_id` 决定，必须能通过 TF 变换到 `base_link` 和 `map`

典型实车 TF 树：

```text
map
└── odom
    └── base_link
        └── lidar_link / laser_link / hesai_link
```

## 外部话题要求

启动主导航前，车辆定位/SLAM/感知侧至少需要提供：

| 话题 | 类型 | 用途 | 默认配置位置 |
| --- | --- | --- | --- |
| `/tf`、`/tf_static` | `tf2_msgs/msg/TFMessage` | 提供 `map` 到 `base_link`，以及 `base_link` 到传感器的 TF | 全局要求 |
| `/localization/odometry` | `nav_msgs/msg/Odometry` | 里程计话题 | `local_planner.odom_topic` |
| `/cloud_nonground` | `sensor_msgs/msg/PointCloud2` | 局部/全局动态障碍输入点云 | `perception_3d_local.lidar.topic`、`perception_3d_global.lidar.topic` |

如果你的定位或点云话题名称不同，修改：

```text
src/dddmr_p2p_move_base/config/p2p_move_base_localization.yaml
```

## 对外输出

可通过rviz2查看。

## 编译

```bash
cd /home/robot/nav26_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 启动方式

```bash
cd ~/nav26_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch p2p_move_base p2p_move_base_localization.launch.py \
  map_dir:=/home/robot/map/hesai_sim/GlobalMap.pcd \
  ground_dir:=/home/robot/map/hesai_sim/GroundMap.pcd \
  use_sim:=false
```

播放rosbag或仿真时：

```bash
ros2 launch p2p_move_base p2p_move_base_localization.launch.py \
  map_dir:=/home/robot/map/hesai_sim/GlobalMap.pcd \
  ground_dir:=/home/robot/map/hesai_sim/GroundMap.pcd \
  use_sim:=true
```

## 下发目标

RViz 中可以使用两种方式下发目标：

- 发布 `geometry_msgs/msg/PoseStamped` 到 `/goal_pose_3d`，适合需要目标朝向的 Ackermann 车辆。
- 使用 Publish Point 发布 `geometry_msgs/msg/PointStamped` 到 `/clicked_point`，脚本会自动转换成 `/p2p_move_base` action，目标朝向默认 `w=1.0`。

也可以直接调用 action：

```bash
ros2 action send_goal /p2p_move_base dddmr_sys_core/action/PToPMoveBase \
"{target_pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

## Ackermann 相关参数

主要参数在：

```text
src/dddmr_p2p_move_base/config/p2p_move_base_localization.yaml
```

| 参数 | 当前默认值 | 说明 |
| --- | --- | --- |
| `global_planner.raw_route_backend` | `hybrid_a_star` | 使用 Hybrid A* 生成符合 Ackermann 约束的全局路径 |
| `global_planner.hybrid_astar.wheelbase` | `0.46` | 车辆轴距，必须按实车修改 |
| `global_planner.hybrid_astar.max_steer` | `0.238` | 最大前轮转角 |
| `local_planner.controller_backend` | `regulated_pure_pursuit` | 当前局部控制后端 |
| `local_planner.regulated_pure_pursuit.wheelbase` | `0.46` | 局部控制使用的轴距 |
| `local_planner.regulated_pure_pursuit.max_steer` | `0.238` | 局部控制最大转角 |
| `local_planner.regulated_pure_pursuit.max_angular_velocity` | `0.526` | 输出角速度上限 |
| `trajectory_generators.ackermann_simple.max_vel_x` | `1.0` | 轨迹采样最大前进速度 |
| `trajectory_generators.ackermann_simple.max_vel_theta` | `0.526` | 轨迹采样角速度上限 |
| `local_planner.cuboid` | 车体 8 个角点 | 车辆碰撞包络，必须按实车尺寸修改 |

`max_steer` 与最小转弯半径相关，常用关系：

```text
max_steer = atan(wheelbase / min_turning_radius)
max_angular_velocity ~= max_linear_speed / min_turning_radius
```

## 目录说明

```text
src/dddmr_p2p_move_base/
  launch/p2p_move_base_localization.launch.py    # 主启动文件
  config/p2p_move_base_localization.yaml         # Ackermann 导航主参数
  rviz/p2p_move_base_localization.rviz           # RViz 配置

src/dddmr_global_planner/                        # 全局规划，含 Hybrid A*
src/dddmr_perception_3d/                         # 3D 静态/动态感知层
src/dddmr_local_planner/                         # 局部规划、轨迹生成、critics、恢复行为
src/dddmr_mcl_3dl/                               # 地图 PCD 发布及 MCL 相关工具
src/dddmr_sys_core/                              # 自定义 action 和状态定义
```

## 致谢
- 感谢dfl-rlab的开源。