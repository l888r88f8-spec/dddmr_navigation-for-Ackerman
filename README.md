# dddmr_navigation-for-Ackerman

本仓库是基于[dddmr_navigation](https://github.com/dfl-rlab/dddmr_navigation)改造的 Ackermann 车型三维导航包，适配了hybrid_a_star+RPP。

## 环境要求

推荐环境：

- Ubuntu 22.04 / Ubuntu 24.04
- ROS2 Humble / ROS2 Jazzy
- C++17 编译器
- `colcon`、`ament_cmake`
- PCL、OpenCV、Eigen
- `gtsam`、`cloud_msgs`、`cv_bridge`、`image_geometry`、`libpcl-all` 等依赖。

## TF 树要求

```text
map
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

## Web 控制台

新增 `dddmr_web_control` 网页控制台，可在浏览器中查看 3D 地图、建图状态、系统资源占用，并下发初始位姿和导航目标点。

启动网页控制台：

```bash
ros2 launch dddmr_web_control web_control.launch.py use_sim:=false
```

仿真或 rosbag 环境可以统一：

```bash
ros2 launch dddmr_web_control web_control.launch.py use_sim:=true
```

启动后在浏览器访问：

```text
http://<机器人IP>:8080
```

### Web 控制台话题

导航页面默认订阅：

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/lio_sam/localization/global_map` | `sensor_msgs/msg/PointCloud2` | 定位全局地图 |
| `mapcloud` | `sensor_msgs/msg/PointCloud2` | 导航点云层 |
| `/weighted_ground` | `sensor_msgs/msg/PointCloud2` | 可选目标点的地面点云 |
| `/lio_sam/localization/odometry` | `nav_msgs/msg/Odometry` | 小车实时位姿 |
| `/global_path` | `nav_msgs/msg/Path` | 规划路径，默认由 `path_topics` 指定 |
| `/p2p_move_base/_action/status` | `action_msgs/msg/GoalStatusArray` | 导航状态，用于到达目标后自动清除目标箭头和路径 |

建图页面默认订阅：

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/lio_sam/mapping/map_global` | `sensor_msgs/msg/PointCloud2` | 建图全局地图 |
| `/lio_sam/mapping/ground_cloud_global` | `sensor_msgs/msg/PointCloud2` | 建图地面点 |
| `/lio_sam/mapping/path` | `nav_msgs/msg/Path` | 建图路径 |
| `/lio_sam/mapping/odometry` | `nav_msgs/msg/Odometry` | 建图里程计位姿 |

网页中的 `Start` / `Stop` 按钮会由 `dddmr_web_control` 节点统一管理以下 launch：

| 目标 | 启动命令 |
| --- | --- |
| Localization | `ros2 launch lio_sam_hesai localization_with_nonground.launch.py use_sim:=<use_sim> use_rviz:=false` |
| Navigation | `ros2 launch p2p_move_base p2p_move_base_localization.launch.py use_sim:=<use_sim>` |
| Mapping | `ros2 launch lio_sam_hesai mapping.launch.py use_sim:=<use_sim> use_rviz:=false` |

## Ackermann 相关参数

主要参数在：

```text
src/dddmr_p2p_move_base/config/p2p_move_base_localization.yaml
```

`max_steer` 与最小转弯半径相关，常用关系：

```text
max_steer = atan(wheelbase / min_turning_radius)
max_angular_velocity ~= max_linear_speed / min_turning_radius
```

## 更新日志

### 2026-05-29
- 适配Ubuntu 24.04相关编译环境，调整部分包依赖和构建配置。

### 2026-05-24
- 新增`dddmr_web_control`网页控制台。
- 支持浏览器查看3D地图、导航状态、建图状态和系统资源占用。
- 支持通过网页下发初始位姿、导航目标点，并统一管理定位、导航、建图launch。

### 2026-05-08
- 启用全局避障能力。
- 调整局部规划器与多层感知层的障碍物输入逻辑。

### 2026-04-27
- 增强RPP局部控制器的避障能力。
- 调整Ackermann轨迹生成器和碰撞模型，提升局部避障稳定性。

### 2026-04-21
- 新增Regulated Pure Pursuit（RPP）局部控制后端。

### 2026-04-20
- 新增Forward Hybrid A*全局规划器。

### 2026-04-10
- 基于`dddmr_navigation`适配Ackermann车型导航。
- 新增Ackermann轨迹生成器、前进速度偏好critic和P2P本地化launch。

## 致谢
- 感谢dfl-rlab的开源。
