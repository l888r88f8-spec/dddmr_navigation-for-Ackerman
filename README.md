# dddmr_navigation-for-Ackerman

本仓库是基于[dddmr_navigation](https://github.com/dfl-rlab/dddmr_navigation)改造的 Ackermann 车型三维导航包，适配了hybrid_a_star+RPP。

## MLS 测量证据与独立 PCD 地形（2026-11-10）

本仓库的 `grid_global_planner` 新增了可独立于在线雷达/定位运行的静态
PCD 地形入口。数据流是：

```text
原始真实 XYZ 点
  → 去重/排序后的 canonical 测量点表
  → 每格每层 MeasurementPatch（保留 base/own/attempt 真实索引）
  → 通过门限的 TerrainNode + 同层连接 + 独立静态障碍记录
  → 不可变 MapSnapshot
  → WheelSupportEvaluator / AckermannHybridAStar / PathValidator
  → RViz 地形面 + 网格边 + 灰色测量证据线框
```

关键边界：

- 唯一离线 YAML 使用 fine `0.20 m`、实测邻域 `R=0.35 m`。一次有界同层扩展
  使用 `evidence_expansion_step_m=0.10`，即实际半径 `0.45 m`。`0.55 m` 未采用：
  它超过 `R + max_sampling_gap/2`，也没有救回 20,-140 / 27,-130。
- 扩展只允许在证据不足（点数、采样间隙、支撑凸包）时进行一次；RMS、内点、
  退化、局部凹陷、坡度失败绝不靠放大邻域“变绿”。20,-140（高 RMS）和
  27,-130（本格凹陷）保持 unknown/rejected，不产生 supported upper node。
- `14,-128` 的 3 个 base 点经真实 `0.45 m` 环恢复为 9 点、z≈`1.352526 m`、
  RMS≈`0.0466126 m`。其最大正残差约 `0.075658 m`，小于现有
  `ground_duplicate_tolerance_m=0.08`，因此**不**制造 occupied；
  14,-130 正控制保持 `0.35 m` 五点拟合（z≈`1.369639 m`）。
- 灰色测量线框只画真实 own-cell 观测的 rejected/observed patch 的实际
  own Z 范围，不是拟合平面、不是邻域范围、也不代表可通行。supported 面
  同样只是几何证据；GroundMap-only 不推断完整自由体积、天花板厚度或净空。
- 静态障碍占用放在真实测量点的 XY；不会沿拟合半径扩散，也不会把无关楼层
  设为 occupied。显式 obstacle PCD 仍使用原有最近表面关联和物理阈值。
- RViz 二维选点不能选择楼层；`base_link` 参考高度由现有 bridge 的固定 Z
  （默认 `-0.5`，与显示 TF 一致）决定。多层入口必须显式设置目标楼层高度，
  bridge 不从地形自动推断。`base_link` 底面相对轮胎接地点为
  `base_contact_height_m=0.22 m`；这是现有模型约定，不是新的车辆能力声明。

构建和 CPU 验收（不启动 ROS/RViz）：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select grid_global_planner \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ctest --test-dir build/grid_global_planner --output-on-failure

# 当前精确 GroundMap + YAML 0.20/R.35/.45 profile；默认空障碍数组
./build/grid_global_planner/offline_static_pcd_smoke \
  /home/robot/map/hesai_sim2/GroundMap.pcd
```

真实地图验收会校验 GroundMap SHA-256
`19bbfbf01f4c9b6fe6e07c02dc1fef105b6ec4f607f4305e30d16cd4d03246d7`，
打印四格证据、全图 metrics、固定正/负整条路径、RSS 和构建时间。该 smoke
只声明 CPU 地图连通性/运动学校验，不声明真实四轮接触、悬挂、净空或实车安全。

设计思想和 API 参考了以下公开项目，但**没有复制上游源码，也没有新增依赖**：
[slam-maps](https://github.com/dfki-ric/slam-maps)、
[traversability_generator3d](https://github.com/dfki-ric/traversability_generator3d)、
[PCT_planner](https://github.com/byangw/PCT_planner)、
[vox_nav](https://github.com/NMBURobotics/vox_nav)。更详细的中文说明见
`src/dddmr_grid_global_planner/docs/offline_planning.md` 和
`src/dddmr_grid_global_planner/docs/implementation_status.md`。

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
    └── lidar_link
    └── imu_link
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

## 编译（Release + CUDA/CPU）

在本仓库内构建独立 overlay。必须同时构建 `p2p_move_base`，否则新 overlay
只会更新库，不会安装两份已设为 `compute_backend: cuda` 的 YAML，导航仍会
沿用旧安装里的 CPU 默认配置。

```bash
cd /home/robot/navigation_ws/src/dddmr_navigation-for-Ackerman
source /opt/ros/humble/setup.bash
source /home/robot/navigation_ws/install/setup.bash

colcon --log-base "$PWD/log/mppi_cuda_integration/colcon" build \
  --base-paths src/dddmr_local_map \
               src/dddmr_local_planner/local_planner \
               src/dddmr_p2p_move_base \
  --build-base "$PWD/log/mppi_cuda_integration/build" \
  --install-base "$PWD/log/mppi_cuda_integration/install" \
  --packages-select dddmr_local_map local_planner p2p_move_base \
  --parallel-workers 4 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
               -DBUILD_TESTING=OFF \
               -DMPPI_ENABLE_CUDA=ON

source log/mppi_cuda_integration/install/setup.bash
```

CUDA 后端按 `CMAKE_CUDA_ARCHITECTURES` 编译（默认 87 / Orin），使用双精度、
`--ftz=false`、精确 div/sqrt，不启用 fast math。两份配置
`src/dddmr_p2p_move_base/config/p2p_move_base_localization.yaml` 与
`p2p_move_base_grid_localization.yaml` 已经写入 `compute_backend: cuda`。

后端参数在节点初始化时读取，修改后重启生效。在实际加载的 YAML 中将
`compute_backend` 设为 `cpu` 即可使用 CPU，已有 CUDA 构建无需重编译。
如果修改仓库内的源 YAML，请重新构建 `p2p_move_base` 以安装配置。

以下是可选的 CPU-only 构建；启动时仍需显式加载 `compute_backend: cpu` 的配置：

```bash
source /opt/ros/humble/setup.bash
source /home/robot/navigation_ws/install/setup.bash
source "$PWD/log/mppi_cuda_integration/install/setup.bash"

colcon --log-base "$PWD/log/mppi_cuda_integration_cpu/colcon" build \
  --base-paths src/dddmr_local_planner/local_planner \
  --build-base "$PWD/log/mppi_cuda_integration_cpu/build" \
  --install-base "$PWD/log/mppi_cuda_integration_cpu/install" \
  --packages-select local_planner --parallel-workers 4 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
               -DBUILD_TESTING=OFF \
               -DMPPI_ENABLE_CUDA=OFF

source log/mppi_cuda_integration_cpu/install/setup.bash
```

CUDA 配置但设备/内核不可用时，`MppiCudaScoringError` 会显式报错、清零命令并
终止本周期，不会静默回退 CPU。

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

### 2026-11-10

- 增加 MLS 测量证据层、一次有界同层恢复、灰色 rejected/observed 线框、
  真实 PCD 四格与固定整条路径 CPU 验收、唯一 YAML 新键和中文说明。
- 保持 fine `0.20 m`、`R=0.35 m`、车辆几何和既有物理门限不变；新增扩展实际
  半径 `0.45 m`，`0.55 m` 未采用。

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
