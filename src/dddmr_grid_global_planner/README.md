# dddmr_grid_global_planner

> **2026-11 小范围回退（当前默认）**：用户实测完整 GlobalMap 方案明显差于
> 原地面点云方案，默认已恢复 `offline.global_cloud_mode=false`，只从
> `GroundMap.pcd` 构建地面；`obstacle_pcd` 默认同 GroundMap，同一路径不重复
> 加载。完整 GlobalMap 仅保留为显式可选障碍输入，不用于重建地面；多尺度/
> 最低层确认等 global-only 参数保留但默认不启用。
>
> 当前启动：
> ```bash
> source /opt/ros/humble/setup.bash
> source /home/robot/navigation_ws/install/setup.bash
> ros2 launch dddmr_grid_global_planner path_planning_on_static_layer.launch.py \
>   use_sim:=false
> ```
> 所有 PCD 路径统一在 YAML 顶部配置：launch 将 `offline.global_pcd` 和
> `offline.ground_pcd` 注入 `pcl_publisher.map_dir`/`ground_dir` 供显示；
> 规划器直接读取 `offline.global_pcd`、`offline.ground_pcd`、
> `offline.obstacle_pcd`，其中 `obstacle_pcd` 保持独立规划语义。

## 0. 最终集成验收（2026-11-07）

TASK2–TASK4 已集成到当前 live 包，TASK2 的 atan2/精确支撑边界、TASK3 的
有界鲁棒拟合和 TASK4 的分离证据分类均保留。最终验收使用全新的隔离构建：

- 普通 Release：`log/multilayer_terrain_fix/final/build_regular` /
  `install_regular`；`BUILD_TESTING=ON`、`DDDMR_GRID_BUILD_TESTS=ON`，
  `DDDMR_GRID_BUILD_BENCHMARKS=ON`（只编译 benchmark，未运行完整 benchmark）。
- 实际 symlink Release：`log/multilayer_terrain_fix/final/build_symlink` /
  `install_symlink`；`AMENT_CMAKE_SYMLINK_INSTALL=ON`、`BUILD_TESTING=OFF`、
  `DDDMR_GRID_BUILD_TESTS=OFF`。
- 最终普通构建完整注册 CTest：**27/27 passed**，含 19 个 gtest、5 个 pytest
  和 `uncrustify`/`cpplint`/`flake8` 三个 ROS linter。证据
  `log/multilayer_terrain_fix/final/ctest_full_after_patch.log`。
- 普通和 symlink 安装均 exit 0；节点、库、头文件、唯一 launch 和唯一 YAML
  均存在；symlink 库/节点/头文件/launch/YAML 为真实符号链接；公开
  `MeasuredSupportBoundaryDistanceM`、`InsufficientConnectionDistanceM`、
  `SupportBoundaryClassificationAt` 符号存在。证据
  `log/multilayer_terrain_fix/final/install_checks.log`。

同一 `GroundMap.pcd`（85475 finite points，SHA256
`19bbfbf01f4c9b6fe6e07c02dc1fef105b6ec4f607f4305e30d16cd4d03246d7`）
以精确当前 0.20 m 参数构建。旧静态 probe（旧 core）与本最终 probe 的对照见
`probe_old_groundmap.log` / `probe_final_groundmap_groundonly.log`：旧
83107 nodes、2696 双层边界 seeds、356259 connections；最终 83339 nodes
（83314 supported + 25 prohibited）、86728 candidate layers、358803
connections。鲁棒尝试 329、恢复 144、拒绝 185（其中显著 depression 保守
拒绝 10）。节点数变化不是验收目标，真实桥区样本才是否定“原始点缺失”的依据。

桥区 cell `(14,-130)`（中心 XY `(2.9,-25.9)`）上层 5 个原始支撑点
Z=`[1.27044,1.32971]`、RMS=`0.00364974`、倾角=`0.397967 rad`（<0.60）
现已保留为 supported measured geometry；该节点 representative z 为
`1.36964`，这是局部拟合平面在格心的外推值，不改变其 5 点实测支撑证据。
audited 样例 `(21,-133)`、`(26,-124)` 的上层 5 点也都保留。保留不等于车辆
可通行；没有降低 `max_slope_rad`、`max_step_height_m`、支撑/空洞或跨层限制。

实际最终 `BuildTerrainVisualization`（200000 上限、静态离线模式）：
rendered_cells=83339、invalid_cells=0、truncated=false、marker_count=83。
独立证据计数：measured seed nodes=4127、insufficient-connection seed nodes=5232、
both=2227、neither=76207，兼容距离不一致=0；连接共 358803，其中 connected
322134、height_step 25522、normal_mismatch 340、boundary_coverage 5853。
最终 0.20 m CPU 链搜索成功 51 poses，共享 `PathValidator` 返回 valid；这是单次
CPU 功能证据，不是性能/实车数据。

Ground-only（空 obstacle）与同一 GroundMap 作为 obstacle 数组的探测结果：
两者节点/几何相同，但 static obstacle records 分别为 occupied=40/
clear=83274/unknown=25 与 occupied=54/clear=83260/unknown=25。YAML 中
`offline.ground_pcd` 与 `offline.obstacle_pcd` 都指向 GroundMap 时，planner 沿用既有
“相同路径不重复加载 obstacle”语义，探测记录 `obstacle_points=0`；这不构成
empty-obstacle authoritative-ground 情形的安全性假设，raised/depressed
ground 证据由上述鲁棒证据路径单独保守处理。

使用唯一入口。首次使用本轮修改时，先在主工作区重新编译并 source；本轮自动验证只安装到了隔离前缀：

```bash
cd /home/robot/navigation_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select dddmr_grid_global_planner
source install/setup.bash
ros2 launch dddmr_grid_global_planner path_planning_on_static_layer.launch.py \
  use_sim:=false
```

`map_dir`/`ground_dir` 仅用于显示，由 launch 从 `offline.global_pcd`/
`offline.ground_pcd` 注入，YAML 不再保存重复显示路径；`ground_pcd`/
`obstacle_pcd` 默认都指向 GroundMap；默认规划模式不读取 `global_pcd`，显示发布器仍读取它。不要额外
创建 launch/YAML。ROS 图、RViz 实渲染、
实时感知和硬件运动仍未运行；不要把本节的 CPU/离线证据表述为实车验收。

### 0.1 最终参数与证据语义

- 离线精细参数保持 `resolution_m=0.20`、`neighborhood_radius_m=0.35`、
  `max_sampling_gap_m=0.30`、`min_neighborhood_points=4`、`max_plane_rms_m=0.05`、
  `max_slope_rad=0.60`、`max_step_height_m=0.12`、
  `layer_gap_threshold_m=0.20`、`min_boundary_coverage=0.25`；候选 cell 上限
  2,000,000。`max_normal_angle_rad=0.35` 只用于相邻拟合面的法向一致性，不再
  作为绝对倾角删除条件；绝对超坡仍按 `max_slope_rad` 标记为 prohibited。
- 鲁棒回退只在普通平面拟合退化或 RMS 超限时启动。最终共识固定为相对原始
  layer N 的 `max(min_points, ceil(2N/3))`；确定性 trim/refit 最多 7 次，无随机
  采样、无阈值放宽。返回前要求成员集合与最终平面逐点残差一致；被排除点不参与
  hull、nearest、RMS、Z 范围、质心、支撑或地形几何 hash；保留的障碍证据仍属于独立占据层。
- 被鲁棒分组排除、且相对恢复平面显著向上的真实 ground 证据，会按既有
  duplicate/height/association 策略作为障碍候选加入该节点，不能静默变成 clear
  低层 ghost；同一 cell 内被排除且低于平面超过既有 `max_step_height_m` 的显著
  depression 会保守拒绝该鲁棒候选（`rejected_robust_depression`），不会在被挖低
  地形上虚构桥面。正的污染 helper 测试保留几何恢复，但不承诺 occupancy-clear。
- 同层 edge 证据先取两侧实际 inlier 凸包与有限公共边的精确裁剪区间，再与
  每个真实 inlier 的既有 `max_sampling_gap_m` 圆在边线上的交叠区间求交；最终
  公共区间长度/边长得到 coverage，高度差在公共区间端点用拟合平面仿射值比较。
  corner 必须位于两侧真实 hull 内且最近真实 inlier 不超过既有 sampling gap。
  该过程不扩大支撑域、不跨洞/跨层，端点零长度接触不计 coverage。
- 硬优先级不变：occupied/prohibited/unknown/unsupported 先于软边界；物理
  footprint/车辆配置仍独立。黄色只表示已知 measured support-domain edge 的
  软热度，紫/蓝色只表示 insufficient connection evidence；0 soft weight 不能
  改写标签或硬门限。
- 诊断按 scope 区分：`candidate_cells` 是网格候选，`candidate_layers` 是分层
  候选，`accepted_nodes`（= supported + prohibited）才是节点；每个最终拒绝
  只有一个首因计数，且 `candidate_layers = accepted_nodes + 全部 rejected_*`。
  每 gate 最多一个、总量最多 8 个有界样例，不输出逐点历史。

> 以下第 1–13 节保留早期阶段历史。凡与本节冲突的旧数字/旧默认值（例如
> 299981 nodes 或旧 GlobalMap 默认）均属历史，不应作为当前结果。

## 1. 中文摘要

## 1. 中文摘要

`dddmr_grid_global_planner` 是新加的独立三维地形栅格全局规划器，保留旧
`global_planner` 包和 `/get_plan` action。核心流程是：

1. 地面点云冻结为不可变 `TerrainPointObservation`；
2. `TerrainGridBuilder` 生成 0.1 m 多表面 `TerrainSnapshot`；
3. `CoarseTerrainMap` 生成默认 0.4 m 引导摘要；
4. `ObstacleTileLayer` / `MapUpdateWorker` 维护静态与动态障碍贡献；
5. `MapSnapshot` 把精细地形、粗图、障碍层和 pinned 距离场绑定为一个只读版本；
6. Graph A* 找候选通道，Ackermann Hybrid A* 在精细图上做四轮支撑/车体姿态/
   碰撞检查，失败时扩大通道或回退精细搜索；
7. `PathValidator` 在最新快照上按同一轨迹重新检查；
8. ROS 节点通过 `/get_plan` 返回 path，并发布诊断/可视化私有 topic。

感知侧（`perception_3d`）增加一个 **默认关闭** 的兼容导出：
`PerceptionSnapshot` 在完整 clear/mark 周期之后冻结当前观测，供同一进程内
的 grid 节点读取。旧 perception 节点不开启时接口、topic、默认行为不变。

## 2. English summary

`dddmr_grid_global_planner` is an independent ROS 2 grid global planner that
keeps the legacy `global_planner` package and the existing `/get_plan` action.
It pins one immutable `MapSnapshot` per planning job, uses coarse guidance plus
fine Ackermann Hybrid A*, checks four-wheel/body support with one evaluator, and
revalidates against the latest snapshot. The perception package optionally
exports an immutable `PerceptionSnapshot` after the full clear/mark cycle; the
export parameter defaults to OFF so old nodes are unchanged.

Stage 6 end-to-end concurrency/performance/vehicle acceptance remains pending.
Current vehicle input measurements are listed in section 9.

## 3. 模块流程 / Module flow

```
perception_3d (grid instance: dddmr_grid_perception, export_perception_snapshot=true)
  plugins selfClear -> selfMark -> updateLethalPointCloud   (full cycle)
        │
        └─ StackedPerception::exportSnapshot()             (after cycle, bounded lock)
             └─ PerceptionSnapshot (deep-copied vectors, no PCL alias)
                  └─ in-process getLatestPerceptionSnapshot()
                       └─ SourceObservationSnapshot (planner-side adapter)

TerrainPointObservation (immutable, deep-owned)
        │
        ▼
TerrainGridBuilder ──► TerrainSnapshot (0.1 m, version-scoped IDs)
        │                    ├──► CoarseTerrainMapBuilder ──► CoarseTerrainMap (0.4 m)
        │                    └──► ObstacleDistanceField (pinned/truncated)
        │
        └── MapUpdateWorker: source TTL/duplicate/order, static/dynamic split,
            Tile/COW publication, background retirement
                    │
                    ▼
             SnapshotStore::Publish(MapSnapshot)
                    │
                    ├──► CorridorBuilder
                    ├──► GridGraphAStar (coarse candidate)
                    ├──► AckermannHybridAStar (fine; wheel support + body OBB)
                    └──► PathValidator (latest snapshot replay)
                              │
                              ▼
                    /get_plan action result + private diagnostics
```

关键边界：

- **观测只在完整周期后导出。** clear/mark 未完成、sensor 过期、cloud 为空或
  非有限点时，导出为 unusable/unknown；**绝不把“没有点”解释成 free**。
- **快照深拷贝。** `PerceptionSnapshot` 只含 `std::vector`，不暴露 mutable
  `pcl::PointCloud` 指针。插件之后写自己的缓存不会改变已导出的对象。
- **锁只包住冻结。** 感知锁内只做当前观测的有限深拷贝；不建地形、不扫全图、
  不更新距离场、不做搜索。
- **source 语义显式。** 每个 source 带 static/dynamic、complete replacement 或
  exact delta、occupancy/clearing、known volume/headroom。
- **未知不是空闲。** 下游对 unusable/empty/stale snapshot 必须 fail closed。

## 4. 源码文件 / Source files

| 文件 | 作用 |
|---|---|
| `include/dddmr_grid_global_planner/terrain_types.h` | ID、点/面、状态、障碍属性、证据 |
| `include/dddmr_grid_global_planner/input_contract.h` | 不可变地面观测与校验策略 |
| `include/dddmr_grid_global_planner/terrain_grid_builder.h` | 可配置精细分辨率的多表面地形构建 |
| `include/dddmr_grid_global_planner/terrain_visualization.h` | 地形单元面片、边框及状态颜色 |
| `include/dddmr_grid_global_planner/map_snapshot.h` | 固定快照、pinned field/coarse、`CreateWithPinnedField` |
| `include/dddmr_grid_global_planner/multiresolution_map.h` | 0.4 m 保守摘要与粗细映射 |
| `include/dddmr_grid_global_planner/corridor_builder.h` | 只读通道视图与有界扩边 |
| `include/dddmr_grid_global_planner/distance_field.h` | 截断距离场、`UpdateIncremental` |
| `include/dddmr_grid_global_planner/wheel_support.h` | 四轮/车体几何评估 |
| `include/dddmr_grid_global_planner/ackermann_kinematics.h` | 唯一精确 Ackermann 积分器 |
| `include/dddmr_grid_global_planner/grid_search.h` | Graph A* / Hybrid A* / 预算 |
| `include/dddmr_grid_global_planner/path_validator.h` | 最新快照路径复核 |
| `include/dddmr_grid_global_planner/source_observation.h` | 阶段 4/5 感知 source 契约（planner 侧） |
| `include/dddmr_grid_global_planner/obstacle_tile.h` | 阶段 4/5 Tile/COW 障碍层 |
| `include/dddmr_grid_global_planner/snapshot_store.h` | 阶段 4/5 保留版本与 latest 获取 |
| `include/dddmr_grid_global_planner/map_update_worker.h` | 阶段 4/5 单写线程、TTL、发布 |
| `include/dddmr_grid_global_planner/grid_global_planner.h` | ROS action、调度和 `~/metrics` 发布 |
| `include/dddmr_grid_global_planner/metrics.h` | 小型阶段计时/查询计数/资源快照类型（steady-clock） |
| `src/*.cpp` | 上述核心实现；唯一安装的可执行入口是 `grid_offline_planner_node.cpp`（原 live 两节点 main 已退役并从 build/install 移除） |
| `config/dddmr_grid_global_planner.yaml` | 唯一 ROS 参数 YAML；五个命名节点 section |
| `launch/path_planning_on_static_layer.launch.py` | 唯一 launch；直接声明五个节点 |
| `src/offline_static_map.cpp` | 真实邻域拟合、多层地形连接、静态障碍分类与离线快照 |
| `src/offline_pcd_loader.cpp` | 通用 PCD 读取与异常处理 |
| `src/grid_offline_planner_node.cpp` | 不创建感知栈的独立可执行入口 |
| `src/dddmr_perception_3d/include/perception_3d/perception_snapshot.h` | 不可变 `PerceptionSourceObservation` / `PerceptionSnapshot` |
| `src/dddmr_perception_3d/src/perception_snapshot.cpp` | 快照校验/诊断实现 |
| `src/dddmr_perception_3d/src/stacked_perception.cpp` | 完整周期后的深拷贝导出 |
| `src/dddmr_perception_3d/src/perception_3d_ros.cpp` | `export_perception_snapshot` 参数与 hook |

原计划测试文件（以下历史清单未表示全部运行；当前离线测试结果见文首）：

- planner core: `test/test_terrain_types.cpp`, `test_terrain_grid.cpp`,
  `test_multiresolution_map.cpp`, `test_distance_field.cpp`,
  `test_wheel_support.cpp`, `test_grid_search.cpp`,
  `test_production_corrections.cpp`, `test_path_validation.cpp`,
  `test_obstacle_updates.cpp`, `test_snapshot_concurrency.cpp`,
  `test_metrics.cpp`, `test_ros_interfaces.py`,
  `test/benchmark_snapshot_search.cpp` (all **UNRUN**).
- benchmark telemetry: `benchmarks/dddmr_global_planner/tests/
  test_telemetry_tools.py` (adapter/collector serialization, hierarchy, query
  classes and failure retention; **UNRUN**).
- perception: `src/dddmr_perception_3d/test/test_perception_snapshot.cpp`。
- 既有 perception 测试不变；未新增插件行为测试。

## 4.1 Telemetry / `~/metrics` (UNRUN)

The node publishes `std_msgs/String` JSON records on `ros.metrics_topic`
(default `~/metrics`), outside the job/commit/store guards and never from the
search hot loop. Two record types are emitted:

- `request`: one record per accepted request and **every** terminal outcome
  (success, no-route, cancel, budget exhaustion, not-ready, invalid input,
  internal error). It carries request/goal sequence, used terrain/content-hash,
  obstacle/coarse version, configured fine/coarse resolution, queue wait,
  search/validation/commit/total durations, expansion/query/support counts,
  four-wheel phase timings, rejection histogram, coarse/fine/fallback/
  reconstruction timings and optional Linux `getrusage` process/thread CPU plus
  peak RSS. Server commit is explicitly separate from client delivery and is
  linearized at the atomic immutable-snapshot acceptance; the action terminal
  transition and private path publish run after the store/job locks are released.
  A map update after that commit may be visible during delivery, so the record
  does not claim map freshness at client receipt.
- `map_update`: one record per processed update event and per publication. It
  carries enqueue/start/publish monotonic instants, queue wait, contribution,
  COW, coarse-overlay, distance-field, snapshot, publish and reclaim durations,
  dirty cells/nodes, updated tiles, estimated copy bytes, live/retired
  diagnostics when available, and the actual terrain-build phase metrics.

Metric hierarchy rules:

- Total and child phases are separate fields. `fine_ms` includes support
  evaluation and reconstruction; `support_eval_ms`/`reconstruction_ms` are not
  added to it again. `coarse_ms` and `fallback_ms` are reported separately.
- Terrain segmentation and attribute phases are inclusive of nested plane
  fitting. `*_fit_inclusive` is measured; `*_fit_exclusive` is an arithmetic
  difference, never a second timing.
- `support_evaluations`, `sampled_poses`, `map_queries` and the per-class
  cell/surface/neighbor/obstacle/distance query counts are distinct counters.
  Evaluator-internal body-envelope lookups are not individually counted, so the
  query counts are documented as instrumented lower bounds.
- Sample counts are evidence-sufficiency gates, not confidence values.
- Sensor/ROS stamps are used for observation order/freshness; they are never
  subtracted from steady-clock instants. Cross-process client latency is left
  `unmeasured` unless a co-host monotonic domain is validated.
- Metric 22 (cold pointcloud-to-path) is not synthesized. It requires a
  pre-submitted target and an actual ground receipt for the used terrain
  revision; the record carries the instants but marks the metric unmeasured
  unless that experiment is run.
- Detailed per-query latency histograms are OFF by default
  (`diagnostics.detailed_query_timing=false`). Missing metric categories are
  published as `unmeasured` with a reason, not as zero.

Collection/adaptation (documentation only, **UNRUN**):

```bash
cd /home/robot/navigation_ws/src/dddmr_navigation-for-Ackerman
python3 benchmarks/dddmr_global_planner/collect_metrics.py \
  --jsonl /tmp/grid_metrics.jsonl --csv /tmp/grid_metrics.csv \
  --adapted-jsonl /tmp/grid_benchmark.jsonl --duration 60
python3 benchmarks/dddmr_global_planner/report.py \
  --input /tmp/grid_benchmark.jsonl --markdown /tmp/report.md --json /tmp/report.json
```

`benchmarks/dddmr_global_planner/collect_metrics.py` is the ROS subscriber;
`telemetry_adapter.py` converts request records into the benchmark schema used
by `report.py`. Update records stay in the raw JSONL for phase analysis. If the
collector and node do not share a validated monotonic domain, the collector
writes `client_latency_status=unmeasured: cross-process monotonic domain not
validated` rather than combining clocks.

### Explicit free-space evidence

`ObservedVolume.free_space` is the only signal that turns a known volume into
clear evidence. A same-source occupied slab that overlaps the volume still
blocks that clear, and a missing/unknown volume never becomes free. Sensor
absence, an empty cloud, or an unusable snapshot is not a clear.

### Physical parameters remain unset

Vehicle geometry (wheel offsets, wheelbase, track, steering limits, base contact
height) is configured from the input manifest. Tire radius/width,
suspension/tire compliance and the full `body_max` envelope are still unset;
`VehicleCapabilityConfig::Validate()` therefore fails closed and the planner
must not plan until measured values are supplied. Offline geometry is not a
load-transfer, rollover or real-vehicle pass claim.

## 5. 构建 / Build (UNRUN)

先加载 ROS 环境（把 `humble` 换成实际发行版）：

```bash
source /opt/ros/humble/setup.bash
cd /home/robot/navigation_ws
```

完整构建相关包（推荐，包含依赖）：

```bash
colcon build --symlink-install \
  --packages-up-to perception_3d dddmr_grid_global_planner p2p_move_base \
  --cmake-args -DBUILD_TESTING=ON
```

只构建新包：

```bash
colcon build --symlink-install \
  --packages-select dddmr_grid_global_planner perception_3d \
  --cmake-args -DBUILD_TESTING=ON
```

运行测试（UNRUN）：

```bash
colcon test --packages-select dddmr_grid_global_planner perception_3d \
  --event-handlers console_direct+
colcon test-result --verbose
```

### 独立核心构建（无 ROS/ament，UNRUN）

From the repository root
`/home/robot/navigation_ws/src/dddmr_navigation-for-Ackerman`, the correct
absolute source path is
`/home/robot/navigation_ws/src/dddmr_navigation-for-Ackerman/src/dddmr_grid_global_planner`
(not `/home/robot/navigation_ws/src/dddmr_grid_global_planner`).

```bash
cd /home/robot/navigation_ws/src/dddmr_navigation-for-Ackerman
cmake -S /home/robot/navigation_ws/src/dddmr_navigation-for-Ackerman/src/dddmr_grid_global_planner \
  -B /tmp/dddmr_grid_build \
  -DDDDMR_GRID_STANDALONE=ON \
  -DDDDMR_GRID_BUILD_TESTS=ON
cmake --build /tmp/dddmr_grid_build -j
ctest --test-dir /tmp/dddmr_grid_build --output-on-failure
```

### 代码格式 / Formatting and lint（ROS uncrustify）

包内 C++ 以 Humble 安装的 ROS `ament_uncrustify` 默认配置
`ament_code_style.cfg` 为唯一格式来源；不再使用 Google `.clang-format`。
头文件按 C++（`--language C++`）解析，避免 `.h` 后缀被当成 C。ROS 构建且
`BUILD_TESTING=ON` 时注册三个真实 CTest lint：

- `uncrustify` — `ament_cmake_uncrustify`，检查包内 `include/`、`src/`、
  `test/` 的 C++；
- `cpplint` — `ament_cmake_cpplint`，保留其自身默认 filters。仅排除
  `-readability/braces`，因为 ROS uncrustify 要求 Allman 大括号，而该
  Google K&R 规则无法同时满足；没有其他 blanket suppression；
- `flake8` — `ament_cmake_flake8`，检查包内 Python。

独立核心构建（`DDDMR_GRID_STANDALONE=ON`）不要求 ament/lint 依赖。
`NOLINT(build/include_subdir)` 只用于 `src/` 私有头文件的同目录 include。
两个内聚函数在闭括号处保留带具体理由的逐函数
`NOLINT(readability/fn_size)`：

- `offline_static_map.cpp` 的 `BuildOfflineStaticMap`：ROS 格式化前 479 个
  非注释行，格式化后 510（+31）；
- `wheel_support.cpp` 的 `WheelSupportEvaluator::Evaluate`：494 → 520（+26）。

两者在 ROS uncrustify 之前都低于 cpplint 的 500 行触发阈值，是格式化后的
续行展开使其越线；拆分需要复制单遍遍历/支撑状态或传递大量局部状态，因此
保留内聚函数并逐函数豁免，而不是关闭 `readability/fn_size` 过滤器。没有
其他 blanket suppression。

### 实际存在的 `DDDMR_GRID_*` CMake 变量

只使用下面四个；不要发明 `DDDMR_GRID_BUILD_NODE` 之类的选项。测试和
benchmark 都还受标准 CMake/CTest 的 `BUILD_TESTING` 总开关约束。

| 变量 | 默认 | 含义 |
|---|---|---|
| `DDDMR_GRID_STANDALONE` | `OFF` | `ON` 时不走 ament/ROS，只构建核心库与测试；不安装 launch/config。 |
| `DDDMR_GRID_BUILD_TESTS` | `ON` | 在 `BUILD_TESTING=ON` 时是否构建核心测试。 |
| `DDDMR_GRID_BUILD_BENCHMARKS` | `ON` | 在测试开启时是否构建 `benchmark_snapshot_search`。 |
| `DDDMR_GRID_ENABLE_TSAN` | `OFF` | 加 `-fsanitize=thread`；平台不支持时必须记录失败，不能宣称已做竞态验证。 |

> **`-D` 拼写注意：** CMake 的 `-D` 前缀 + 变量名 `DDDMR_...`，所以命令行里
> 变量名前面是 **四个 D**：`-DDDDMR_GRID_STANDALONE`，不是
> `-DDDMR_GRID_STANDALONE`。本文所有命令均按此写法。

### ThreadSanitizer（UNRUN）

```bash
cd /home/robot/navigation_ws/src/dddmr_navigation-for-Ackerman
cmake -S /home/robot/navigation_ws/src/dddmr_navigation-for-Ackerman/src/dddmr_grid_global_planner \
  -B /tmp/dddmr_grid_tsan \
  -DDDDMR_GRID_STANDALONE=ON \
  -DDDDMR_GRID_BUILD_TESTS=ON \
  -DDDDMR_GRID_ENABLE_TSAN=ON
cmake --build /tmp/dddmr_grid_tsan -j
ctest --test-dir /tmp/dddmr_grid_tsan --output-on-failure
```

## 6. 启动 / Launch (UNRUN)

本包只有一个 launch 和一个 ROS 参数 YAML：

- `launch/path_planning_on_static_layer.launch.py`
- `config/dddmr_grid_global_planner.yaml`（五个命名节点 section）

```bash
cd /home/robot/navigation_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mcl_3dl perception_3d dddmr_grid_global_planner \
  --parallel-workers 1
source install/setup.bash
ros2 launch dddmr_grid_global_planner path_planning_on_static_layer.launch.py \
  use_sim:=false
```

launch 参数只有 `use_sim` 和 `rviz_config`。所有 PCD 路径在唯一 YAML
`config/dddmr_grid_global_planner.yaml` 中设置：

- 规划输入：`dddmr_grid_global_planner.ros__parameters.offline` 下的
  `global_pcd`、`ground_pcd`、`obstacle_pcd`。
- 显示输入：launch 从上述 `offline.global_pcd`、`offline.ground_pcd`
  注入 `pcl_publisher` 的 `map_dir`、`ground_dir`；YAML 不另存显示路径。

`global_cloud_mode=false` 时使用 `ground_pcd` 重建地面，`true` 时使用
`global_pcd`；不同路径的 `obstacle_pcd` 仍可作为独立障碍输入。
显示路径只是同一 YAML 顶部路径的副本，不覆盖规划输入，也不能通过 launch
命令行单独改写。
点云应已处于 `map` 坐标系；显示变换不会变换规划器直接读取的 PCD。

默认启动五个节点：PCD 显示发布器、显示用静态 TF（`map2baselink`）、独立
离线规划器 `dddmr_grid_offline_planner_node`、RViz 选点 bridge 和 RViz。
不会创建 perception、雷达插件、点云预处理、定位或控制节点。独立规划器读取
`offline.ground_pcd` / `offline.obstacle_pcd`，只提供一个 `/get_plan`
provider，并发布处理后的地形 `~/terrain`、粗摘要 `~/coarse` 和请求路径
`~/path`。

`use_sim_time` 在所有五个节点上都以
`ParameterValue(..., value_type=bool)` 传递，避免 Humble 把字符串
`"false"` 当成真值。YAML 中每个 section 使用实际 ROS 节点名：

| Section | 节点 |
| --- | --- |
| `pcl_publisher` | `mcl_3dl/pcl_publisher` |
| `map2baselink` | `global_planner/static_transform_from_params.py` |
| `dddmr_grid_global_planner` | `dddmr_grid_offline_planner_node` |
| `dddmr_grid_rviz_bridge` | `rviz_goal_bridge.py` |
| `rviz2` | `rviz2` |

不再加载外部 p2p/mcl YAML。共享坐标系、话题、bridge、PCD 发布器和车辆参数
都直接保存在这一个 YAML 中。文件中的 `wheel_support.evaluation_model` 为
`offline_kinematic_static_map`，用户入口固定为离线 PCD 模型；包内 C++/ROS
header 默认仍保留 `full_wheel`，供其他内部调用路径使用。

### 6.1 RViz 点击规划 / RViz click planning

RViz 工具与 bridge topic（均不触发定位）：

| 用途 | RViz 工具 | Topic | 类型 / QoS |
| --- | --- | --- | --- |
| 选起点 | `rviz_default_plugins/SetInitialPose` | `/dddmr_grid_global_planner/rviz_start` | `PoseWithCovarianceStamped`, reliable volatile |
| 选终点 | `rviz_default_plugins/SetGoal` | `/dddmr_grid_global_planner/rviz_goal` | `PoseStamped`, reliable volatile |
| 起终点箭头 | MarkerArray | `/dddmr_grid_global_planner/rviz_markers` | `MarkerArray`, transient local reliable |
| 规划路径显示 | Path | `/dddmr_grid_global_planner/rviz_path` | `Path`, transient local reliable |
| 规划请求 | — | `/get_plan` | `dddmr_sys_core/action/GetPlan`, `activate_threading=true` |

先用 **2D Pose Estimate** 选起点和朝向，再用 **2D Goal Pose** 选终点和
朝向。bridge 保存独立起点，通过 `/get_plan` 提交请求，只显示当前请求成功
返回的路径；它不会发布 `/initialpose` 或任何车辆命令。

bridge section 的高度策略：

- `start_height_mode` / `goal_height_mode`：`fixed`（默认）或 `click`。
- action pose 的 position 是 map 中 base_link 的参考位置/高度，不是地形
  表面高度；solver 负责选择支撑层并拟合 pose。
- 默认 `start_fixed_z_m` / `goal_fixed_z_m` 为 `-0.5`，与显示静态 TF 的
  `tf_z=-0.5` 一致。多层地图应把这两个值改成目标楼层对应的 base_link 参考
  高度；bridge 不会根据地形/TF 自动推断 z。
- `vehicle.base_contact_height_m = 0.22 m` 是 planner 车辆参数，不是 bridge
  侧 z 偏移。

最新请求优先且有界 fail-closed：bridge 最多保留一个在途 send future 或一个
active handle；新终点会清空 `rviz_path` 并异步取消旧 goal，只有旧 goal 到达
终态后才提交最新 pending。`rviz_path` 由 bridge 唯一持有，planner 自己的
`~/path` 保持不变。

结果通道失败不会伪造终态：`get_result_async()` 的 future 抛异常、返回空响应
或非终态/`UNKNOWN` 状态时，bridge 保留该 active handle、只请求一次取消，并
继续不提交任何新 goal。失败的 future 不会自行变成终态，bridge 也不会自动
重试或自动恢复；若取消后仍拿不到终态，只能人工重启 action server 或重启
bridge，之后由用户重新选点。恢复动作必须是显式的。

RViz 布局 `rviz/dddmr_grid_global_planner.rviz` 使用 Fixed Frame `map`，
显示 `/mapground`、`/mapcloud`、`/navigation/preprocessed_cloud`、
`/dddmr_grid_global_planner/terrain`、`~/coarse`、`~/corridor`、
`/dddmr_grid_global_planner/rviz_path` 和起终点 markers。离线入口没有实时
雷达，`PreprocessedCloud` 可关闭。

`PlannerTerrain` 使用 `visualization_msgs/msg/MarkerArray` 显示真实格网单元。
每格 XY 边界来自地图原点、格子索引和分辨率（当前配置为 0.20 m），
四角高度来自该层地形的拟合平面，坡面和同一 XY 位置的不同楼层分别绘制。
`terrain_cells` namespace 显示面片，`terrain_grid_edges` 显示深色边框，
可在 RViz 的该图层下分别开关。没有节点的位置保持空白，不做补洞。

| 面片颜色 | 含义 |
|---|---|
| 绿色 | 支持地形、当前模型 clear，且不在已知支撑边缘/证据不足范围内 |
| 黄色/暖色 | 已知 measured support-domain edge 的软边界热度；不是几何悬崖或车辆约束 |
| 紫/蓝色 | insufficient connection evidence（邻近有可用支撑面但连接未评估或证据不足）；不得当作黄色几何边缘或安全绿色 |
| 红色 | 当前模型中已占据 |
| 橙色 | 地形被标记为禁止通行 |
| 灰色 | 有可绘制平面，但地形或占据证据为未知 |

离线模式的颜色依据静态占据；实时模式只有静态、动态两层均 clear 才显示绿色。
硬状态（occupied/prohibited/unknown/unsupported）优先于边界热度。把
`boundary_penalty_weight` 或 `boundary_penalty_distance_m` 设为 0 只关闭软热度，
不会把 insufficient-connection 紫/蓝标签变成绿色，也不会放宽硬门限。
规划器的三个 PCD 路径以 YAML 为准；launch 仅将顶部 `global_pcd`/
`ground_pcd` 注入显示发布器，不改写规划输入。
默认 `offline.ground_pcd` 和 `offline.obstacle_pcd` 同路径，planner 跳过独立障碍
输入。只有在 YAML 中配置不同路径的独立障碍 PCD 时才额外加载；
额外 obstacle 输入只做加法，不会替换地面几何。
因此 `MapCloud` 的显示开关不影响占据判定。当前静态模型中，一个点相对关联
地面高出 0.08 m 且不超过 1.00 m 就会触发占据；这不是对噪点、动态残影或
车辆可跨越能力的识别。应核对障碍输入的分割、配准和点云质量。
关联先按水平位置选择最近支撑格，再按高度区分该位置的不同楼层；位于点上方
超过地面容差的层不能充当其支撑面，避免错位占据和上层地面掩盖下层障碍。
面片表达格网和局部拟合面，不证明整个格子具有轮胎支撑、足够净空或整车可通行。
每批最多绘制 2048 个单元；`diagnostics.max_visualization_points` 对此图层
限制的是单元表面数量，0 表示不限制。达到上限或跳过无效几何时会报告显示警告，
不会改变规划地图。每次完整替换都会先清除旧面片，空地图也不会残留旧格子。

旧版本 `~/terrain` 是 PointCloud2，新版本为 MarkerArray；升级后需重新编译并
重启 launch。使用自己保存的旧 RViz 布局时，重新加载本包布局，或将
`PlannerTerrain` 替换为订阅同一话题的 MarkerArray 显示。

### 6.2 地形支持边界软代价 / Terrain support-boundary risk

这是**仅离线静态 PCD 模型**的软路由偏好，不是碰撞或实车通过性证书。

- 缓存方向：对每个 supported 节点检查 8 个 XY 方向；只要任一方向缺少通过真实
  connected supported 局部邻接边可达的支撑，该节点就是边界种子（距离 0）。
  距离由多源 Dijkstra 只沿这些已连接的相邻支撑格传播，单位是格网水平距离。
  因此缓存覆盖地图边缘、空洞、断开的地面以及 prohibited 支撑面边界；占据层
  的 occupied 墙格本身不进入这个支持边界缓存，不能把墙格当作边界种子。
- 代价：`weight * clamp(1 - distance / range, 0, 1)^2`。参数
  `wheel_support.boundary_penalty_distance_m`（range）与
  `wheel_support.boundary_penalty_weight`（weight）必须有限且非负，默认
  `0.50 m` / `1.0`。任一为 0 显式关闭该项。参数只在启动时读取，修改后必须
  重启 launch；只影响离线模型，不影响 `full_wheel` 模型。
  对应唯一 YAML 中的 `wheel_support.boundary_penalty_distance_m` 和
  `wheel_support.boundary_penalty_weight`。
- 规划取值：评估姿态时，先把车体测量 XY 矩形按 query heading 旋转，枚举与
  这个**有向整车 footprint** 保守相交（含恰好接触）的细格；对同一批格子执行
  hole/prohibited/occupied/chassis 硬门限并取 boundary penalty 的最大值。该
  最大值加入离线评估 `cost`，随后由既有 `search.terrain_cost_weight` 计入搜索
  累计代价。矩形不可用时回退到 `wheel_support.offline_planar_collision_radius_m`
  配置的圆形包络；该半径应覆盖实际整车包络。格网量化取
  `terrain.fine_resolution_m`（当前配置 0.20 m），不是按固定采样间距跳过格子。
- RViz：`PlannerTerrain` 的 clear supported 单格颜色按该格自身距离由绿到黄
  （warm）显示；规划实际使用的是整个有向 footprint 的最大值，所以单格绿色
  不代表整车姿态安全。occupied 红、prohibited 橙、unknown/invalid 灰始终优先，
  灰也用于参数非法，不会被软热力覆盖。

本节的几何/代价/可视化回归已在本地 CPU 软件中实际运行，记录在
`log/terrain_boundary_cost/`；未运行 ROS graph、RViz 端到端或硬件，不能据此
声称实时或实车验收。

## 7. Action 用法 / Action usage (UNRUN)

Action 类型：`dddmr_sys_core/action/GetPlan`，名称 `/get_plan`。
请求字段：`goal`、`start`、`activate_threading`。

### 7.1 正常规划，显式 `start`

```bash
ros2 action send_goal /get_plan dddmr_sys_core/action/GetPlan \
'{goal: {header: {frame_id: "map"}, pose: {position: {x: 8.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}},
  start: {header: {frame_id: "map"}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}},
  activate_threading: true}'
```

### 7.2 正常规划，空/default `start` 走 TF

```bash
ros2 action send_goal /get_plan dddmr_sys_core/action/GetPlan \
'{goal: {header: {frame_id: "map"}, pose: {position: {x: 8.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}},
  start: {header: {frame_id: ""}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}},
  activate_threading: true}'
```

语义（必须以节点实现为准）：

- `activate_threading == true`：正常 planning。
- `start` 为空/默认：用 TF 解析 `map <- base_link`，也可用已有 global pose
  source；TF 不可用或过期 -> 明确失败，不猜起点。
- `start` 显式且有限：经过 frame 变换/校验后使用；不是“有 start 就跳过 TF
  安全校验”。
- `activate_threading == false`：取消/清空命令。取消正在运行的 plan、清空
  待发布 path，返回 `succeeded` + 空 path（即使 planner 忙也要 bypass），
  绝不把它当几何请求。
- 错误走 Action status + `~/diagnostics`；该 action 没有 error 字段。

### 7.3 取消/清空

```bash
ros2 action send_goal /get_plan dddmr_sys_core/action/GetPlan \
'{goal: {header: {frame_id: "map"}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}},
  start: {header: {frame_id: ""}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}},
  activate_threading: false}'
```

## 8. 参数来源 / Parameter provenance

1. **已确认车辆几何**：来自
   `benchmarks/dddmr_global_planner/baseline/input_manifest.json` 和本地地图
   overlay。轴距 `0.55 m`，前后轮距 `0.52 m`，四轮接触偏移
   `(±0.275, ±0.26, -0.22)`，base 接触高度 `0.22 m`，中心/内侧转向角
   `0.3270520925102273 / 0.3839724354387525 rad`。这些不是本文新测的。
2. **配置默认值**：`config/dddmr_grid_global_planner.yaml`。包括
   `terrain.fine_resolution_m`、`terrain.coarse_resolution_m`、
   `terrain.use_coarse_guidance`、`obstacles.truncation_distance_m`、
   `obstacles.reject_unknown`、`search.*`、`validation.*`。
3. **阶段 4/5 worker 参数**（节点实现/配置落地后按该文件为准）：
   `dynamic_source_ttl_s` / 兼容 `source_ttl_s`（dynamic 到期转 unknown；
   static 默认无 TTL）、`max_pending_batches`、`max_sources`、
   `max_records_total`、`publish_period_s`、`retained_snapshots`。队列有硬上限，
   full 时返回显式 reject 且不推进 sequence（调用方重试/重同步）；不允许
   accept-then-drop。`retained_snapshots` 是 latest 之外的 historical 数量，
   retire 由后台 reclaimer 在 `use_count==1` 后释放，达到 pinned bound 时
   publication 反压/合并而不是丢弃 pinned 引用。它们不是搜索热路径参数。
4. **launch 参数**：唯一入口
   `path_planning_on_static_layer.launch.py` 默认启动 PCD 显示发布器、
   `map2baselink` 静态 TF、独立离线规划器、RViz bridge 和 RViz；没有实时
   雷达/感知/定位/控制节点。显示和规划 PCD 路径统一由 YAML 顶部规划器
   `offline` 分组配置，launch 将 `global_pcd`/`ground_pcd` 注入显示发布器。固定 TF
   默认 `tf_x=3.0, tf_y=0.1, tf_z=-0.5, roll/pitch/yaw=0`。可用参数只有
   `use_sim`、`rviz_config`。
5. **perception 导出参数**：`export_perception_snapshot` 默认 `false`；
   `perception_snapshot_static_plugin` 默认 `map`（按插件名把静态层与动态层
   分开）。旧节点不设置时保持旧行为。
6. **不在本 checkout 的测量值**：不能从代码/配置推断。缺失时必须 fail closed，
   不能填猜测值。

## 9. 车辆测量参数与剩余验收

用户已补充轮胎、刚性悬挂模型和包含轮胎的整车包络。测量原点是整车几何
中心在地面的投影；当前 base_link 位于其上方 0.22 m，因此仅 Z 减去 0.22 m。

| 配置项 | 当前值（米） |
| --- | --- |
| `body_min_m_approximate` | `[-0.410, -0.322, -0.220]` |
| `body_max_m` | `[0.410, 0.322, 0.090]` |
| `wheel_radius_m` | `0.1375` |
| `tire_width_m` | `0.08912` |
| `suspension_travel_m` | `0.0` |
| `tire_compliance_m` | `0.0` |

`body_min_m_approximate` 保留旧参数名，但 `body_min_is_approximate=false`；
包络与车辆能力配置标记均为 true。可选 `contact_patch_length_m` 未单独测量，
完整支撑模型按已有规则使用轮胎直径作为保守采样长度。配置数值有效并不
等于已通过实车验证。整体包络包含轮胎；离线平面碰撞与完整三维车体/
底盘净空判定的含义应分开理解，不应擅自提高包络下界以获取可行路径。

另外未提供/未测量：

- 真实轮胎接触力学、载荷转移、侧翻/动态稳定性模型；
- 实车通过性、同步 ground/map/TF/action 回放 bag；
- 运行时 p50/p95/p99、锁等待、峰值内存、成功率；
- 阶段 6 的并发压力/实车验收数据。

## 10. 兼容感知导出 / Perception compatibility export

- 新文件：`perception_snapshot.h/.cpp`；`stacked_perception.h/.cpp` 和
  `perception_3d_ros.h/.cpp` 只加最小 hook。
- `PerceptionSnapshot` 含：frame、sequence、terrain revision（可未知）、
  sensor/receipt stamp、`sensors_current`、per-source static/dynamic、
  complete replacement / exact delta、occupied、cleared、known volumes、
  headroom。
- 导出发生在 `sensorsUpdateLoop()` 的 `doClear_then_Mark()` 与
  `refreshObservationCache()` **之后**；锁内只深拷贝当前插件观测。
- 空 cloud、非有限点、过期 sensor -> snapshot 为 incomplete/unusable；
  没有证据的空观测 **never clear**。显式的 empty complete replacement 只有在
  同时携带有效 known/clear volume 或 cleared slab 时才有效，用于最终清障；
  这只是“显式观测到的空”，不是“没有点就 free”。
- 现有 `Sensor` 接口没有暴露 clearing 点和 free-FOV/headroom，因此兼容导出把
  `cleared` 留空、`known_volumes`/`headroom` 标记 unknown；这是显式的“没有
  证据”，不是“free”。点云只能贡献 occupied/unknown，不能证明整个 bbox free。
  新节点必须通过显式 evidence 接口 fail closed，不能因为“有 occupied 点”就
  宣称 ready-to-plan。
- 插件名等于 `perception_snapshot_static_plugin` 的 contribution 标为 static，
  其余标为 dynamic；dynamic 永远不能清 static（下游 worker 再校验）。
- 导出关闭时：不创建快照、不增加锁路径、旧节点默认行为不变。
- sensor stamp 必须是每 source 的真实 acquisition stamp（优先冻结 PCL header
  并显式时钟/坐标系 provenance）；缺失即 unknown。snapshot cycle stamp 与
  source stamp 分开，`isCurrent` 不能替代 per-source 排序/TTL。不得把
  unknown/source-frame 点云直接改标为 map frame。

### 10.1 显式 ROS observation 输入 / Explicit ROS evidence input

- `~/source_observation`（新消息 `dddmr_grid_global_planner/msg/SourceObservation.msg`）
  携带 Header acquisition stamp/frame、source id、sequence、terrain revision
  （+known flag）、static/dynamic、complete/delta、occupied/cleared slab、
  known free volume、headroom。
- 节点冻结消息后交给有界 managed ingestion；拒绝伪 stamp、非 map frame、
  越界 volume 和不支持的 transform。生产默认不允许用 enclosing AABB 猜 free
  corner。
- 原始 `PointCloud2` 路径是 **显式 opt-in**，默认不会与 perception 导出重复
  产生 dynamic source；raw cloud 只能贡献 occupied/unknown，不能声明 free
  volume/headroom。
- 可复用 core 仍保留 `perception_3d::Perception3D_ROS` + provider/weak callback
  的 in-process 导出消费路径；但唯一安装的离线入口
  `dddmr_grid_offline_planner_node` 不构造 perception 栈。原 live 两节点
  executable 已退役，不再 build/install，也不随唯一 launch 启动。

## 11. 坑 / Pitfalls

- **点云密度**：如果输入已按 0.3 m 降采样，缩小到 0.1 m 格子不能恢复细节；
  空格仍是 unknown。需要保留细节的地面输入或独立配置。
- **未知不是 free**：空 cell、缺失障碍记录、unusable/stale snapshot 都必须
  fail closed；不能因为“没有点”就清障。
- **多层地形**：同一 XY 可能有多层表面；不能按同 XY/局部 surface index 合并
  上下层，也不能把桥面障碍封到桥下。
- **deactivation**：`activate_threading=false` 是 cancel/clear，不是“规划到
  默认起点”；不要把它当几何请求。
- **快照隔离**：不要读插件 mutable `pcl::PointCloud`，不要跨进程共享旧
  `SharedData`；只用 `PerceptionSnapshot` 的 const 数据。已发布对象不得被
  后续写入改变。
- **缺少物理模型**：没有 `body_max`、轮胎/悬挂参数时不能宣称可通行；不能把
  合成测试车辆当成生产车辆。
- **距离场截断**：`min(d, R)` 是观测度量，不是保证 clearance；未知边界必须
  单独传播。
- **粗图不是最终权威**：粗图无路不能直接判精细无路。
- **不要用本文档声称运行成功**：所有阶段 4/5 能力都是 UNRUN。

## 12. 用户测试清单 / User-test checklist (UNRUN)

构建与静态：

- [ ] `colcon build` 成功；perception 新库 `perception_snapshot` 被链接。
- [ ] `test_perception_snapshot` 编译并通过（深拷贝、无证据 empty 拒绝、
      显式 empty + known/clear volume 接受、exact delta、非有限点、headroom、
      expired sensor、empty snapshot）。
- [ ] planner 的 `test_obstacle_updates`、`test_snapshot_concurrency` 编译并通过。
- [ ] `~/source_observation` 显式 evidence 映射、越界 volume 拒绝、raw cloud
      fallback 默认关闭、stamped cloud 缺 TF 拒绝，且有 diagnostics。
- [ ] TSan 构建通过，或记录平台不支持的具体原因；不能宣称已完成竞态验证。
- [ ] `test_metrics` 通过：JSON escaping、total/child hierarchy、rejection
      histogram、query-class separation、update backpressure retention。
- [ ] `benchmarks/dddmr_global_planner/tests/test_telemetry_tools.py` 通过：
      adapter 保留 measured/unmeasured，collector 保留 success/no_plan/cancel/
      budget 和 map-update 记录，CSV/JSONL 不写假零。
- [ ] `collect_metrics.py` 实际订阅 `~/metrics`，JSONL 同时含 request 与
      map_update；`telemetry_adapter.py` 输出可被 `report.py` 读取。

启动与接口：

- [ ] 唯一 launch 只启动 `dddmr_grid_offline_planner_node` 一个 provider，
      不会同时启动旧 `global_planner_node` 或在线两节点 executable。
- [ ] action `activate_threading:=true` + 显式 start 返回 path。
- [ ] action `activate_threading:=true` + 空/default start 走 TF；TF 缺失时明确失败。
- [ ] `activate_threading:=false` 在 planner 忙时也能取消并返回 succeeded 空 path。
- [ ] `route_action_name` 仍是 `get_plan`。
- [ ] 唯一 launch 默认启动 PCD 发布器、静态 TF、离线 planner、RViz bridge
      和 RViz；`rviz_config:=...` 可覆盖布局，`use_sim` 以 typed bool 传给
      全部五个节点。
- [ ] 当前模式实际读取的 YAML `offline.*_pcd` 路径存在；规划输入缺失时
      离线 planner 给出明确错误，而不是静默空地图。
- [ ] 唯一 YAML 的 `dddmr_grid_rviz_bridge` 默认
      `start_fixed_z_m=goal_fixed_z_m=-0.5`，与显示静态 TF `tf_z` 一致。
- [ ] `static_transform_from_params.py` 只发布 `map->base_link`；sensor
      extrinsics 仍由外部提供，未在本 launch 中伪造。
- [ ] 2D Pose Estimate 只更新起点 marker，不发布 `/initialpose`；2D Goal
      Pose 只提交 `/get_plan`（`activate_threading=true`）。
- [ ] 未选起点时点终点有明确提示且不提交；`require_start=false` 时才走
      planner 的空 start TF fallback。
- [ ] `fixed`/`click` 高度策略、同层/多层 start/goal fixed z 与 planner 的
      base_link 参考、coarse surface 选择、四轮支撑结果一致；默认 0 不作为
      多层场景默认正确值。
- [ ] 快速连续点终点：旧 goal 取消、`rviz_path` 清空、只有最新 goal 成功
      或明确失败；旧 generation 成功不显示。
- [ ] ack 超时不堆积 future；迟到 accepted ack 会 cancel 并跟踪到终态；
      active 超时不 abandon，terminal 前不提交下一次；server 无回应时文档
      明确需要恢复/重启。
- [ ] `use_sim=true` 且 `/clock` 暂停时 watchdog 仍由 steady clock 驱动。
- [ ] RViz 中 `mapground`/`mapcloud`/`navigation/preprocessed_cloud`/
      `terrain`/`coarse`/`corridor`/`rviz_path`/起终点 markers 可独立显示；
      某个点云缺失不影响其他 display；planner `~/path` 不参与显示。

地图/感知：

- [ ] 导出的 snapshot 深拷贝：导出后继续改插件缓存，已取快照不变。
- [ ] 空/部分 cloud 且无显式 volume evidence -> `IsUsable()==false`，不产生 clear；
      显式 empty + 有效 known/clear volume 才允许最终清障。
- [ ] 静态层与动态层 source 分离；动态 clear 不影响 static。
- [ ] 同 XY 多层地面、桥下/桥上障碍、窄通道、负坐标、同点数重建。
- [ ] 关闭 `export_perception_snapshot` 时旧 perception 行为/默认值不变。

性能/物理（阶段 6，未开始）：

- [ ] 记录更新关闭/开启的搜索 p50/p95/p99、扩展数、查询次数、峰值内存。
- [ ] 记录真实锁等待（acquire 前后）而不是查询总耗时。
- [ ] 实车确认车辆几何、悬挂/轮胎参数后再验证通过性；离线几何不等于实车结论。

## 13. 未运行声明 / No-runtime-claims

- 当前实现只写了源码、CMake、测试源码和文档；**UNRUN**。
- 阶段 1–5 的“Implemented”只表示代码已写入，不表示编译/测试/运行通过。
- 阶段 6（并发、性能、实车）未完成；没有 p95、成功率、锁等待、内存或实车
  通过性数据。
- 任何数字、p50/p95、benchmark 或“已验证”表述都必须来自用户实际运行，
  不能从本文档推断。
