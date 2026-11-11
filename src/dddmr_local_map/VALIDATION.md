# 验证记录与部署边界

日期：2026-11-05（本轮追加）。分支：`feat/local-occupancy-mppi`。

**软件接入已实现，实时控制验收尚未通过。** 未启动实车导航、发布速度或切换现有安装环境。2026-11-04 的历史记录保留在文末，旧表数字属于上一轮优化，不能当作本轮 before/after 结果。

## 2026-11-05 稠密列索引复核

### 基线说明（重要）

中断轮次保存在 `/tmp/dddmr-map-baseline` 与 `/tmp/dddmr-map-before.patch` 的副本已丢失（两路径均不存在）。仓库内仍保留：

- `log/query-perf-20261105/dddmr-map-before.patch`：只含 tracked 文件的基线 diff；
- `log/query-perf-20261105/dddmr-map-before-untracked.txt`：未跟踪文件清单；
- `log/local_map_verified_build/dddmr_local_map/benchmark_before_run{1,2,3}.csv`：2026-11-05 13:08 的旧 before 数据，30,000 个有效样本；
- `log/local_map_verified_install/dddmr_local_map/lib/libdddmr_local_map_core.a`：2026-11-04 13:18 的 Release 旧 core 静态库。

旧源码快照无法完整恢复，因此本轮没有“从旧源码重新编译”的 fresh before。为做可核验比较，本轮把上述旧 core 静态库复制到持久目录，用同一份未修改的 `benchmark_core.cpp` 重新链接出 before 基准二进制，并在同一台机器、同一 CPU 绑核、同一场景和查询序列下重跑。该 before 是“复用旧二进制库”的重跑，不是旧源码重建；其 P50/P95/P99 与 13:08 历史 CSV 接近（见下），可互相印证。历史 CSV 本身不作为本轮新结果。因此下文差值是可核验的“旧 Release 归档 vs 当前源码”对比，不是严格的旧源码到新源码加速比。

### 构建与环境

- 独立构建目录：`log/local_map_query_20261105/build`，未改动运行 install，未切换 `/home/robot/navigation_ws/install`。
- `cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON`，`cmake --build ... -j2`；无编译警告。
- 编译器：`/usr/bin/c++` (Ubuntu 11.4.0-1ubuntu1~22.04.3)；Release 标志 `-O3 -DNDEBUG`。
- 平台：Jetson Orin NX，8 核 Cortex-A78AE；未修改功耗模式，未执行锁定时钟操作（观测 governor=`schedutil`）。
- 基准进程用 `taskset -c 2` 绑定 CPU 2；同一次会话内 before/after 交替执行。

### 功能回归

| 套件 | 结果 |
| --- | ---: |
| `test_voxel_map` | 21/21 通过 |
| `test_terrain` | 42/42 通过 |
| `test_ros_adapter` | 18/18 通过 |
| `ctest`（三个套件） | 3/3 通过 |
| `tools/test_local_map_latency_report.py` | 5/5 通过 |

- 当前 42 项 `test_terrain` 也在旧 core 静态库上重编译运行，42/42 通过：新增的负坐标、滚动窗口、观测空列、free+实际 HIT、多层回归用例在修改前后都成立，属于行为保持测试。
- `test_ros_adapter` 中的 `Parameters.Shadow005ConfigLoadsFiveCentimeterWindow` 实际加载 `config/local_map_005.yaml`，确认 `voxel_size=0.05`、`window=160x160x80`、8x8x4 m，且 geometry 未设置、`control_ready` 为 false。
- 本轮没有启动导航、全局规划或控制，也没有发布速度；没有运行 shadow 的实时 bag/TF 回放。

### 有效姿态基准

同一合成场景：10,201 条射线，1.5x1.1 m 测试车体，10,000 个查询姿态；每个配置/路径跑 3 次，聚合 30,000 个样本。所有样本 `valid=1`，没有任何无效查询被计入。

两种分辨率的合成基准均沿用 `benchmark_core.cpp` 的 `inflation_radius=0.25 m`；
shadow 配置的膨胀半径为 `0.0 m`，且没有测试车体几何。下述耗时不直接代表
完整 shadow 配置的运行耗时，地图更新时间尤其受这些设置影响。

0.10 m（64x64x40，6.4x6.4x4.0 m，`max_voxels=2,000,000`）：

| 单姿态查询 | before（复用旧 core） | 本轮 dense columns | 变化 |
| --- | ---: | ---: | ---: |
| P50 | 0.094178 ms | 0.073538 ms | -21.9% |
| P95 | 0.100130 ms | 0.078466 ms | -21.6% |
| P99 | 0.103363 ms | 0.080898 ms | -21.7% |
| 平均 | 0.094262 ms | 0.073646 ms | -21.9% |
| 有效数 | 30,000/30,000 | 30,000/30,000 | - |

0.10 m 使用 benchmark 既定窗口，不是生产默认的 12.8x12.8x4.0 m；0.05 m 变体沿用 benchmark 的 `inflation_radius=0.25`，与 `local_map_005.yaml` 的 `0.0` 不同。两者仅用于分辨率对照。

0.05 m 离线对照（160x160x80，8x8x4.0 m，`max_voxels=2,500,000`）：

| 单姿态查询 | before（复用旧 core） | 本轮 dense columns | 变化 |
| --- | ---: | ---: | ---: |
| P50 | 0.467436 ms | 0.367305 ms | -21.4% |
| P95 | 0.500301 ms | 0.389545 ms | -22.1% |
| P99 | 0.521197 ms | 0.401290 ms | -23.0% |
| 平均 | 0.467687 ms | 0.366863 ms | -21.6% |
| 有效数 | 30,000/30,000 | 30,000/30,000 | - |

历史 13:08 before CSV（仅用于印证，不作为本轮 before）：P50 0.094467 ms、P95 0.101284 ms、P99 0.113925 ms、30,000/30,000 有效，与本轮复用的 before 二进制一致。

地图更新耗时（`integrateRays`，包含体素更新、快照和地形缓存重建）：

| 配置 | before 3 次均值 | 本轮 3 次均值 |
| --- | ---: | ---: |
| 0.10 m | 52.386 ms | 52.646 ms |
| 0.05 m | 854.812 ms | 886.447 ms |

0.05 m 更新耗时约 0.85-0.93 s，明显高于 0.10 m；本轮查询加速不改变这一构建成本。

### 结论与限制

- 在合成场景中，当前实现相对保留旧库的单姿态查询 P50/P95/P99 低约 21-23%，且 120,000 个查询样本（两种分辨率 × before/after × 3 次）全部有效。由于旧源码快照缺失，这不是严格的旧源码重建到新源码的加速比。
- 这只证明查询路径本身；按 0.0736 ms × 96 个候选 × 约 75 步的工作量估算约 0.53 s/周期，仍远高于 50 ms，不能据此宣称满足 15 Hz 或 50 ms 控制周期。该数字是乘法估算，不是完整控制周期测量。
- 0.10 m 基准窗口是 6.4x6.4x4.0 m，不是生产默认 12.8x12.8x4.0 m；0.05 m 变体沿用 benchmark 的 inflation 0.25，而 `local_map_005.yaml` 是 0.0。不要把本表当作完整 shadow profile 或生产配置的端到端基准。
- 未使用真实雷达 bag、未验证完整导航链、未做实车接触/落差/坡道验收；0.05 m 配置仅供离线 shadow，不是控制配置。
- 原始数据：`log/local_map_query_20261105/{before,after}/`、`logs/benchmark_stats.{txt,json}`、`logs/update_stats.txt`；其余命令输出保存在同目录的 `logs/` 中。

## 2026-11-04 历史记录（上一轮优化，勿与本轮数据混用）

以下为 2026-11-04 原记录；其“优化前/优化后”属于上一轮稀疏索引优化，不是本轮的 dense-column before/after，也不能与上表直接拼接。

在 ROS 2 Humble 下，以 Release 构建 `dddmr_local_map`、`local_planner`、`p2p_move_base`；使用隔离的 `log/local_map_verified_build` 和 `log/local_map_verified_install`，限制构建并行数为 2。

- 地图、地形和 ROS 适配三个测试套件共 77 项通过。
- 新增 MPPI 轨迹／健康门禁 21 项、原有 MPPI 124 项通过。
- 新配置及 launch 参数覆盖 5 项、性能统计工具 5 项通过。
- 原有感知回归 28 项通过。
- 原有 p2p 测试保留三项既有失败：两项地图路径断言与用户原有配置不一致，一项要求的 YAML 注释标记在原 HEAD 中已不存在。未修改这些用户文件或测试以消除失败。
- 用户已有三处文件修改与保存的基线补丁逐字节一致。

隔离 ROS 域的 shadow 启动验证通过：完整地面云生成 `odom` 坐标系下的占据输出；停止输入后约 0.6 s 发布空输出，清除过期显示。测试期间没有 `/cmd_vel` 发布者，未启动全局规划或导航节点，结束后清理诊断进程。

## Orin NX 合成性能

平台：Jetson Orin NX，MAXN 模式；没有修改功耗模式或锁定时钟。

同一合成场景为 6.4 × 6.4 × 4.0 m、0.1 m 体素、10,201 条垂直射线，使用 1.5 × 1.1 m 的测试车体，查询 10,000 个有效姿态。它不是实际雷达采集，也不等同于默认 12.8 × 12.8 × 4.0 m 地图及实车包络。

| 单个位姿查询 | 优化前 | 优化后 |
| --- | ---: | ---: |
| 平均 | 0.967 ms | 0.0958 ms |
| P95 | 1.019 ms | 0.1015 ms |
| P99 | 1.169 ms | 0.1048 ms |
| 有效查询 | 10,000/10,000 | 10,000/10,000 |

优化跳过已知空体素的几何计算，并按地图帧索引可能影响净空的体素；地面、高度上界、未知／过期策略及逐姿态检查范围不变。

优化后单次 `LocalMap::integrateRays` 约 48.8 ms，**包含体素更新、快照和地形缓存重建**，不是单独的射线耗时。

当前配置约 96 条候选 × 75 步。仅将单姿态均值相乘，地形查询就约需 **0.69 s/周期**，尚未包括初始姿态、路径元数据、最终序列复验及其他控制开销。这是工作量估算，不是完整控制周期测量；不能据此宣称满足 50 ms 预算或 15 Hz。大量轨迹因未知区域提前拒绝，也不能用来证明完整有效候选的吞吐。

原始数据在仓库 `log/local_map_verified_build/dddmr_local_map/benchmark_queries.csv`、`benchmark_queries_opt.csv`；构建、测试、优化分段测量和 shadow 结果保存在同一 `log/` 目录。统计工具见 `tools/LOCAL_MAP_PERFORMANCE.md`。

## 尚待验收

- 测量附件最高点并核对前后悬伸、base 高度；当前 `body_max` 故意未配置，新控制后端不会就绪。
- 继续处理整轮查询吞吐，使用真实输入测量端到端延迟、CPU、内存和 deadline miss。
- 用含完整去畸变云、TF 和里程计的数据回放，验证平地、坡脚／坡顶、低障碍、悬空物及落差。
- 核对实车轮胎接触区的可观测性；未观测区域不会由远处平面填充。

在这些条件满足之前，使用 shadow 进行诊断，不将新后端切入实车控制。
