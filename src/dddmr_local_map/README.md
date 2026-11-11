# dddmr_local_map

## Quick start (standard workspace)

```bash
source /opt/ros/humble/setup.bash
source /home/robot/navigation_ws/install/setup.bash
# Self-contained demo, RViz on by default:
ros2 launch dddmr_local_map local_map_demo.launch.py
# Headless:
ros2 launch dddmr_local_map local_map_demo.launch.py use_rviz:=false
# Real sensor input, RViz on by default:
ros2 launch dddmr_local_map local_map.launch.py
# Real sensor input, headless:
ros2 launch dddmr_local_map local_map.launch.py use_rviz:=false
# Diagnostic 0.05 m demo using the installed profile:
CONFIG="$(ros2 pkg prefix dddmr_local_map)/share/dddmr_local_map/config/local_map_005.yaml"
ros2 launch dddmr_local_map local_map_demo.launch.py config_file:="$CONFIG" use_rviz:=false
```

The demo starts the same `local_map_node` mapping loop as the real-input launch.
It publishes synthetic `odom <- demo_base_link <- demo_lidar` TF, never a
command or velocity, and is not a field-acceptance result. The real-input
`local_map.launch.py` and the installed YAML profiles keep the vehicle
`base_link` frame unchanged. The real-input launch opens
`rviz/local_map.rviz` by default; its Fixed Frame follows the effective map frame
(`map_frame` override, otherwise `local_map.frames.map` from `config_file`) and
its relative diagnostic topics follow the shared namespace. Pass
`rviz_config:=/abs/path.rviz` to replace the view, or `use_rviz:=false` on a
headless host.

Pure C++ bounded 3-D local occupancy map, layered terrain surface cache, and
rigid-body traversability core, plus a ROS 2 adapter and a standalone shadow
diagnostic node. This README covers the ROS adapter. The core stays independent
of ROS.

**Acceptance status (2026-11-05):** shadow mapping and software regressions
pass; the new backend has not passed real-time or field acceptance. In an
isolated Release rebuild on the Orin NX, the frozen dense-column query path
reduced a synthetic single-pose query at 0.10 m from P50/P95/P99
0.094/0.100/0.103 ms to 0.074/0.078/0.081 ms, and in the synthetic 0.05 m
comparison from 0.467/0.500/0.521 ms to 0.367/0.390/0.401 ms. The comparison
uses a preserved pre-change core library; the exact old source snapshot was
not recoverable. Both synthetic benchmarks use 0.25 m inflation, while the
shadow profiles use 0.0 m. All 10,000 poses per run were valid. These are
pose-query timings only, not a complete control
cycle. At 96 candidates and about 75 steps the query cost still estimates far
beyond the 50 ms budget, and the vehicle attachment height is unmeasured.
Keep the new backend out of vehicle control until these gates pass. The
2026-11-04 record and its older optimization numbers are kept in
[VALIDATION.md](VALIDATION.md), clearly separated from this round. The source
package includes `IMPLEMENTATION_PLAN.md` and `VALIDATION.md` with details and
raw-log paths.

## Standalone demo (diagnostic only)

The package installs three executables: `local_map_node` (real input),
`local_map_shadow` (compatibility), and `local_map_demo_scene` (synthetic
source). The demo launch runs the same `local_map_node` mapping loop, a
deterministic synthetic ground/box cloud, static `odom <- demo_base_link <-
demo_lidar` TF, and optionally RViz. `demo_base_link` is a demo-only child so
the synthetic source does not collide with a real vehicle `base_link` TF in the
same ROS domain. It publishes no vehicle control and is not a navigation or
field-acceptance result.

From a shell that already has the standard workspace available:

```bash
source /opt/ros/humble/setup.bash
source /home/robot/navigation_ws/install/setup.bash
ros2 launch dddmr_local_map local_map_demo.launch.py
```

RViz is on by default. For headless operation or CI:

```bash
ros2 launch dddmr_local_map local_map_demo.launch.py use_rviz:=false
```

The formal real-input entry is separate; it does not start the synthetic source.
It starts `local_map_node` plus the real-input RViz view by default:

```bash
ros2 launch dddmr_local_map local_map.launch.py
```

For a headless host or CI, disable only the view:

```bash
ros2 launch dddmr_local_map local_map.launch.py use_rviz:=false
```

`rviz/local_map.rviz` shows only TF, `local_map/occupancy`, and
`local_map/surface_candidates`. All diagnostic topics are relative, so the RViz
instance uses the same `namespace` as `local_map_node`; it does not subscribe to
the demo's `/local_map_demo/points`. The real cloud topic stays in `config_file`
and is not hard-coded in the RViz file. The launch passes `-f` with the effective
map frame (`map_frame` override, otherwise `local_map.frames.map`) so the view
matches the map even when the launch override differs from the YAML. Override the
whole view with `rviz_config:=/abs/path.rviz` when needed. The existing demo
launch keeps its own RViz window, custom-topic remap, and `demo_base_link`; it
passes `use_rviz:=false` to the included formal entry so two RViz processes are
never started. `local_map_shadow.launch.py` remains unchanged and headless.

Useful demo arguments: `demo_cloud_topic` (absolute synthetic topic, default
`/local_map_demo/points`), `sensor_frame` (default `demo_lidar`),
`demo_azimuth_samples` (default `180`), `demo_range_samples` (default `24`),
`use_sim_time`, and `config_file`. When `demo_cloud_topic` is overridden, the
RViz input subscription is remapped from `/local_map_demo/points` to the chosen
topic, and `local_map_node` subscribes to that same topic.

The synthetic source defaults to 180 x 24 = 4,320 rays per frame. This is a
demo-input choice, not a map rule: on the Orin NX the former 480 x 72 = 34,560
rays took about 0.7 s per update at 0.05 m, beyond the unchanged 0.5 s age
limit, so the node reported `stale_stamp` and published empty latched
diagnostics. The reduced default keeps the unchanged age/geometry checks and
produces non-empty occupancy and surface candidates with the unchanged 0.10 m
profile and the diagnostic 0.05 m profile. Do not raise the age limit or change
the profile merely to make the demo pass.

## Build and source

The package builds with `ament_cmake` and C++17. From a normal ROS 2 workspace:

```bash
colcon build --packages-select dddmr_local_map
source install/setup.bash
```

The exported targets are `dddmr_local_map::dddmr_local_map_core` and
`dddmr_local_map::dddmr_local_map_ros`. The ROS library links `rclcpp`,
`tf2_ros`, `sensor_msgs`, and `geometry_msgs`; `local_map_shadow` and
`test_ros_adapter` link it. Both installed libraries are position-independent,
and the node is installed under `lib/dddmr_local_map` so `ros2 run`/`launch`
can find it. Do not add a dependency on `dddmr_p2p_move_base`; it would create
a cycle.

## Shadow node

```bash
ros2 launch dddmr_local_map local_map_shadow.launch.py
# optional:
#   config_file:=/path/local_map.yaml use_sim_time:=true namespace:=demo
```

Default inputs and window:

- cloud input: `/lio_sam/deskew/cloud_deskewed`
- sensor frame: `hesai_lidar`, map frame: `odom`, base frame: `base_link`
- map window: 12.8 x 12.8 x 4.0 m at 0.1 m voxels
- cloud QoS: `SensorDataQoS`, depth 1
- callback group: dedicated mutually exclusive group

The shadow process owns its `tf2_ros::TransformListener` and buffer. It never
publishes a command or velocity. Diagnostics are published only when the
corresponding topic has subscribers:

- `local_map/occupancy` - occupied voxel centers.
- `local_map/surface_candidates` - raw occupied-run tops per column. This is a
  layered raw surface diagnostic, not a drivable-support map and not a global
  lowest-return map. Actual traversability comes from the cached
  `TerrainModel` in the handle.

Both topics are in the snapshot map frame and use transient-local QoS. Their
names are relative, so `namespace:=demo` publishes them under
`/demo/local_map/occupancy` and `/demo/local_map/surface_candidates`; the
real-input RViz view is started in that same namespace. When the map is empty or
unhealthy, the node publishes an empty cloud on a subscribed diagnostic topic so
a previously latched sample is cleared.

## Offline 0.05 m comparison profile

`config/local_map_005.yaml` is a diagnostic-only profile: 0.05 m voxels,
8 x 8 x 4 m window, 160 x 160 x 80 cells, and `max_voxels=2500000`. It is not
loaded by any navigation launch and is not wired into p2p or the controller.
Geometry is intentionally omitted, so `AdapterStatus.control_ready` stays
false. Do not treat it as a control-ready configuration.

```bash
# config_file must be an absolute path; run only the diagnostic shadow node.
CONFIG="$(ros2 pkg prefix dddmr_local_map)/share/dddmr_local_map/config/local_map_005.yaml"
ros2 launch dddmr_local_map local_map_shadow.launch.py config_file:="$CONFIG"
```

The profile is for offline/shadow replay comparisons only. It has 2,048,000
cells. A synthetic benchmark using the same window but 0.25 m inflation and
test vehicle geometry took about 0.85-0.93 s per map update; this profile uses
0.0 m inflation and unset geometry, so that timing does not directly measure
this profile. See `VALIDATION.md` for the benchmark settings. The window must
cover the stopping and avoidance distances for the intended speed before any
future control use. The default `config/local_map.yaml` remains
0.10 m and is unchanged. The parameter-load test verifies the 0.05 m voxel,
160 x 160 x 80 window, and unset geometry; it does not run navigation or a
real vehicle.

## Reusable in-process adapter

The planner keeps its own TF buffer and passes it in:

```cpp
#include "dddmr_local_map/ros_adapter.hpp"

dddmr_local_map::ros::AdapterConfig config;
auto status = dddmr_local_map::ros::loadAdapterConfig(node, "local_map.", &config);
if (!status.ok) {
  // report status.error; do not silently fall back
}

dddmr_local_map::ros::LocalMapRosAdapter adapter(node, tf_buffer, config);
```

The full constructor creates the cloud subscription immediately. The
controller must take one handle per planning cycle and use the snapshot and
terrain from that same handle:

```cpp
const auto handle = adapter.handle();
if (!handle || !handle->mapHealthy()) {
  // health reason: handle ? handle->status.reason : "no bundle yet"
  return;
}
if (!handle->readyForControl()) {
  // map may be usable for diagnostics, but geometry is unset or terrain failed
}
const auto snapshot = handle->snapshot;
const auto terrain = handle->terrain;
```

`AdapterHandle` is immutable and contains:

- `snapshot`, `terrain`
- `measurement_stamp` (cloud acquisition), `receive_stamp` (ROS),
  `receive_steady`
- `version` (monotonic across resets), `generation`, `mapping_ms`
- `status` with `{code, map_healthy, control_ready, reason}`

## Health and safety rules

- Zero, non-finite, future, stale, non-monotonic, and wrong-frame acquisition
  stamps are rejected.
- Empty clouds, malformed fields, and clouds with no valid rays are rejected.
- TF lookups use `map<-sensor` and `map<-base` at the acquisition stamp. A TF
  failure never publishes a fresh healthy bundle.
- Acquisition age is rechecked after map integration and again after terrain
  construction, using both the ROS stamp and steady-clock receipt time. A
  frozen `/clock` cannot hide a slow update or sensor loss.
- Updates are serialized. Exactly one snapshot and one terrain cache are built
  after integration; recentering does not publish an intermediate bundle.
- An invalid terrain build status keeps the raw map diagnostics available but
  forces `control_ready=false`. An unhealthy map is never reported healthy.
- A ROS clock rewind recreates the map, increments `generation`, increments the
  never-reset `version`, and never reuses old history for a new bag.
- Invalid or unset vehicle geometry does not throw and does not stop shadow
  mapping. It makes `control_ready=false` and disables self filtering.
- Status logging is throttled. No metrics file is written; use
  `handle->mapping_ms` for external logging.

## Filtering and map window

The adapter transforms the full cloud into the map frame and removes only:

1. non-finite points, and
2. points strictly inside the configured rigid-body AABB, with
   `body_min.z < z <= body_max.z`. Points at or below `body_min.z` are retained
   so the floor, ramps, and wheel contact are not removed.

There is no height/non-ground crop and no adapter-side max-range clip; the core
owns max ray clipping. Production conversion uses
`WindowRayPolicy::kKeepCrossingRays`: an endpoint outside the local window is
kept when the sensor-to-endpoint segment crosses the window. The core then
clips the finite ray to max range/window and does not mark the out-of-window
endpoint as a boundary hit. The default converter policy still drops
out-of-window endpoints for callers that explicitly want that behavior.

## Geometry parameters

All geometry is in the base frame and must describe the actual vehicle.
User-supplied dimensions and remaining measurements are recorded in
[VEHICLE_GEOMETRY.md](VEHICLE_GEOMETRY.md). Synthetic values in unit tests are
test fixtures, not a vehicle specification.

- `geometry.wheel_contacts`: exactly 12 numbers, four `x,y,z` contacts ordered
  front-left, front-right, rear-left, rear-right. Each contact z must equal
  `-geometry.base_contact_height`.
- `geometry.body_min`, `geometry.body_max`: three numbers each, defining the
  base-frame axis-aligned rigid-body envelope. Each `body_max` component must
  be strictly greater than the matching `body_min` component.
- `geometry.base_contact_height`: positive wheel-contact height above the
  support plane.
- `geometry.body_clearance`: non-negative required ground clearance.

Self filtering is derived only from the validated `body_min`/`body_max` AABB.
The optional `self_filter.enabled` parameter defaults to true but has no effect
until all geometry validates. If geometry is unset, the shadow map runs with
self filtering disabled. Never fill these fields with guessed values.

## Parameters

All parameters are under `local_map.*`; `local_map.frames.map` replaces a shared
root `global_frame`. `config/local_map.yaml` is a valid ROS parameter file with
the `/** ros__parameters:` wrapper and contains the complete shadow defaults.
Geometry is intentionally omitted there for shadow operation.

## Offline testing

`LocalMapRosAdapter(AdapterConfig)` creates no node or subscription, and
`ingestCloudWithTransforms(...)` takes supplied Eigen `map<-sensor` /
`map<-base` transforms. `pointCloudToMapPoints`, `validateAcquisitionStamp`,
`validateCloudFrame`, and the Eigen transform helpers are pure and testable
without DDS or a network. The gtest suite does not spin a live ROS node.

## Query performance (2026-11-05)

Isolated Release rebuild (`-O3 -DNDEBUG`, GCC 11.4.0, Orin NX, no power-mode
change), same synthetic scene and 10,000 valid query poses per run, three
measured runs aggregated. "Pre-change" is a benchmark binary linked against
the preserved pre-dense Release core archive; it is not a fresh source
baseline (the `/tmp` baseline source copy was lost). See
[VALIDATION.md](VALIDATION.md) for the full record, raw paths, and limits.

| Voxel / window | Path | P50 (ms) | P95 (ms) | P99 (ms) | Valid |
| --- | --- | ---: | ---: | ---: | ---: |
| 0.10 m / 6.4x6.4x4.0 m | pre-change | 0.0942 | 0.1001 | 0.1034 | 30,000/30,000 |
| 0.10 m / 6.4x6.4x4.0 m | dense columns | 0.0735 | 0.0785 | 0.0809 | 30,000/30,000 |
| 0.05 m / 8x8x4.0 m | pre-change | 0.4674 | 0.5003 | 0.5212 | 30,000/30,000 |
| 0.05 m / 8x8x4.0 m | dense columns | 0.3673 | 0.3895 | 0.4013 | 30,000/30,000 |

These are per-pose query times. They are not a full MPPI control cycle, not a
50 ms deadline measurement, and not a real-vehicle result. The 0.05 m profile
is offline/shadow only.

## MPPI controller integration

- Set `local_planner.controller_backend: "mppi"` and
  `local_planner.mppi.environment_backend: "local_map"` to consume this adapter
  in-process. The only accepted environment values are `legacy_points`
  (default) and `local_map`.
- The adapter is constructed only for that exact combination. A map/parameter
  or measured-geometry validation failure stops command output; there is no
  silent fallback to the legacy point cloud path.
- The controller takes one immutable `AdapterHandle` per control cycle, checks
  ROS stamp age and steady receipt age before and after optimization, and does
  not mix a newer handle into the already pinned sequence.
- `p2p_move_base/launch/p2p_move_base_local_map.launch.py` is a separate opt-in
  localization entry. Its navigation nodes explicitly load the baseline YAML,
  then `p2p_move_base/config/p2p_move_base_local_map.yaml`, then launch-specific
  time/map overrides. This order also overrides existing wheelbase keys. The
  original localization launch and configuration remain unchanged.
- User-supplied dimensions and the remaining measurement items are recorded in
  [VEHICLE_GEOMETRY.md](VEHICLE_GEOMETRY.md). The current authoritative
  Hunter SE driver geometry is wheelbase 0.550 m, track 0.520 m, virtual
  central steering limit 0.3270520925102273 rad (22 deg inner wheel), with
  approximate base height 0.220 m and chassis ground clearance 0.120 m.
  Body x extents are a symmetry approximation that still requires checking
  against the actual overhangs. The older 0.460/0.550 calibration is
  superseded by the 2026-11-05 user directive.
- The chassis height 0.310 m excludes rigid attachments. Their highest point
  is unmeasured, so `geometry.body_max` is intentionally omitted. Complete
  that envelope before enabling control; the supplied overlay will report
  geometry not ready until it validates.

Build all three affected packages together before using this backend, so the
planner and its hosting process use matching interfaces:

```bash
source /opt/ros/humble/setup.bash
cd /home/robot/navigation_ws
colcon build --packages-up-to p2p_move_base --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
# Diagnostic mapping only; does not publish velocity:
ros2 launch dddmr_local_map local_map_shadow.launch.py
```

The opt-in navigation entry is `ros2 launch p2p_move_base
p2p_move_base_local_map.launch.py`. It starts navigation processes and should
only be used after geometry and offline/field acceptance. Existing navigation
launch commands continue to select the legacy environment by default.

## Implementation and acceptance

The implementation separates circular occupancy updates, per-map-frame
measured-surface caching, cycle-pinned MPPI rollouts, and final command
validation. It keeps the existing global PCD/Hybrid A*/dGraph path. It does not
implement GNSS/AMap global mapless navigation.

Synthetic tests cover rolling storage, observation expiry, ground/support
geometry, low and overhanging obstacles, unknown areas, pose consistency and
launch parameter precedence. Recorded-scene replay and vehicle acceptance
still need flat ground, slope transitions, curbs, dropoffs and overhead
obstacles. Spatial resolution, sensor noise and the rigid four-wheel model
limit what can be inferred; it is not a suspension/contact dynamics model.
Unknown support is rejected, including at startup. A LiDAR blind region under
the wheels is not filled from a distant plane or treated as observed ground;
the installed sensor layout and initialization must provide sufficient contact
region observations. Diagnose this in shadow mode before navigation tests.

Performance measurements must distinguish mapping, pose queries and complete
control latency. The repository tool
`tools/local_map_latency_report.py` reports CSV P95/P99 and deadline misses;
see `tools/LOCAL_MAP_PERFORMANCE.md`. A synthetic benchmark is not a substitute
for the complete navigation chain with real sensor data.

