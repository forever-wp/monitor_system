# Collision Voxel Layer Design

## Context

The current collision pipeline in `nav2_monitor` consumes obstacle evidence as a flat set of `xy`
points from `LaserScan`, `PointCloud2`, and ultrasonic sources. This is simple, but it has two
practical weaknesses for real-vehicle collision detection:

- low obstacles are not reliably covered by 2D lidar alone
- noisy depth points can create abrupt trigger / recover oscillations if injected directly as raw
  points

We want a new independent package that borrows the useful ideas from
`spatio_temporal_voxel_layer`, but does not depend on Nav2 costmap plugins or navigation runtime.
Its purpose is to provide a unified, temporally stable obstacle representation for collision
evaluation.

## Goals

- Build an independent ROS 2 package for multi-sensor obstacle fusion.
- First version uses `LaserScan + depth PointCloud2` as the main inputs.
- Internally maintain a sparse voxel layer with temporal decay.
- Publish a unified voxel occupancy result for downstream collision detection.
- Let `nav2_monitor` consume voxel occupancy directly instead of raw merged points.
- Reduce noise-driven trigger / recover jumps compared with direct raw point fusion.

## Non-Goals

- Do not implement Nav2 costmap plugin compatibility in the first version.
- Do not make 3D mapping or global environment reconstruction a goal.
- Do not require STVL as a dependency.
- Do not make ultrasonic or 3D lidar first-class inputs in v1.

## Constraints

- Worktree code is the source of truth.
- Before implementation starts, current engineering state must be saved so work can be rolled back.
- The solution should remain usable even when navigation is not running.
- Existing collision zone / TTC semantics in `nav2_monitor` should be preserved as much as
  possible.

## Recommendation

Create a new package: `collision_voxel_layer`.

`collision_voxel_layer` is responsible for sensor synchronization, source preprocessing, sparse
voxel maintenance, decay, and voxel publishing.

`nav2_monitor` remains responsible for collision semantics:

- polygon zone triggering
- TTC triggering
- fault latching / recovery
- safety command selection

This keeps the world-model concern separate from the collision-policy concern.

## Why This Approach

Compared with directly fusing raw point clouds:

- temporal decay smooths short dropouts and isolated noise
- low obstacles from depth can persist long enough to be useful without causing one-frame spikes
- 2D lidar remains a stable primary source for forward geometry

Compared with embedding a voxel layer inside `nav2_monitor`:

- the package stays reusable
- debugging is much easier
- future sensor expansion is cleaner

Compared with directly reusing STVL:

- we avoid Nav2 costmap coupling
- we keep only the subset of behavior needed for collision detection

## Inputs

### v1 Main Inputs

- `sensor_msgs/msg/LaserScan`
- `sensor_msgs/msg/PointCloud2` for depth-derived cloud
- `tf2`

### Reserved for Future

- additional `PointCloud2` sources, including 3D lidar
- ultrasonic input

## High-Level Data Flow

1. Synchronize `LaserScan` and depth cloud using ROS 2 `message_filters`.
2. Transform both inputs into `base_link`.
3. Preprocess each source independently.
4. Insert evidence into a sparse voxel grid.
5. Decay stale voxels over time.
6. Publish the current voxel occupancy state.
7. Let `nav2_monitor` subscribe to the voxel state and run the existing collision logic on it.

## Package Structure

### `sync_manager`

Responsibilities:

- subscribe to `scan` and `depth_cloud`
- use `ApproximateTime` synchronization
- pass synchronized pairs to the update pipeline

Reasoning:

- official ROS 2 `message_filters` is preferred over archived third-party forks
- approximate sync is more realistic for real robots than exact sync

### `source_adapter`

Responsibilities:

- transform sensor data into `base_link`
- apply range and height filters
- drop invalid points
- optionally downsample depth input before voxel insertion

Rules:

- `LaserScan` hits are expanded into configurable vertical columns
- depth cloud is used as near-field low-obstacle reinforcement

### `sparse_voxel_grid`

Responsibilities:

- maintain sparse occupied voxels via hashed indexing
- store occupancy strength and timestamps
- store source provenance

Each voxel keeps:

- center index
- occupancy value
- last seen time
- source bitmask

### `decay_engine`

Responsibilities:

- periodically decay occupancy of stale voxels
- remove voxels below threshold

Chosen strategy for v1:

- time decay only
- no raytracing clearing in v1

### `voxel_exporter`

Responsibilities:

- publish the authoritative voxel occupancy message
- publish visualization markers
- optionally publish debug cloud output

## Sensor Modeling

### 2D Lidar

2D lidar is the stable geometric backbone in v1.

Each hit is projected to `base_link`, then expanded into a vertical column:

- configurable `scan_z_min`
- configurable `scan_z_max`
- configurable voxel height resolution

Why:

- this gives 2D lidar a meaningful 3D occupancy footprint
- it lets lidar and depth coexist in the same voxel representation
- collision logic downstream can treat both as one evidence field

### Depth Cloud

Depth is the low obstacle supplement in v1.

Depth processing should be stricter than lidar:

- shorter max range
- tighter height limits
- optional prefilter voxel downsampling
- lower trust outside the near field

Why:

- depth is useful for low obstacles
- depth is also much noisier in reflective / low-texture / difficult lighting conditions

## Occupancy Update Model

For each observation:

- locate the voxel cell
- add source-specific occupancy weight
- update `last_seen`
- update `source_mask`
- clamp occupancy to a configured max

Suggested initial source behavior:

- lidar: stable medium-weight evidence
- depth: strong near-field evidence, but only within configured trusted range

The first version should keep this simple and parameterized rather than trying to infer sensor
reliability dynamically.

## Decay Model

Use exponential decay in v1.

Rationale:

- smooth reduction of stale evidence
- more stable under intermittent sensor dropout
- reduces abrupt recoveries better than linear decay

Core parameters:

- `voxel_decay_time_s`
- `prune_threshold`
- `publish_rate`

## External Interface

### New Package Messages

Create package-local messages:

#### `collision_voxel_layer/msg/VoxelCell.msg`

- `float32 x`
- `float32 y`
- `float32 z`
- `float32 occupancy`
- `uint8 source_mask`

#### `collision_voxel_layer/msg/VoxelGrid.msg`

- `std_msgs/Header header`
- `float32 resolution_xy`
- `float32 resolution_z`
- `float32 decay_time_s`
- `VoxelCell[] cells`

Why not `PointCloud2`:

- downstream needs occupancy strength, not just point coordinates
- source provenance is useful for debugging and later policy refinement

### Debug Topics

- `/collision_voxel_layer/grid`
- `/collision_voxel_layer/markers`
- `/collision_voxel_layer/debug_cloud`

## `nav2_monitor` Integration

### Config Changes

Extend `collision_detection` config with:

- `voxel_topic`
- `voxel_min_occupancy`
- optional `voxel_min_height`
- optional `voxel_max_height`

### Runtime Changes

Add voxel subscription and storage in `nav2_monitor`.

`MonitorDataStore` gains voxel state alongside existing raw source state.

`CollisionEvaluator` gains a voxel-backed obstacle path:

- `ZONE`
  - sum occupancies of voxels whose centers fall in the polygon
  - existing `min_points` effectively becomes a minimum occupancy weight threshold
- `TTC`
  - use voxel centers as obstacle candidates
  - filter out cells below `voxel_min_occupancy`
  - reuse the existing TTC trajectory logic as much as possible

### Compatibility Rule

If `voxel_topic` is configured and active, it becomes the preferred collision evidence source for
zone / TTC evaluation.

Raw `scan_topic` / `pointcloud_topic` / `ultrasonic_topic` remain for compatibility, but first
version integration should prefer one authoritative source path to avoid double counting.

## Parameters

### Common

- `base_frame`
- `publish_rate`
- `voxel_size_xy`
- `voxel_size_z`
- `voxel_decay_time_s`
- `prune_threshold`
- `occupancy_max`

### Lidar Source

- `scan_topic`
- `scan_min_range`
- `scan_max_range`
- `scan_z_min`
- `scan_z_max`
- `scan_weight`

### Depth Source

- `depth_cloud_topic`
- `depth_min_range`
- `depth_max_range`
- `depth_min_height`
- `depth_max_height`
- `depth_weight`
- `depth_voxel_prefilter`

### `nav2_monitor`

- `collision_detection.voxel_topic`
- `collision_detection.voxel_min_occupancy`
- `collision_detection.voxel_min_height`
- `collision_detection.voxel_max_height`

## Failure Handling

### Missing Sensor

- keep publishing decayed voxel state
- do not immediately clear all occupancy
- surface source health in logs and status

### TF Failure

- reject that source update
- keep previous voxel state decaying naturally
- throttle warnings

### Synchronization Miss

- allow approximate sync window
- optionally fall back to latest-available per-source update policy in a later version

## Verification Plan

### Stage 1: New Package Only

- compile package
- unit test voxel insertion and decay
- replay bag data and inspect voxel markers
- verify low obstacles appear more consistently than lidar-only
- verify isolated depth noise decays without instant stop / resume oscillation

### Stage 2: Integrate with `nav2_monitor`

- add voxel subscription path
- unit test zone occupancy accumulation
- unit test TTC candidate generation from voxels
- replay bag data with existing collision zones
- compare trigger / recover stability against current raw-point pipeline

## Rollback / Safety Plan

Before implementation:

1. inspect current worktree status
2. create a clear git save point for rollback
3. implement `collision_voxel_layer` separately first
4. integrate `nav2_monitor` in a later isolated commit

This gives two rollback levels:

- revert only `nav2_monitor` voxel integration
- revert the entire voxel-layer feature line

## Open Decisions Already Resolved

- use an independent package instead of Nav2 plugin reuse
- use time-decay rather than raytracing clearing in v1
- use `LaserScan + depth PointCloud2` as the first main input set
- model lidar hits as vertical columns in the voxel layer
- let `nav2_monitor` consume voxel occupancy directly instead of fused points

## References

- Nav2 STVL tutorial: https://docs.nav2.org/tutorials/docs/navigation2_with_stvl.html
- STVL source: https://github.com/SteveMacenski/spatio_temporal_voxel_layer
- ROS 2 `message_filters`: https://docs.ros.org/en/humble/p/message_filters/
- ROS 2 ApproximateTime tutorial:
  https://docs.ros.org/en/ros2_packages/rolling/api/message_filters/doc/Tutorials/Approximate-Synchronizer-Cpp.html
- Archived Intel fork noted during exploration:
  https://github.com/intel/ros2_message_filters
