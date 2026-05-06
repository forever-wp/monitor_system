# Collision Voxel Layer Config Alignment Design

**Context**

`collision_voxel_layer` already provides the fused voxel topic consumed by `nav2_monitor`, but it still behaves like a standalone package demo:
- launch defaults read package-local config
- runtime config is not mirrored under `config/Monitor/`
- there is no file-based hot reload path like `bridge` and `nav2_monitor`

The monitor worktree should treat it as a first-class runtime module.

**Goals**

- Keep `collision_voxel_layer` as an independent package with a clear topic contract.
- Store the authoritative runtime config under `config/Monitor/collision_voxel_layer/`.
- Make the node support both ROS 2 standard parameter updates and file-based hot reload.
- Keep `nav2_monitor` coupled only through `collision_detection.voxel_topic`.
- Preserve the existing rule that the control worktree shares the same code and differs only in config entry paths.

**Design**

- Add a monitor-side runtime config bundle at `config/Monitor/collision_voxel_layer/collision_voxel_layer_params.yaml`.
- Change the monitor launch file to load `/opt/ry/config/Monitor/collision_voxel_layer/collision_voxel_layer_params.yaml`.
- Extend `CollisionVoxelLayerNode` with:
  - `config_file`, `config_reload_enabled`, `config_reload_period_s`
  - a filesystem mtime watcher for the config file
  - an `on_set_parameters_callback` for live parameter validation and reconfiguration
- Reconfigure publishers, subscriptions, synchronizer, timers, and voxel grid in place when parameters change.
- Resolve relative `config_file` values against the package share directory so package-local configs still work in the control worktree.

**Behavior Notes**

- File reload uses ROS 2 parameter file parsing and applies values through standard parameter updates.
- Invalid file reloads log an error and keep the current live configuration.
- Reconfiguring structural parameters resets the in-memory voxel grid to avoid mixing occupancy built from incompatible settings.

**Verification**

- Add node tests covering forced reload from a temp YAML file and runtime parameter updates.
- Extend monitor OTA layout tests to verify the new runtime config bundle and launch path.
- Build and test `collision_voxel_layer` and `nav2_monitor` in both worktrees after sync.
