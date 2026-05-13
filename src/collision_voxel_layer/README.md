# collision_voxel_layer

Sparse voxel obstacle fusion package for collision detection.

项目级数据链路见 [项目架构与数据链路](../../docs/project_architecture.html)。

Runtime config follows the Monitor OTA layout and should use the deployed `/opt/ry` file as
the single source of truth on robot.

- Repository runtime source: `config/Monitor/collision_voxel_layer/collision_voxel_layer_params.yaml`
- Deployed runtime path: `/opt/ry/config/Monitor/collision_voxel_layer/collision_voxel_layer_params.yaml`
- Package fallback config: `src/collision_voxel_layer/config/collision_voxel_layer_params.yaml`
  also points `config_file` at the deployed `/opt/ry` path for hot reload.

The node supports:

- ROS 2 standard parameter updates through the built-in parameter service
- File-based hot reload through `config_file`, `config_reload_enabled`, and `config_reload_period_s`
- Independent `/scan` and depth `PointCloud2` inputs. If one source is missing, stale, or has no
  publishers, the node logs a throttled warning and still publishes voxels from any available source.
- Depth cloud prefers TF to `base_frame`; if the TF tree is disconnected for the configured
  `depth_source_frame`, the node can fall back to the calibrated extrinsic parameters and keep
  producing sparse obstacle points.
- Runtime source health on `/collision_voxel_layer/source_status`
