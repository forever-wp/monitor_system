# collision_voxel_layer

Sparse voxel obstacle fusion package for collision detection.

Runtime config follows the Monitor OTA layout.

- Repository runtime source: `config/Monitor/collision_voxel_layer/collision_voxel_layer_params.yaml`
- Deployed runtime path: `/opt/ry/config/Monitor/collision_voxel_layer/collision_voxel_layer_params.yaml`
- Package fallback config: `src/collision_voxel_layer/config/collision_voxel_layer_params.yaml`

The node supports:

- ROS 2 standard parameter updates through the built-in parameter service
- File-based hot reload through `config_file`, `config_reload_enabled`, and `config_reload_period_s`
