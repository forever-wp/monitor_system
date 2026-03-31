# runtime_configs

Centralized runtime configuration bundle for OTA deployment.

## Layout

- `config/Monitor/nav2_monitor/`
- `config/Monitor/safety_emergency_executor/`
- `config/Monitor/bridge/`

This package is the OTA-facing source of truth for runtime parameter files.

## Authoritative runtime files

- `config/Monitor/nav2_monitor/nav2_monitor_params.yaml`
- `config/Monitor/nav2_monitor/fault_detector_config.yaml`
- `config/Monitor/nav2_monitor/profiles/*.yaml`
- `config/Monitor/safety_emergency_executor/safety_emergency_executor_params.yaml`
- `config/Monitor/bridge/bridge_py_params.yaml`
- `config/Monitor/bridge/generic_multi_bridge_spec.yaml`
- `config/Monitor/bridge/bridge_cpp_params.yaml`
