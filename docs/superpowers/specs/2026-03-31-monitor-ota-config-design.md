# Monitor OTA Config Design

## Background

`nav2_monitor`、`bridge`、`safety_emergency_executor` 当前已经有运行时参数文件，但参数入口仍分散在包内 `config/`、`runtime_configs` 和节点内部相对路径解析逻辑里。这样不利于后续 OTA 统一下发和替换参数。

本次目标是把三部分参数统一收口到 `/opt/ry/config/Monitor/`，并让运行时只认这个 OTA 目录，不再回退到包内配置或 `runtime_configs`。

## Scope

In scope:

- 为三部分参数定义统一的 OTA 目录结构
- 修改 launch 和路径解析逻辑，使运行时强制读取 `/opt/ry/config/Monitor/...`
- 调整参数文件中的内部引用，全部改为 OTA 绝对路径
- 调整 `runtime_configs` 的目录布局和安装规则，为后续把文件安装到 `/opt/ry/config/Monitor` 做准备
- 补充与路径切换相关的测试

Out of scope:

- 修改参数语义、默认值或监控策略
- 新增 OTA 下发机制本身
- 变更其他不在本次范围内的 ROS 包

## Requirements

1. 运行时配置根目录固定为 `/opt/ry/config/Monitor`
2. 三部分参数分目录存放，不平铺
3. 运行时不得再回退到 `runtime_configs` 或包内 `config/`
4. `nav2_monitor` 的级联配置和 `bridge` 的 spec 文件也必须切到 OTA 绝对路径
5. 保持现有参数内容和行为不变，只改参数来源和组织方式

## Chosen Approach

采用统一绝对路径方案:

- `nav2_monitor.launch.py` 直接加载 `/opt/ry/config/Monitor/nav2_monitor/nav2_monitor_params.yaml`
- `safety_emergency_executor.launch.py` 直接加载 `/opt/ry/config/Monitor/safety_emergency_executor/safety_emergency_executor_params.yaml`
- `bridge` 参数文件中的 `spec_file` 直接指向 `/opt/ry/config/Monitor/bridge/generic_multi_bridge_spec.yaml`
- `nav2_monitor_params.yaml` 中的 `fault_config` 和 `task_fault_configs.*` 全部改为 `/opt/ry/config/Monitor/nav2_monitor/...`
- 代码中的路径解析逻辑不再兜底包内 share、`runtime_configs` 或相对路径配置

## Target Layout

最终 OTA 目录结构如下:

```text
/opt/ry/config/Monitor/
├── bridge/
│   ├── bridge_cpp_params.yaml
│   ├── bridge_py_params.yaml
│   └── generic_multi_bridge_spec.yaml
├── nav2_monitor/
│   ├── fault_detector_config.yaml
│   ├── nav2_monitor_params.yaml
│   └── profiles/
│       ├── fault_detector_elevator.yaml
│       ├── fault_detector_reverse.yaml
│       └── fault_detector_todoor.yaml
└── safety_emergency_executor/
    └── safety_emergency_executor_params.yaml
```

仓库中的 `runtime_configs` 同步整理为:

```text
src/runtime_configs/config/Monitor/
├── bridge/
├── nav2_monitor/
└── safety_emergency_executor/
```

安装时应从该结构直接产出 `/opt/ry/config/Monitor/...`。

## Detailed Design

### 1. nav2_monitor

- 修改 launch 文件，参数入口固定为 `/opt/ry/config/Monitor/nav2_monitor/nav2_monitor_params.yaml`
- 修改 `nav2_monitor_params.yaml`
  - `fault_config` 指向 `/opt/ry/config/Monitor/nav2_monitor/fault_detector_config.yaml`
  - `task_fault_configs.default` 指向同一路径
  - 其他任务 profile 指向 `/opt/ry/config/Monitor/nav2_monitor/profiles/*.yaml`
- 修改 `resolve_config_path()`，不再尝试:
  - 当前工作目录
  - `nav2_monitor` 包 share
  - `runtime_configs` 包 share
- 保留绝对路径校验逻辑，若路径不存在则返回原路径并由后续加载流程报错或保持当前配置

### 2. bridge

- 保留 `bridge_py_params.yaml` 和 `bridge_cpp_params.yaml` 两份参数文件
- 将 `bridge_py_params.yaml` 中的 `spec_file` 改为 `/opt/ry/config/Monitor/bridge/generic_multi_bridge_spec.yaml`
- 修改 `resolve_spec_path()`，仅支持:
  - 已存在的绝对路径
- 去掉:
  - `bridge` 包 share 的相对路径解析
  - `runtime_configs` 包 share 的相对路径解析
  - 当前工作目录兜底
- 保持 `load_spec()`、热重载和桥接行为不变

### 3. safety_emergency_executor

- 修改 launch 文件，参数入口固定为 `/opt/ry/config/Monitor/safety_emergency_executor/safety_emergency_executor_params.yaml`
- 参数文件内容本身保持不变，仅调整安装和组织位置

### 4. runtime_configs

- 将目录从:
  - `config/nav2_monitor/`
  - `config/bridge/`
  - `config/safety_emergency_executor/`
- 调整为:
  - `config/Monitor/nav2_monitor/`
  - `config/Monitor/bridge/`
  - `config/Monitor/safety_emergency_executor/`
- 更新 README，使其明确 `Monitor` 是 OTA 大目录
- 更新安装规则，为后续文件直接安装到 `/opt/ry/config/Monitor` 做准备

## Testing Strategy

优先做静态验证和单元测试:

- `bridge`:
  - 新增/调整 Python 单测，验证绝对路径 spec 可解析
  - 验证相对路径不再被 `resolve_spec_path()` 自动兜底到旧位置
- `nav2_monitor`:
  - 增加与路径解析相关的 C++ 单测，或最少对参数文件做静态断言，确保所有内部引用都为 `/opt/ry/config/Monitor/...`
- `runtime_configs`:
  - 验证目录结构已经迁移到 `config/Monitor/...`
  - 验证关键文件存在
- 若环境允许，再补构建或目标测试:
  - `bridge` pytest
  - `nav2_monitor` 相关 gtest

## Risks And Mitigations

- 风险: `/opt/ry/config/Monitor` 尚未部署时，节点将无法依赖旧配置启动
  - 缓解: 这是本次明确选择的强制策略，需在部署侧保证 OTA 目录先到位
- 风险: 仍有文档或脚本引用旧路径
  - 缓解: 本次至少更新直接相关 README、launch 和参数文件；其余文档按需补齐
- 风险: 安装规则若仍输出到 ROS share 目录，会与最终 OTA 目标不一致
  - 缓解: 本次同步整理 `runtime_configs` 目录和安装规则，确保产物结构与 OTA 结构一致

## Verification Criteria

满足以下条件即可认为设计落地成功:

1. 三部分参数都以 `/opt/ry/config/Monitor` 为唯一运行时入口
2. `nav2_monitor` 的级联 fault config 和 `bridge` 的 spec 也都只指向 OTA 目录
3. 代码中不存在对 `runtime_configs` 作为运行时兜底配置源的依赖
4. 仓库内配置目录已整理为 `Monitor` 大目录
5. 相关测试覆盖新的路径约束

## Notes

由于当前会话未获得显式的子代理授权，本设计文档的审阅将先采用本地人工复核方式完成；若后续需要，也可以在用户明确允许后补做独立 reviewer 审阅。
