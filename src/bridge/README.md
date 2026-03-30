# bridge

Generic bridge package for `nav2_monitor/msg/AlgorithmFeedback`.

## Modes

- `bridge_py_node`: primary multi-topic, config-driven bridge
- `bridge_cpp_node`: example typed bridge template

## Python spec format

The Python node reads a YAML file with a top-level `bridges:` list.
Each entry defines one input topic, message type, and the metrics to publish as `AlgorithmFeedback`.

See `config/examples/generic_multi_bridge_spec.yaml`.
