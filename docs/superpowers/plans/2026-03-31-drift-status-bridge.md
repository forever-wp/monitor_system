# Drift Status Bridge Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a `/drift_status` PoseStamped bridge into `light-lm` and expose separate drift feedback rules.

**Architecture:** Extend the existing config-driven bridge spec instead of creating a new node, then wire `light-lm` feedback rules through the existing `nav2_monitor` fault detector configuration. Keep runtime OTA files and mirrored package config files aligned.

**Tech Stack:** ROS 2, YAML config, pytest, gtest-backed config validation

---

### Task 1: Add failing bridge layout assertions

**Files:**
- Modify: `src/bridge/test/test_monitor_ota_layout.py`

- [ ] **Step 1: Write failing assertions for `/drift_status`**

- [ ] **Step 2: Run bridge tests and verify failure**

### Task 2: Add `/drift_status` bridge spec

**Files:**
- Modify: `config/Monitor/bridge/generic_multi_bridge_spec.yaml`

- [ ] **Step 1: Add the new `PoseStamped` bridge entry**

- [ ] **Step 2: Re-run bridge tests and verify pass**

### Task 3: Add `light-lm` feedback rules

**Files:**
- Modify: `config/Monitor/nav2_monitor/fault_detector_config.yaml`
- Modify: `config/Monitor/nav2_monitor/profiles/fault_detector_todoor.yaml`
- Modify: `config/Monitor/nav2_monitor/profiles/fault_detector_elevator.yaml`
- Modify: `config/Monitor/nav2_monitor/profiles/fault_detector_reverse.yaml`
- Modify: `src/nav2_monitor/config/fault_detector_config.yaml`
- Modify: `src/nav2_monitor/config/profiles/fault_detector_todoor.yaml`
- Modify: `src/nav2_monitor/config/profiles/fault_detector_elevator.yaml`
- Modify: `src/nav2_monitor/config/profiles/fault_detector_reverse.yaml`

- [ ] **Step 1: Add independent `drift_state` and `drift_delta_norm` rules under `light-lm`**

- [ ] **Step 2: Run nav2_monitor tests and verify pass**

### Task 4: Update docs

**Files:**
- Modify: `src/nav2_monitor/README.md`

- [ ] **Step 1: Document `/drift_status` bridge and `light-lm` feedback rules**

- [ ] **Step 2: Re-run targeted verification**

Plan complete and saved to `docs/superpowers/plans/2026-03-31-drift-status-bridge.md`. Ready to execute.
