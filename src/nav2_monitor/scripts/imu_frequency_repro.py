#!/usr/bin/env python3

import argparse
import os
import signal
import subprocess
import sys
import tempfile
import textwrap
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav2_monitor.msg import MonitorStatus
from sensor_msgs.msg import Imu


class ImuFrequencyProbe(Node):
    def __init__(self, monitor_topic: str, status_topic: str = '/nav2_monitor/status'):
        super().__init__('imu_frequency_probe')
        self.monitor_topic = monitor_topic
        self.imu_count = 0
        self.first_recv_time = None
        self.last_recv_time = None
        self.reported_samples = []

        self.imu_sub = self.create_subscription(Imu, monitor_topic, self._imu_callback, 20)
        self.status_sub = self.create_subscription(MonitorStatus, status_topic, self._status_callback, 10)

    def _imu_callback(self, _msg: Imu):
        now = self.get_clock().now()
        if self.first_recv_time is None:
            self.first_recv_time = now
        self.last_recv_time = now
        self.imu_count += 1

    def _status_callback(self, msg: MonitorStatus):
        for topic, frequency in zip(msg.monitored_topics, msg.topic_frequencies):
            if topic == self.monitor_topic:
                self.reported_samples.append(float(frequency))
                break

    def observed_hz(self) -> float:
        if self.imu_count < 2 or self.first_recv_time is None or self.last_recv_time is None:
            return 0.0
        duration = (self.last_recv_time - self.first_recv_time).nanoseconds / 1e9
        if duration <= 0.0:
            return 0.0
        return (self.imu_count - 1) / duration

    def latest_reported_hz(self) -> float:
        return self.reported_samples[-1] if self.reported_samples else 0.0

    def avg_reported_hz(self) -> float:
        return sum(self.reported_samples) / len(self.reported_samples) if self.reported_samples else 0.0

    def max_reported_hz(self) -> float:
        return max(self.reported_samples) if self.reported_samples else 0.0


def run_shell_command(command: str, env: dict) -> subprocess.Popen:
    return subprocess.Popen(
        ['/bin/bash', '-lc', command],
        env=env,
        preexec_fn=os.setsid,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )


def stop_process(process: subprocess.Popen | None):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        process.wait(timeout=5)
    except Exception:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        except Exception:
            pass


def make_temp_configs(workspace_root: Path, monitor_topic: str, min_hz: float) -> tuple[Path, Path]:
    tmpdir = Path(tempfile.mkdtemp(prefix='nav2_monitor_imu_freq_'))
    fault_config = tmpdir / 'fault_detector_config.yaml'
    params_file = tmpdir / 'nav2_monitor_params.yaml'

    fault_config.write_text(textwrap.dedent(f'''
        multi_value_judge:
          trigger_count: 2
          recover_count: 2

        modules:
          - name: "imu_probe"
            supervisor: 0
            safety_system: 0
            nodes: []
            watch_topics:
              - name: "{monitor_topic}"
                min_hz: {min_hz}
    ''').strip() + '\n')

    params_file.write_text(textwrap.dedent(f'''
        nav2_monitor:
          ros__parameters:
            timeout: 2.0
            scan_rate: 1.0
            check_rate: 5.0
            safety_cooldown_s: 2.0
            supervisor_cooldown_s: 5.0
            topic_states_topic: "/monitor/topic_states"
            vehicle_state_topic: "/monitor/vehicle_state"
            node_tf_state_topic: "/monitor/node_tf_state"
            monitor_battery_state_topic: "/monitor/battery_state"
            feedback_state_topic: "/monitor/feedback_state"
            collision_state_topic: "/monitor/collision_state"
            fault_config: "{fault_config}"
            vehicle_status_file: "/tmp/nav2_monitor_dummy_status.json"
            target_transforms: []
    ''').strip() + '\n')

    return fault_config, params_file


def main() -> int:
    parser = argparse.ArgumentParser(description='Replay rosbag and compare IMU frequency with nav2_monitor status output.')
    parser.add_argument('bag_path', help='Path to rosbag directory')
    parser.add_argument('--topic', default='/livox/imu', help='IMU topic to monitor')
    parser.add_argument('--min-hz', type=float, default=100.0, help='Temporary watch_topics min_hz value')
    parser.add_argument('--settle-seconds', type=float, default=1.5, help='Wait time before bag playback')
    parser.add_argument('--tail-seconds', type=float, default=2.0, help='Wait time after bag playback for status updates')
    args = parser.parse_args()

    bag_path = Path(args.bag_path).resolve()
    if not bag_path.exists():
        print(f'Bag path not found: {bag_path}', file=sys.stderr)
        return 2

    workspace_root = Path(__file__).resolve().parents[3]
    _, params_file = make_temp_configs(workspace_root, args.topic, args.min_hz)

    env = os.environ.copy()
    env.setdefault('ROS_LOG_DIR', '/tmp/ros_logs')

    source_setup = f'source /opt/ros/humble/setup.bash && source {workspace_root / "install" / "setup.bash"}'
    monitor_cmd = (
        f'{source_setup} && '
        f'ros2 run nav2_monitor nav2_monitor_aggregator_node --ros-args --params-file {params_file}'
    )
    bag_cmd = (
        f'{source_setup} && '
        f'ros2 bag play {bag_path} --topics {args.topic}'
    )

    monitor_proc = None
    bag_proc = None
    rclpy.init()
    probe = ImuFrequencyProbe(args.topic)

    try:
        monitor_proc = run_shell_command(monitor_cmd, env)
        time.sleep(args.settle_seconds)

        bag_proc = run_shell_command(bag_cmd, env)
        while bag_proc.poll() is None:
            rclpy.spin_once(probe, timeout_sec=0.1)

        end_time = time.time() + args.tail_seconds
        while time.time() < end_time:
            rclpy.spin_once(probe, timeout_sec=0.1)

        print('=== IMU Frequency Reproduction Result ===')
        print(f'Bag: {bag_path}')
        print(f'Topic: {args.topic}')
        print(f'Received IMU messages: {probe.imu_count}')
        print(f'Observed IMU frequency: {probe.observed_hz():.2f} Hz')
        print(f'nav2_monitor latest frequency: {probe.latest_reported_hz():.2f} Hz')
        print(f'nav2_monitor average frequency: {probe.avg_reported_hz():.2f} Hz')
        print(f'nav2_monitor max frequency: {probe.max_reported_hz():.2f} Hz')
        return 0
    finally:
        stop_process(bag_proc)
        stop_process(monitor_proc)
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
