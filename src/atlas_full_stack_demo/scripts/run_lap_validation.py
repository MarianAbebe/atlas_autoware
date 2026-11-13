#!/usr/bin/env python3
"""Run one isolated N-lap closed-loop trial and retain evaluation evidence."""
import argparse
import bisect
import json
import math
import os
import signal
import subprocess
import time
from collections import deque
from pathlib import Path

import rclpy
from rclpy.qos import (QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy,
                       qos_profile_sensor_data)
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64MultiArray, String
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import SteeringReport
from autoware_internal_debug_msgs.msg import Float32Stamped, Float64Stamped
from autoware_perception_msgs.msg import DetectedObjects, PredictedObjects
from geometry_msgs.msg import PoseWithCovarianceStamped
from autoware_planning_msgs.msg import Trajectory

from lap_metrics import (angle_error, extract_loop_prefix, prepare_route, project_pose,
                         nearest_trajectory_reference, terminal_goal_stopped, unwrap_progress,
                         synchronized_steering_rates, simulator_steering_integration_check,
                         perception_stage_observation)

NS = '/atlas_loc/'
MAX_SAMPLES = 40000
SAFETY_STATUS_SOURCES = {
    NS+'safety_status': 'perception_guard',
    NS+'base_safety_status': 'localization_watchdog',
}
PERCEPTION_TRACE_TOPICS = (
    ('extended', 'lidar/extended'), ('deskewed', 'lidar/deskewed'),
    ('compact', 'lidar/compact'),
    ('ground_filtered', 'obstacles/pointcloud'), ('clusters', 'clusters'),
)


def perception_trace_topics(enabled):
    """Full-cloud observer set is opt-in to bound normal recorder overhead."""
    return PERCEPTION_TRACE_TOPICS if enabled else ()


def stamp(s):
    return int(s.sec) * 1_000_000_000 + int(s.nanosec)


def pose_state(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
    return [stamp(msg.header.stamp), p.x, p.y, p.z, yaw,
            msg.twist.twist.linear.x, msg.twist.twist.linear.y]


def write_jsonl(path, rows):
    with path.open('w', encoding='utf-8') as f:
        for row in rows:
            f.write(json.dumps(row, separators=(',', ':'), allow_nan=False) + '\n')


def object_adapter_timing_record(payload, collector_receipt_monotonic_sec):
    """Keep the adapter's own event payload intact with this observer's receipt time."""
    return {'collector_receipt_monotonic_sec': collector_receipt_monotonic_sec,
            'payload': payload}


def build_launch_command(laps, output, launch_ego_marker=False, steering_step_diagnostics=False):
    """Build a headless validation launch; marker output is opt-in when RViz is off."""
    return ['ros2', 'launch', 'atlas_full_stack_demo', 'five_lap_demo.launch.py',
            f'laps:={laps}', f'results_dir:={output}', 'launch_rviz:=false',
            f'launch_ego_marker:={str(bool(launch_ego_marker)).lower()}',
            f'steering_step_diagnostics:={str(bool(steering_step_diagnostics)).lower()}']


def append_process_event(path, event, **fields):
    """Append one bounded lifecycle event; clocks remain explicitly named."""
    row = dict(event=event, receipt_monotonic_sec=time.monotonic(),
               utc_time_ns=time.time_ns(), **fields)
    with path.open('a', encoding='utf-8') as stream:
        stream.write(json.dumps(row, separators=(',', ':'), allow_nan=False) + '\n')
        stream.flush()


def stop_launch_process(process, lifecycle_path, reason):
    if process is None:
        return None
    if process.poll() is None:
        append_process_event(lifecycle_path, 'process_group_signal', reason=reason,
                             signal='SIGINT', process_pid=process.pid, process_group_id=process.pid)
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            append_process_event(lifecycle_path, 'process_group_signal', reason=reason,
                                 signal='SIGTERM', process_pid=process.pid, process_group_id=process.pid)
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                append_process_event(lifecycle_path, 'process_exit_timeout', process_pid=process.pid)
    append_process_event(lifecycle_path, 'launch_process_exit', process_pid=process.pid,
                         return_code=process.returncode, reason=reason)
    return process.returncode


class Trial:
    def __init__(self, output, laps, trace_perception=False):
        self.node = rclpy.create_node('atlas_lap_evaluator')
        self.output, self.target_laps = output, laps
        self.trace_perception = bool(trace_perception)
        self.started = time.monotonic()
        self.route_message = None
        self.route_points = None
        self.route = self.route_lengths = self.route_cumulative = None
        self.route_length = None
        self.samples = {k: deque(maxlen=MAX_SAMPLES) for k in
                        ('truth', 'ekf', 'ndt', 'ndt_runtime', 'raw', 'detected_objects',
                         'predicted_objects', 'trajectory', 'planned_trajectory', 'controller_commands',
                         'commands', 'steering_report', 'scene_timing', 'object_adapter_timing',
                         'clock_timing', 'steering_integration', 'cluster_metrics',
                         'perception_raw', 'perception_extended', 'perception_deskewed',
                         'perception_compact', 'perception_ground_filtered', 'perception_clusters',
                         'process_resources')}
        self.samples['scene_route'] = deque(maxlen=MAX_SAMPLES)
        self.samples['planner_trajectory'] = deque(maxlen=MAX_SAMPLES)
        self.safety = deque(maxlen=1000)
        self.scene = {}
        self.latest = {}
        self.last_receipt = {}
        self.max_receipt_gap = {}
        self.max_header_gap = {}
        self.last_header = {}
        self.max_state_age = {}
        self.progress = 0.0
        self.max_progress = 0.0
        self.previous_s = None
        self.tracking = deque(maxlen=MAX_SAMPLES)
        self.actual_truth_distance = 0.0
        self.actual_ekf_distance = 0.0
        self.prev_truth = self.prev_ekf = None
        self.latch_seen = False
        self.last_clock_sample_monotonic_ns = None
        self.last_clock_values = None
        self.clock_sample_interval_ns = 250_000_000
        self.end_hold_started = None
        self.last_truth = self.last_ekf = None
        self.last_command = None
        self.last_controller_command = None
        self.last_steering_report = None
        self.nondet = False
        self.last_trajectory_stage_capture = {}
        self.last_perception_stage = {}
        self.runtime_log_offset = 0
        self.last_resource_sample_monotonic_ns = None
        self.previous_process_counters = {}

        self.node.create_subscription(RosPath, NS+'route_path', self.on_route,
                                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.create_subscription(Odometry, NS+'sim/output/odometry', self.on_truth, 20)
        self.node.create_subscription(Odometry, NS+'ekf/ekf_odom', self.on_ekf, 20)
        self.node.create_subscription(PoseWithCovarianceStamped, NS+'ndt/ndt_pose_with_covariance',
                                      lambda m: self.on_timed('ndt', m.header.stamp, pose=self._pose(m)), 20)
        self.perception_observer_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        # Deep tracing deserializes multiple full PointCloud2 streams in the
        # evaluator process. Keep this opt-in so acceptance runs do not pay
        # that diagnostic cost. Retain the original raw-cloud receipt observer
        # in every mode for source/receipt freshness comparison.
        self.node.create_subscription(PointCloud2, NS+'lidar/raw', self.on_raw_pointcloud,
                                      self.perception_observer_qos)
        # Best-effort depth-1 observers retain only stamps, receipt clocks,
        # frame and counts for exact cross-stage correlation.
        for stage, topic in perception_trace_topics(self.trace_perception):
            self.node.create_subscription(PointCloud2, NS+topic,
                lambda msg, name=stage: self.on_perception_cloud(name, msg),
                                          self.perception_observer_qos)
        self.node.create_subscription(DetectedObjects, NS+'obstacles/detected_objects',
                                      self.on_detected_objects, 10)
        self.node.create_subscription(PredictedObjects, NS+'obstacles/predicted_objects',
                                      self.on_predicted_objects, 10)
        self.node.create_subscription(Control, NS+'gated/control_cmd', self.on_command, 20)
        self.node.create_subscription(Control, NS+'control_cmd', self.on_controller_command, 20)
        self.node.create_subscription(SteeringReport, NS+'sim/output/steering', self.on_steering_report, 20)
        self.node.create_subscription(Float64MultiArray,
            NS+'sim/output/steering_integration_step', self.on_steering_integration, 50)
        self.node.create_subscription(RosPath, NS+'trajectory_visualization', self.on_trajectory, 10)
        self.node.create_subscription(Trajectory, NS+'raw_trajectory',
                                      lambda m: self.on_trajectory_stage('scene_route', m), 10)
        self.node.create_subscription(Trajectory, NS+'obstacle_trajectory',
                                      lambda m: self.on_trajectory_stage('planner_trajectory', m), 10)
        self.node.create_subscription(Trajectory, NS+'trajectory', self.on_planned_trajectory, 10)
        # Keep the source labels consistent with full_stack.launch.py remappings.
        for topic, source in SAFETY_STATUS_SOURCES.items():
            self.node.create_subscription(String, topic,
                lambda msg, name=source: self.on_safety(name, msg), 20)
        self.node.create_subscription(String, NS+'scene_info', self.on_scene,
                                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.create_subscription(String, NS+'diagnostics/scene_timing', self.on_scene_timing, 20)
        # The adapter already reports bounded per-detection timing at this
        # topic. Preserve its payload and collector receipt time so a late
        # predicted-object sample can be separated from adapter-side delay.
        self.node.create_subscription(String, NS+'diagnostics/object_adapter_timing',
                                      self.on_object_adapter_timing, 20)
        self.node.create_subscription(Float32Stamped, NS+'ndt/exe_time_ms',
                                      lambda m: self.on_timed('ndt_runtime', m.stamp, ms=m.data), 10)
        for metric_name in ('cyclic_time_ms', 'processing_time_ms', 'pipeline_latency_ms'):
            self.node.create_subscription(Float64Stamped,
                NS+'euclidean_cluster/debug/'+metric_name,
                lambda msg, name=metric_name: self.on_cluster_metric(name, msg), 10)
        self.sample_clocks(force=True)

    def sample_clocks(self, force=False):
        """Record ROS, realtime, and monotonic clocks without mixing their epochs."""
        mono_ns = time.monotonic_ns()
        if (not force and self.last_clock_sample_monotonic_ns is not None and
                mono_ns - self.last_clock_sample_monotonic_ns < self.clock_sample_interval_ns):
            return
        ros_ns = int(self.node.get_clock().now().nanoseconds)
        wall_ns = time.time_ns()
        row = {
            'receipt_monotonic_ns': mono_ns,
            'system_wall_time_ns': wall_ns,
            'ros_time_ns': ros_ns,
            'system_wall_minus_ros_ns': wall_ns - ros_ns,
        }
        if self.last_clock_values is not None:
            mono_delta = (mono_ns - self.last_clock_values['monotonic_ns']) * 1e-9
            wall_delta = (wall_ns - self.last_clock_values['wall_ns']) * 1e-9
            ros_delta = (ros_ns - self.last_clock_values['ros_ns']) * 1e-9
            row.update(monotonic_delta_sec=mono_delta, wall_delta_sec=wall_delta,
                       ros_delta_sec=ros_delta,
                       ros_monotonic_delta_discrepancy_sec=ros_delta - mono_delta,
                       wall_monotonic_delta_discrepancy_sec=wall_delta - mono_delta,
                       likely_clock_discontinuity=(abs(ros_delta - mono_delta) > 0.1 or
                                                   abs(wall_delta - mono_delta) > 0.1))
        else:
            row['likely_clock_discontinuity'] = False
        self.samples['clock_timing'].append(row)
        self.last_clock_sample_monotonic_ns = mono_ns
        self.last_clock_values = {'monotonic_ns': mono_ns, 'wall_ns': wall_ns, 'ros_ns': ros_ns}

    def sample_process_resources(self):
        """Sample only relevant Linux processes once per second using procfs."""
        now_ns = time.monotonic_ns()
        if (self.last_resource_sample_monotonic_ns is not None and
                now_ns-self.last_resource_sample_monotonic_ns < 1_000_000_000):
            return
        self.last_resource_sample_monotonic_ns = now_ns
        # Record host pressure alongside per-process scheduling data. Memory
        # pressure and swap activity can delay ROS delivery without one process
        # appearing CPU-saturated; these fields are sampled once per second.
        try:
            meminfo = {}
            for line in Path('/proc/meminfo').read_text().splitlines():
                key, value = line.split(':', 1)
                fields = value.split()
                if fields and fields[0].isdigit():
                    meminfo[key] = int(fields[0]) * (1024 if len(fields) > 1 and fields[1] == 'kB' else 1)
            pressure = {}
            for resource in ('cpu', 'memory', 'io'):
                values = {}
                for line in Path(f'/proc/pressure/{resource}').read_text().splitlines():
                    fields = line.split()
                    values[fields[0]] = dict(field.split('=', 1) for field in fields[1:] if '=' in field)
                pressure[resource] = values
            load = Path('/proc/loadavg').read_text().split()
            self.samples['process_resources'].append({
                'sample_monotonic_ns': now_ns, 'pid': None, 'command': '__system__',
                'load_average_1m': float(load[0]),
                'memory_available_bytes': meminfo.get('MemAvailable'),
                'swap_free_bytes': meminfo.get('SwapFree'),
                'swap_total_bytes': meminfo.get('SwapTotal'),
                'pressure': pressure,
            })
        except (OSError, ValueError, IndexError):
            pass
        markers = ('component_container', 'static_objects.py', 'scene.py',
                   'perception_guard.py', 'run_lap_validation.py')
        hz = float(os.sysconf('SC_CLK_TCK'))
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit():
                continue
            try:
                pid = int(entry.name)
                command = (entry/'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace').strip()
                if not command or not any(marker in command for marker in markers):
                    continue
                stat_text = (entry/'stat').read_text()
                stat_fields = stat_text[stat_text.rfind(')')+2:].split()
                cpu_ticks = int(stat_fields[11])+int(stat_fields[12])
                status = (entry/'status').read_text(errors='replace')
                rss_kib = next((int(line.split()[1]) for line in status.splitlines()
                                if line.startswith('VmRSS:')), None)
                threads = next((int(line.split()[1]) for line in status.splitlines()
                                if line.startswith('Threads:')), None)
                try:
                    sched = [int(value) for value in (entry/'schedstat').read_text().split()[:3]]
                except (OSError, ValueError):
                    sched = []
                previous = self.previous_process_counters.get(pid)
                elapsed = None if previous is None else (now_ns-previous['monotonic_ns'])*1e-9
                cpu_sec = cpu_ticks/hz
                cpu_percent = None
                runqueue_wait_delta_sec = None
                if previous is not None and elapsed and elapsed > 0:
                    cpu_percent = max(0.0, (cpu_sec-previous['cpu_sec'])/elapsed*100.0)
                    if len(sched) >= 2 and previous.get('runqueue_wait_ns') is not None:
                        runqueue_wait_delta_sec = max(0, sched[1]-previous['runqueue_wait_ns'])*1e-9
                self.previous_process_counters[pid] = {
                    'monotonic_ns': now_ns, 'cpu_sec': cpu_sec,
                    'runqueue_wait_ns': sched[1] if len(sched) >= 2 else None,
                }
                self.samples['process_resources'].append({
                    'sample_monotonic_ns': now_ns, 'pid': pid, 'command': command[:240],
                    'interval_sec': elapsed, 'cpu_time_sec': cpu_sec,
                    'cpu_percent_of_one_core': cpu_percent, 'rss_kib': rss_kib,
                    'thread_count': threads, 'runqueue_wait_delta_sec': runqueue_wait_delta_sec,
                })
            except (OSError, ValueError, IndexError, PermissionError):
                continue

    @staticmethod
    def _pose(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        return [p.x, p.y, p.z, math.atan2(2.0*(q.w*q.z+q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z))]

    def on_route(self, msg):
        if self.route_points is not None or len(msg.poses) < 3:
            return
        self.route_points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.route_message = msg
        self.configure_route()

    def configure_route(self):
        if self.route is not None or not self.route_points or not self.scene:
            return
        length = float(self.scene.get('route_length_m', 0.0))
        prefix = extract_loop_prefix(self.route_points, length)
        self.route, self.route_lengths, self.route_cumulative, self.route_length = prepare_route(
            prefix, require_closed=False)

    def on_timed(self, name, header_stamp, **values):
        now_mono = time.monotonic()
        now_ros = self.node.get_clock().now().nanoseconds
        state_stamp = stamp(header_stamp)
        row = dict(receipt_monotonic_sec=now_mono, receipt_system_wall_time_ns=time.time_ns(),
                   receipt_ros_time_ns=now_ros, header_stamp_ns=state_stamp,
                   header_age_sec=(now_ros-state_stamp)*1e-9, **values)
        self.samples[name].append(row)
        if name in ('raw', 'ndt'):
            if name in self.last_receipt:
                self.max_receipt_gap[name] = max(self.max_receipt_gap.get(name, 0.0), now_mono-self.last_receipt[name])
            self.last_receipt[name] = now_mono
            self.latest[name] = row
            if name in self.last_header:
                gap = (state_stamp-self.last_header[name])*1e-9
                if gap >= 0.0:
                    self.max_header_gap[name] = max(self.max_header_gap.get(name, 0.0), gap)
            self.last_header[name] = state_stamp
            self.max_state_age[name] = max(self.max_state_age.get(name, 0.0), row['header_age_sec'])

    def on_route_pose(self, state, kind):
        if kind == 'truth':
            if self.prev_truth is not None:
                self.actual_truth_distance += math.hypot(state[1]-self.prev_truth[1], state[2]-self.prev_truth[2])
            self.prev_truth = state
            self.last_truth = state
        else:
            if self.prev_ekf is not None:
                self.actual_ekf_distance += math.hypot(state[1]-self.prev_ekf[1], state[2]-self.prev_ekf[2])
            self.prev_ekf = state
            self.last_ekf = state
            if self.route is not None:
                cross, s, route_yaw = project_pose(state[1], state[2], self.route,
                                                   self.route_lengths, self.route_cumulative)
                if self.previous_s is None:
                    self.progress = s
                else:
                    self.progress += unwrap_progress(self.previous_s, s, self.route_length)
                self.previous_s = s
                self.max_progress = max(self.max_progress, self.progress)
                if self.progress <= self.target_laps*self.route_length:
                    self.tracking.append([state[0], cross, angle_error(state[4], route_yaw)])

    def on_truth(self, msg):
        state = pose_state(msg)
        row = dict(header_stamp_ns=state[0], receipt_monotonic_sec=time.monotonic(),
                   position=state[1:4], yaw=state[4], speed_xy=math.hypot(state[5], state[6]))
        self.samples['truth'].append(row)
        self.on_route_pose(state, 'truth')

    def on_ekf(self, msg):
        state = pose_state(msg)
        row = dict(header_stamp_ns=state[0], receipt_monotonic_sec=time.monotonic(),
                   position=state[1:4], yaw=state[4], speed_xy=math.hypot(state[5], state[6]))
        self.samples['ekf'].append(row)
        self.on_route_pose(state, 'ekf')

    def on_command(self, msg):
        row = dict(receipt_monotonic_sec=time.monotonic(), stamp_ns=stamp(msg.stamp),
                   steering=msg.lateral.steering_tire_angle,
                   steering_rate=msg.lateral.steering_tire_rotation_rate,
                   steering_rate_defined=msg.lateral.is_defined_steering_tire_rotation_rate,
                   velocity=msg.longitudinal.velocity, acceleration=msg.longitudinal.acceleration)
        self.samples['commands'].append(row)
        self.last_command = row

    def on_controller_command(self, msg):
        row = dict(receipt_monotonic_sec=time.monotonic(), stamp_ns=stamp(msg.stamp),
                   steering=msg.lateral.steering_tire_angle,
                   steering_rate=msg.lateral.steering_tire_rotation_rate,
                   steering_rate_defined=msg.lateral.is_defined_steering_tire_rotation_rate,
                   velocity=msg.longitudinal.velocity, acceleration=msg.longitudinal.acceleration)
        self.samples['controller_commands'].append(row)
        self.last_controller_command = row

    def on_steering_report(self, msg):
        source_stamp_ns = stamp(msg.stamp)
        receipt = time.monotonic()
        rate = None
        if self.last_steering_report is not None:
            prior_stamp, prior_angle = self.last_steering_report
            dt = (source_stamp_ns - prior_stamp) * 1e-9
            if dt > 0.0:
                rate = (msg.steering_tire_angle - prior_angle) / dt
        self.samples['steering_report'].append({
            'receipt_monotonic_sec': receipt,
            'stamp_ns': source_stamp_ns,
            'steering_angle_rad': msg.steering_tire_angle,
            'derived_rate_radps': rate,
        })
        self.last_steering_report = (source_stamp_ns, msg.steering_tire_angle)

    def on_steering_integration(self, msg):
        """Record the simulator's actual model step; array schema is documented by the patch tool."""
        receipt = time.monotonic()
        data = list(msg.data)
        if len(data) != 9:
            self.nondet = True
            return
        dt, previous, requested, applied, increment, rate, limit, ros_seconds, valid_dt = data
        self.samples['steering_integration'].append({
            'receipt_monotonic_sec': receipt,
            'receipt_system_wall_time_ns': time.time_ns(),
            'receipt_ros_time_ns': self.node.get_clock().now().nanoseconds,
            'integration_dt_sec': float(dt),
            'previous_steering_rad': float(previous),
            'requested_steering_rad': float(requested),
            'applied_steering_rad': float(applied),
            'steering_increment_rad': float(increment),
            'internal_steering_rate_radps': float(rate),
            'configured_steering_rate_limit_radps': float(limit),
            'integration_ros_time_sec': float(ros_seconds),
            'valid_dt': bool(valid_dt),
        })

    def on_trajectory(self, msg):
        self.samples['trajectory'].append({
            'receipt_monotonic_sec': time.monotonic(),
            'header_stamp_ns': stamp(msg.header.stamp),
            'frame_id': msg.header.frame_id,
            'point_count': len(msg.poses),
        })

    def on_trajectory_stage(self, stage, msg):
        """Keep at most one compact route/planner snapshot per second and stage."""
        capture_mono = time.monotonic()
        if capture_mono - self.last_trajectory_stage_capture.get(stage, -math.inf) < 1.0:
            return
        self.last_trajectory_stage_capture[stage] = capture_mono
        points = msg.points
        if not points:
            summary = dict(point_count=0, path_length_m=0.0, first_position=None,
                           last_position=None, zero_speed_points=[])
        else:
            positions = [(float(p.pose.position.x), float(p.pose.position.y)) for p in points]
            length = sum(math.dist(a, b) for a, b in zip(positions, positions[1:]))
            stride = max(1, (len(points) - 1) // 10)
            zero = [{'index': i, 'position': list(positions[i]),
                     'velocity_mps': float(point.longitudinal_velocity_mps)}
                    for i, point in enumerate(points)
                    if abs(float(point.longitudinal_velocity_mps)) <= 1e-6]
            summary = dict(
                point_count=len(points), path_length_m=length,
                first_position=list(positions[0]), last_position=list(positions[-1]),
                first_velocity_mps=float(points[0].longitudinal_velocity_mps),
                last_velocity_mps=float(points[-1].longitudinal_velocity_mps),
                maximum_velocity_mps=max(float(p.longitudinal_velocity_mps) for p in points),
                zero_speed_points=zero[:32],
                profile=[{'index': i, 'position': list(positions[i]),
                          'velocity_mps': float(points[i].longitudinal_velocity_mps)}
                         for i in sorted(set([0, len(points)-1] + list(range(0, len(points), stride))))]
            )
        ego = None
        if self.last_truth is not None:
            ego = {'position': self.last_truth[1:3], 'yaw_rad': self.last_truth[4],
                   'speed_mps': math.hypot(self.last_truth[5], self.last_truth[6])}
        self.samples[stage].append(dict(
            receipt_monotonic_sec=capture_mono, header_stamp_ns=stamp(msg.header.stamp),
            frame_id=msg.header.frame_id, ego=ego, **summary))

    def on_planned_trajectory(self, msg):
        velocities = [float(point.longitudinal_velocity_mps) for point in msg.points]
        zero_points = [point for point in msg.points if abs(float(point.longitudinal_velocity_mps)) <= 1e-6]
        goal = self.scene.get('goal') if self.scene else None
        last_point = msg.points[-1] if msg.points else None
        last_position = None if last_point is None else [last_point.pose.position.x,
                                                         last_point.pose.position.y]
        receipt = time.monotonic()
        ego = None
        if self.last_truth is not None:
            ego = {'position': self.last_truth[1:3], 'yaw_rad': self.last_truth[4],
                   'speed_mps': math.hypot(self.last_truth[5], self.last_truth[6])}
        reference = None if ego is None else nearest_trajectory_reference(
            msg.points, ego['position'], ahead_count=4)
        self.samples['planned_trajectory'].append({
            'receipt_monotonic_sec': receipt,
            'controller_ego_at_receipt': ego,
            'nearest_controller_reference': reference,
            'latest_controller_command': self.last_controller_command,
            'header_stamp_ns': stamp(msg.header.stamp),
            'frame_id': msg.header.frame_id,
            'point_count': len(msg.points),
            'first_velocity_mps': velocities[0] if velocities else None,
            'last_velocity_mps': velocities[-1] if velocities else None,
            'maximum_velocity_mps': max(velocities) if velocities else None,
            'first_zero_speed_position': (None if not zero_points else
                [zero_points[0].pose.position.x, zero_points[0].pose.position.y]),
            'last_point_position': last_position,
            'last_point_distance_to_goal_m': (None if last_position is None or not goal else
                math.dist(last_position, goal[:2])),
        })

    def on_detected_objects(self, msg):
        row = self.perception_observation('detected_objects', msg.header.stamp, msg.header.frame_id,
                                          len(msg.objects))
        row['object_count'] = len(msg.objects)
        self.samples['detected_objects'].append(row)

    def perception_observation(self, stage, header, frame_id, item_count):
        mono_ns = time.monotonic_ns()
        ros_ns = self.node.get_clock().now().nanoseconds
        source_ns = stamp(header)
        previous = self.last_perception_stage.get(stage)
        row = perception_stage_observation(stage, source_ns, mono_ns, ros_ns,
                                           frame_id, item_count, previous)
        self.last_perception_stage[stage] = row
        return row

    def on_perception_cloud(self, stage, msg):
        self.samples['perception_'+stage].append(self.perception_observation(
            stage, msg.header.stamp, msg.header.frame_id, msg.width*msg.height))

    def on_raw_pointcloud(self, msg):
        self.on_timed('raw', msg.header.stamp, points=msg.width*msg.height,
                      frame=msg.header.frame_id)
        self.on_perception_cloud('raw', msg)

    def on_cluster_metric(self, metric_name, msg):
        self.samples['cluster_metrics'].append({
            'metric': metric_name,
            'measurement_stamp_ns': stamp(msg.stamp),
            'receipt_monotonic_ns': time.monotonic_ns(),
            'receipt_ros_time_ns': self.node.get_clock().now().nanoseconds,
            'value_ms': float(msg.data),
        })

    def on_predicted_objects(self, msg):
        row = self.perception_observation('predicted_objects', msg.header.stamp, msg.header.frame_id,
                                          len(msg.objects))
        objects = []
        for obj in msg.objects:
            p = obj.kinematics.initial_pose_with_covariance.pose.position
            xyz = [float(p.x), float(p.y), float(p.z)]
            distance = None
            if self.last_truth is not None:
                distance = math.dist(xyz, self.last_truth[1:4])
            objects.append({'position': xyz, 'distance_to_truth_m': distance})
        row['objects'] = objects
        self.samples['predicted_objects'].append(row)

    def on_safety(self, source, msg):
        try:
            state = json.loads(msg.data)
        except (ValueError, TypeError):
            state = {'unparsed': msg.data}
        state['receipt_monotonic_sec'] = time.monotonic()
        state['source'] = source
        self.safety.append(state)
        self.latch_seen |= bool(state.get('latched', False))

    def on_scene(self, msg):
        try:
            self.scene = json.loads(msg.data)
            self.configure_route()
        except (ValueError, TypeError):
            pass

    def on_scene_timing(self, msg):
        try:
            row = json.loads(msg.data)
            row['collector_receipt_monotonic_sec'] = time.monotonic()
            self.samples['scene_timing'].append(row)
        except (ValueError, TypeError):
            self.nondet = True

    def on_object_adapter_timing(self, msg):
        self.samples['object_adapter_timing'].append(object_adapter_timing_record(
            msg.data, time.monotonic()))

    def spin(self, duration, process, runtime_log_path):
        deadline = time.monotonic() + duration
        next_log_check = time.monotonic()
        while rclpy.ok() and time.monotonic() < deadline and process.poll() is None:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            self.sample_clocks()
            self.sample_process_resources()
            if time.monotonic() >= next_log_check:
                next_log_check = time.monotonic() + 0.5
                try:
                    # Inspect only newly appended output; rereading an ever-
                    # growing launch log on every poll caused quadratic I/O.
                    with runtime_log_path.open('r', encoding='utf-8', errors='replace') as log:
                        log.seek(self.runtime_log_offset)
                        text = log.read()
                        self.runtime_log_offset = log.tell()
                    if 'process has died' in text and 'user interrupted with ctrl-c' not in text:
                        return 'child_process_failed'
                except OSError:
                    pass
            target_progress = self.target_laps*self.route_length if self.route_length else None
            goal = self.scene.get('goal') if self.scene else None
            at_terminal_goal = bool(self.last_ekf and goal and terminal_goal_stopped(
                self.last_ekf[1:3], math.hypot(self.last_ekf[5], self.last_ekf[6]), goal))
            if target_progress is not None and self.max_progress >= target_progress-1e-6 and at_terminal_goal:
                if self.end_hold_started is None:
                    self.end_hold_started = time.monotonic()
                elif time.monotonic()-self.end_hold_started >= 3.0:
                    return 'completed_and_held'
            else:
                self.end_hold_started = None
            if self.latch_seen:
                return 'watchdog_latched'
        return 'timeout' if process.poll() is None else 'launch_exited'

    def finalize(self, status, process_rc, timeout_sec, runtime_log=''):
        for name, rows in self.samples.items():
            write_jsonl(self.output / f'{name}.jsonl', rows)
        write_jsonl(self.output / 'safety_status.jsonl', self.safety)
        truth = sorted(self.samples['truth'], key=lambda x: x['header_stamp_ns'])
        truth_stamps = [x['header_stamp_ns'] for x in truth]
        matched = []
        for estimate in self.samples['ekf']:
            state_stamp = estimate['header_stamp_ns']
            index = bisect.bisect_left(truth_stamps, state_stamp)
            if index >= len(truth) or truth_stamps[index] != state_stamp:
                if index == 0 or index >= len(truth):
                    continue
                left, right = truth[index-1], truth[index]
                interval = right['header_stamp_ns']-left['header_stamp_ns']
                if interval <= 0 or interval > 100_000_000:
                    continue
                ratio = (state_stamp-left['header_stamp_ns'])/interval
                truth_xyz = [a+ratio*(b-a) for a, b in zip(left['position'], right['position'])]
                truth_yaw = left['yaw']+ratio*angle_error(right['yaw'], left['yaw'])
            else:
                truth_xyz, truth_yaw = truth[index]['position'], truth[index]['yaw']
            if not all(math.isfinite(v) for v in truth_xyz+[truth_yaw]):
                continue
            xyz = math.sqrt(sum((a-b)**2 for a, b in zip(estimate['position'], truth_xyz)))
            yaw = abs(angle_error(estimate['yaw'], truth_yaw))
            matched.append([xyz, yaw])
        cross = [r[1] for r in self.tracking]
        heading = [abs(r[2]) for r in self.tracking]
        target_distance = self.target_laps*(self.route_length or 0.0) + float(self.scene.get('terminal_extension_m', 0.0))
        final_position_error = None
        if self.last_ekf and self.scene.get('goal'):
            goal = self.scene['goal']
            final_position_error = math.dist(self.last_ekf[1:3], goal[:2])
        metric = lambda xs: {'rms': math.sqrt(sum(x*x for x in xs)/len(xs)),
                             'maximum': max(xs)} if xs else None
        controller_reference_distance = []
        controller_reference_heading = []
        for row in self.samples['planned_trajectory']:
            reference = row.get('nearest_controller_reference')
            ego = row.get('controller_ego_at_receipt')
            if reference and ego and reference.get('points'):
                controller_reference_distance.append(float(reference['distance_m']))
                controller_reference_heading.append(abs(angle_error(
                    float(ego['yaw_rad']), float(reference['points'][0]['yaw_rad']))))
        gated_commands = list(self.samples['commands'])
        raw_commands = list(self.samples['controller_commands'])
        steering_feedback = list(self.samples['steering_report'])
        steering_angles = [abs(float(row['steering_angle_rad'])) for row in steering_feedback]
        steering_rate_measurement = synchronized_steering_rates(steering_feedback, truth)
        steering_rates = steering_rate_measurement['rates_radps']
        integration_steps = list(self.samples['steering_integration'])
        integration_check = simulator_steering_integration_check(integration_steps, len(truth))
        max_gated_speed = max((abs(float(row['velocity'])) for row in gated_commands), default=None)
        max_raw_speed = max((abs(float(row['velocity'])) for row in raw_commands), default=None)
        max_gated_steer = max((abs(float(row['steering'])) for row in gated_commands), default=None)
        acceleration_samples = []
        for before, after in zip(truth, truth[1:]):
            dt = (after['header_stamp_ns'] - before['header_stamp_ns']) * 1e-9
            if dt > 0.0:
                acceleration_samples.append((after['speed_xy'] - before['speed_xy']) / dt)
        measured_accel_max = max((abs(v) for v in acceleration_samples), default=None)
        acceptance_checks = {
            'target_lap_progress': self.route_length is not None and
                self.max_progress >= self.target_laps * self.route_length - 0.25,
            'goal_position_tolerance': final_position_error is not None and final_position_error <= 0.15,
            'terminal_hold_completed': status == 'completed_and_held' and self.end_hold_started is not None,
            'safety_gate_not_latched': not self.latch_seen,
            'controller_reference_cross_track': bool(controller_reference_distance) and
                max(controller_reference_distance) <= 0.30,
            'controller_reference_heading': bool(controller_reference_heading) and
                max(controller_reference_heading) <= 0.15,
            'gated_speed_limit': max_gated_speed is not None and max_gated_speed <= 0.81,
            'gated_steering_limit': max_gated_steer is not None and max_gated_steer <= 0.501,
            'simulated_steering_limit': bool(steering_angles) and max(steering_angles) <= 0.501,
            # The report/odometry streams have independent publication stamps;
            # only integration-step telemetry identifies the actual model dt.
            # Keep the paired estimate in the report as a diagnostic, not as a
            # substitute for the simulator's applied-step measurement.
            'simulated_steering_rate_limit': integration_check['passed'],
            'internal_steering_integration_step_limit': integration_check['passed'],
            'simulated_acceleration_limit': bool(acceleration_samples) and measured_accel_max <= 0.85,
            'controller_respects_speed_limit': max_raw_speed is not None and max_raw_speed <= 0.81,
        }
        log_lines = runtime_log.splitlines()
        shutdown_index = next((i for i, line in enumerate(log_lines)
                               if 'user interrupted with ctrl-c' in line), len(log_lines))
        runtime_errors = [line.strip() for i, line in enumerate(log_lines[:shutdown_index])
                          if any(token in line for token in ('Traceback (most recent call last)',
                              'process has died', 'FAIL: route selection', 'Unhandled exception'))]
        shutdown_exceptions = [line.strip() for i, line in enumerate(log_lines[shutdown_index:], shutdown_index)
                               if any(token in line for token in ('Traceback (most recent call last)', 'process has died'))]
        result = {
            'schema_version': 1, 'target_laps': self.target_laps,
            'status': status,
            'passed': status == 'completed_and_held' and
                      process_rc in (0, -signal.SIGINT) and not runtime_errors and
                      all(acceptance_checks.values()),
            'acceptance_checks': acceptance_checks,
            'acceptance_failures': [name for name, passed in acceptance_checks.items() if not passed],
            'timeout_sec': timeout_sec, 'launch_exit_code': process_rc,
            'runtime_error_lines': runtime_errors,
            'expected_shutdown_exception_lines': shutdown_exceptions,
            'route_length_m': self.route_length,
            'planned_distance_m': target_distance,
            'completed_lap_count_from_ekf_projection': (max(0, int(self.max_progress/self.route_length)) if self.route_length else 0),
            'fractional_lap_progress_from_ekf_projection': (self.max_progress / self.route_length if self.route_length else 0.0),
            'actual_truth_distance_m': self.actual_truth_distance,
            'actual_ekf_distance_m': self.actual_ekf_distance,
            'final_route_cross_track_m': final_position_error,
            'cross_track_error_m': metric(cross), 'heading_error_rad': metric(heading),
            'controller_reference_cross_track_m': metric(controller_reference_distance),
            'controller_reference_heading_error_rad': metric(controller_reference_heading),
            'max_gated_speed_mps': max_gated_speed,
            'max_controller_speed_mps': max_raw_speed,
            'max_gated_steering_angle_rad': max_gated_steer,
            'max_simulated_steering_angle_rad': max(steering_angles, default=None),
            'max_simulated_steering_rate_radps': max(steering_rates, default=None),
            'internal_steering_integration_step_measurement': integration_check,
            'steering_rate_measurement': {
                'method': 'SteeringReport angle deltas paired to nearest simulator odometry update stamp',
                'paired_samples': steering_rate_measurement['paired_samples'],
                'input_samples': steering_rate_measurement['input_samples'],
                'maximum_pair_skew_sec': steering_rate_measurement['maximum_pair_skew_sec'],
                'maximum_message_stamp_secant_radps': max((abs(float(row['derived_rate_radps']))
                    for row in steering_feedback if row['derived_rate_radps'] is not None), default=None),
                'maximum_synchronized_update_rate_radps': max(steering_rates, default=None),
                'interpretation': ('Informational only: SteeringReport and odometry are stamped in separate '
                    'publication calls and do not carry a shared simulator-step identifier. The authoritative '
                    'simulated rate check uses integration-step telemetry.'),
            },
            'max_odometry_derived_acceleration_mps2': measured_accel_max,
            'localization_error_against_evaluation_truth': {
                'matched_samples': len(matched),
                'position_3d_rmse_m': math.sqrt(sum(x[0]*x[0] for x in matched)/len(matched)) if matched else None,
                'yaw_rmse_rad': math.sqrt(sum(x[1]*x[1] for x in matched)/len(matched)) if matched else None,
            },
            'samples': {name: len(rows) for name, rows in self.samples.items()},
            'max_header_gap_sec': self.max_header_gap,
            'max_receipt_gap_sec': self.max_receipt_gap,
            'max_header_age_sec': self.max_state_age,
            'watchdog_latch_seen': self.latch_seen,
            'final_safety_status': self.safety[-1] if self.safety else None,
            'final_command': self.last_command,
            'final_controller_command': self.last_controller_command,
            'endpoint_hold_sec': 0.0 if self.end_hold_started is None else time.monotonic()-self.end_hold_started,
            'route_scene_info': self.scene,
            'start_finish_position_error_m': final_position_error,
            'notes': ['Ground truth is used only for evaluation.',
                      'lap completion is counted from EKF path progress on the closed route prefix; terminal completion also requires reaching the terminal spur goal.',
                      'No full-trajectory pose alignment is applied.'],
        }
        (self.output/'validation.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--laps', type=int, choices=(1, 2, 5), required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout', type=float, help='bounded wall timeout; default 90 + 95 seconds per lap')
    parser.add_argument('--launch-ego-marker', action='store_true',
                        help='enable the visualization marker during headless validation (off by default)')
    parser.add_argument('--trace-perception', action='store_true',
                        help='subscribe to raw/intermediate full point clouds for source-stamp stage tracing')
    parser.add_argument('--steering-step-diagnostics', action='store_true',
                        help='enable simulator integration-step telemetry (requires diagnostic simulator overlay)')
    args = parser.parse_args()
    timeout = args.timeout if args.timeout is not None else 90.0 + 95.0*args.laps
    if timeout < 60 or timeout > 700:
        parser.error('--timeout must be between 60 and 700 seconds')
    if args.output.exists():
        parser.error(f'results directory already exists: {args.output}')
    args.output.mkdir(parents=True)
    lifecycle_path = args.output/'process_lifecycle.jsonl'
    (args.output/'run_metadata.json').write_text(json.dumps({
        'laps': args.laps, 'timeout_sec': timeout, 'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'),
        'perception_stage_trace_enabled': args.trace_perception,
        'steering_step_diagnostics_enabled': args.steering_step_diagnostics,
        'atlas_autoware_root': os.environ.get('ATLAS_AUTOWARE_ROOT'),
        'started_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
    }, indent=2)+'\n')

    rclpy.init()
    trial = Trial(args.output, args.laps, trace_perception=args.trace_perception)
    append_process_event(lifecycle_path, 'validator_started', process_pid=os.getpid(),
                         ros_time_ns=trial.node.get_clock().now().nanoseconds,
                         monotonic_clock='time.monotonic shared host clock')
    launch_cmd = build_launch_command(args.laps, args.output, args.launch_ego_marker,
                                      args.steering_step_diagnostics)
    log = (args.output/'runtime.log').open('w', encoding='utf-8')
    process = None
    result = None
    try:
        process = subprocess.Popen(launch_cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            process_group_id = os.getpgid(process.pid)
        except ProcessLookupError:
            process_group_id = None
        append_process_event(lifecycle_path, 'launch_process_started', process_pid=process.pid,
                             process_group_id=process_group_id, start_new_session=True,
                             command=launch_cmd)
        run_status = trial.spin(timeout, process, args.output/'runtime.log')
        append_process_event(lifecycle_path, 'validation_monitor_stopped', reason=run_status,
                             launch_process_return_code=process.poll())
        if process.poll() is None:
            stop_launch_process(process, lifecycle_path, run_status)
        else:
            append_process_event(lifecycle_path, 'launch_process_exit', process_pid=process.pid,
                                 return_code=process.returncode, reason='exited_before_validator_cleanup')
        result = trial.finalize(run_status, process.returncode, timeout)
    except KeyboardInterrupt:
        stop_launch_process(process, lifecycle_path, 'validator_keyboard_interrupt')
        result = trial.finalize('interrupted', None if process is None else process.returncode, timeout)
    except Exception as exc:
        append_process_event(lifecycle_path, 'validator_exception',
                             exception_type=type(exc).__name__, exception=str(exc))
        stop_launch_process(process, lifecycle_path, 'validator_exception')
        result = trial.finalize('validation_error', None if process is None else process.returncode, timeout)
        (args.output/'error.txt').write_text(f'{type(exc).__name__}: {exc}\n')
    finally:
        log.close()
        try:
            runtime_log = (args.output/'runtime.log').read_text(encoding='utf-8', errors='replace')
            if result is not None:
                result = trial.finalize(result.get('status', 'interrupted'),
                    None if process is None else process.returncode, timeout, runtime_log)
        except OSError:
            pass
        trial.node.destroy_node()
        rclpy.try_shutdown()
    print(json.dumps(result, indent=2))
    return 0 if result and result.get('passed') else 1


if __name__ == '__main__':
    raise SystemExit(main())
