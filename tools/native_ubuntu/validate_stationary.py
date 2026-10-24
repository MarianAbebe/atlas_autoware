#!/usr/bin/env python3
"""Run engagement-free stationary localization, safety, and optional RViz checks."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from autoware_vehicle_msgs.msg import Engage
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration


class Inconclusive(RuntimeError):
    """Required telemetry was not observed; this is not a PASS."""


def atlas_pids(patterns=("full_stack.launch.py", "atlas_loc.full_stack_scene",
                         "localization_watchdog.py", "perception_guard.py",
                         "autoware_ndt_scan_matcher_node", "autoware_ekf_localizer_node",
                         "autoware_motion_velocity_planner_node", "controller_node_exe")):
    found = []
    for entry in Path("/proc").glob("[0-9]*/cmdline"):
        try:
            args = entry.read_bytes().replace(b"\0", b" ").decode(errors="replace")
        except OSError:
            continue
        if any(pattern in args for pattern in patterns):
            found.append((int(entry.parent.name), args.strip()))
    return found


def rviz_pids():
    found = []
    for entry in Path("/proc").glob("[0-9]*/comm"):
        try:
            if entry.read_text().strip() == "rviz2":
                found.append(int(entry.parent.name))
        except OSError:
            pass
    return found


class Monitor(Node):
    def __init__(self):
        super().__init__("atlas_stationary_validation")
        self.begin = time.monotonic()
        self.base = None
        self.combined = None
        self.scene_records = []
        self.truth = []
        self.ndt = []
        self.ekf = []
        self.engage_messages = 0
        self.base_history = []
        self.combined_history = []
        self.health_violations = []
        self.create_subscription(String, "/atlas_loc/base_safety_status", self.base_status, 20)
        self.create_subscription(String, "/atlas_loc/safety_status", self.combined_status, 20)
        self.create_subscription(Engage, "/atlas_loc/gate/input/engage", self.engagement, 10)
        self.create_subscription(Engage, "/atlas_loc/sim/input/engage", self.engagement, 10)
        self.create_subscription(String, "/atlas_loc/diagnostics/scene_timing", self.scene, 30)
        self.create_subscription(Odometry, "/atlas_loc/sim/output/odometry", self.odom, 50)
        self.create_subscription(PoseWithCovarianceStamped,
                                 "/atlas_loc/ndt/ndt_pose_with_covariance", self.ndt_pose, 20)
        self.create_subscription(Odometry, "/atlas_loc/ekf/ekf_odom", self.ekf_odom, 50)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

    def status(self, raw):
        try:
            value = json.loads(raw.data)
            value["receipt_monotonic"] = time.monotonic()
            return value
        except (ValueError, TypeError):
            return None

    def update_status(self, msg, label):
        value = self.status(msg)
        if value is None:
            self.health_violations.append({"input": label, "reason": "invalid_status_json"})
            return
        history = self.base_history if label == "base" else self.combined_history
        history.append(value)
        previous_armed = any(item.get("armed") for item in history[:-1])
        if value.get("latched") or (previous_armed and
                                     (not value.get("armed") or value.get("currently_stale"))):
            self.health_violations.append({"input": label, "status": value})
        setattr(self, label, value)

    def base_status(self, msg): self.update_status(msg, "base")
    def combined_status(self, msg): self.update_status(msg, "combined")

    def engagement(self, msg):
        if msg.engage:
            self.engage_messages += 1

    def scene(self, msg):
        try:
            value = json.loads(msg.data)
            value["receipt_monotonic"] = time.monotonic()
            self.scene_records.append(value)
        except ValueError:
            pass

    def odom(self, msg):
        self.truth.append((time.monotonic(), msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                           math.sqrt(msg.twist.twist.linear.x ** 2 +
                                     msg.twist.twist.linear.y ** 2 +
                                     msg.twist.twist.linear.z ** 2),
                           (msg.pose.pose.position.x, msg.pose.pose.position.y,
                            msg.pose.pose.position.z)))

    def ndt_pose(self, msg):
        self.ndt.append((time.monotonic(), msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9))

    def ekf_odom(self, msg):
        self.ekf.append((time.monotonic(), msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9))

    def tf_available(self):
        return self.tf_buffer.can_transform("map", "base_link", rclpy.time.Time(),
                                            timeout=Duration(seconds=0.0))


def spin_for(node, seconds, rviz=None):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if rviz is not None and rviz.poll() is not None:
            return "rviz_exited"
        if node.engage_messages:
            return "engagement_request_observed"
        if node.health_violations:
            return "watchdog_health_violation"
        for name, status in (("base", node.base), ("combined", node.combined)):
            if status and status.get("latched"):
                return f"{name}_watchdog_latched"
    return None


def fresh_status(value, now):
    if not value:
        return False
    return now - value["receipt_monotonic"] < 0.3 and not value.get("currently_stale")


def gaps(samples):
    vals = [x[0] for x in samples]
    return max((b-a for a, b in zip(vals, vals[1:])), default=None)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--without-rviz", action="store_true")
    parser.add_argument("--hold-sec", type=float, default=120.0)
    args = parser.parse_args()
    if args.hold_sec < 120:
        parser.error("--hold-sec must be at least 120 seconds")
    root = Path(__file__).resolve().parents[2]
    run = args.output or Path("/tmp") / ("atlas_native_stationary_" + time.strftime("%Y%m%d_%H%M%S", time.gmtime()))
    run = run.resolve()
    run.mkdir(parents=True, exist_ok=False)
    results = run / "stack_results"
    results.mkdir()
    (run / "ros_logs").mkdir()
    env = dict(os.environ, ATLAS_REPO_ROOT=str(root),
               ATLAS_AUTOWARE_ROOT=os.environ.get("ATLAS_AUTOWARE_ROOT", str(root.parent/"references/autoware")),
               ATLAS_MAP_DIR=os.environ.get("ATLAS_MAP_DIR", str(Path.home()/"atlas_assets")),
               ROS_LOG_DIR=str(run/"ros_logs"), PYTHONDONTWRITEBYTECODE="1")
    # rclpy is initialized in this process, not only in the child processes.
    # Keep its logs with the rest of this run's evidence as documented.
    os.environ["ROS_LOG_DIR"] = env["ROS_LOG_DIR"]
    report = {"result": "RUNNING", "output": str(run), "engagement_requested": False,
              "hold_sec": args.hold_sec, "phases": {}, "faults": []}
    (run / "summary.json").write_text(json.dumps(report, indent=2))

    existing = atlas_pids()
    existing_rviz = rviz_pids()
    if existing or existing_rviz:
        report.update(result="BLOCKED", existing_atlas_processes=existing,
                      existing_rviz_pids=existing_rviz)
        (run / "summary.json").write_text(json.dumps(report, indent=2))
        raise SystemExit("Existing Atlas/Autoware or RViz process detected; no processes were started. See summary.json")

    map_dir = Path(env["ATLAS_MAP_DIR"])
    check = subprocess.run(["sha256sum", "-c", str(root/"tools/native_ubuntu/map_assets.sha256")],
                           cwd=map_dir, env=env, stdout=(run/"map_checksums.txt").open("w"),
                           stderr=subprocess.STDOUT)
    if check.returncode:
        report.update(result="BLOCKED", reason="map asset missing or checksum mismatch")
        (run/"summary.json").write_text(json.dumps(report, indent=2))
        raise SystemExit("Required map asset missing or checksum mismatch; see map_checksums.txt")

    children = []
    rosbag = None
    telemetry = None
    rviz = None
    node = None
    try:
        recorder_log = (run/"recorder.log").open("w")
        rosbag = subprocess.Popen(["ros2", "run", "atlas_full_stack_demo",
                                   "record_timing_diagnostics.sh", str(run/"recording")],
                                  cwd=root, env=env, stdout=recorder_log, stderr=subprocess.STDOUT,
                                  start_new_session=True)
        children.append(rosbag)
        time.sleep(2.0)
        telemetry_log = (run/"telemetry.log").open("w")
        telemetry = subprocess.Popen(["ros2", "run", "atlas_full_stack_demo", "scheduler_telemetry.py",
                                      "--output", str(run/"scheduler_telemetry"), "--interval-ms", "50"],
                                     cwd=root, env=env, stdout=telemetry_log,
                                     stderr=subprocess.STDOUT, start_new_session=True)
        children.append(telemetry)
        launch_log = (run/"launch.log").open("w")
        launch = subprocess.Popen(["ros2", "launch", "atlas_full_stack_demo", "full_stack.launch.py",
                                   "obstacle:=false", "request_engagement:=false",
                                   f"results_dir:={results}", f"osm_path:={map_dir/'finalMap_repaired.osm'}",
                                   f"pcd_path:={map_dir/'pointcloud_map_georeferenced.pcd'}",
                                   f"projector_path:={map_dir / 'map_projector_info.yaml'}"],
                                  cwd=root, env=env, stdout=launch_log, stderr=subprocess.STDOUT,
                                  start_new_session=True)
        children.append(launch)
        rclpy.init()
        node = Monitor()
        start_wait = time.monotonic() + 60
        while time.monotonic() < start_wait:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.base and node.truth:
                break
            if launch.poll() is not None:
                raise RuntimeError("Full-stack launch exited during startup")
        if not node.truth:
            raise Inconclusive("Simulator odometry was not observed")

        init_log = (run/"localization_initialization.log").open("w")
        initializer = subprocess.Popen(["ros2", "run", "atlas_full_stack_demo", "initialize_localization.py"],
                                       cwd=root, env=env, stdout=init_log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
        children.append(initializer)
        initializer_deadline = time.monotonic() + 60
        while initializer.poll() is None and time.monotonic() < initializer_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if launch.poll() is not None:
                raise RuntimeError("Full-stack launch exited during localization initialization")
        if initializer.poll() is None:
            raise Inconclusive("Localization-only initializer did not finish within 60 seconds")
        if initializer.returncode:
            raise RuntimeError("Localization-only initializer failed; see localization_initialization.log")
        children.remove(initializer)
        report["phases"]["localization"] = "PASS"

        deadline = time.monotonic() + 90
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if node.base and node.combined and node.base.get("armed") and node.combined.get("armed"):
                break
            if launch.poll() is not None:
                raise RuntimeError("Full-stack launch exited before watchdogs armed")
            for label, status in (("base", node.base), ("combined", node.combined)):
                if status and status.get("latched"):
                    raise RuntimeError(f"{label} watchdog latched before arming")
        if not (node.base and node.combined and node.base.get("armed") and node.combined.get("armed")):
            raise Inconclusive("Both watchdogs did not arm within 90 seconds")
        if not node.tf_available():
            tf_deadline = time.monotonic() + 10
            while time.monotonic() < tf_deadline and not node.tf_available():
                rclpy.spin_once(node, timeout_sec=0.05)
        if not node.tf_available():
            raise Inconclusive("TF map -> base_link unavailable")
        report["phases"]["localization_and_tf"] = "PASS"

        scene_start_index = len(node.scene_records)
        stationary_truth_index = max(0, len(node.truth) - 1)
        reason = spin_for(node, args.hold_sec)
        if reason:
            raise RuntimeError(reason)
        report["phases"]["stationary_safety_120s"] = "PASS"

        for phase, status in (("base", node.base), ("combined", node.combined)):
            if not fresh_status(status, time.monotonic()) or status.get("latched") or not status.get("armed"):
                raise RuntimeError(f"{phase} watchdog unhealthy at stationary hold end")
        stationary_truth = node.truth[stationary_truth_index:]
        if not stationary_truth:
            raise Inconclusive("Simulator odometry was missing during the stationary hold")
        if max(item[2] for item in stationary_truth) > 0.001:
            raise RuntimeError("Simulator speed exceeded 0.001 m/s during observation")
        start_position = stationary_truth[0][3]
        max_displacement = max(math.dist(sample[3], start_position) for sample in stationary_truth)
        if max_displacement > 0.001:
            raise RuntimeError("Simulator displacement exceeded 0.001 m during observation")
        if node.engage_messages:
            raise RuntimeError("An engagement request was observed; simulator must remain disengaged")
        if not node.ndt or not node.ekf:
            raise Inconclusive("NDT or EKF output missing")
        max_ndt_gap = gaps(node.ndt)
        max_ekf_gap = gaps(node.ekf)
        skips = [x for x in node.scene_records[scene_start_index:]
                 if x.get("scan_skipped")]
        report["metrics"] = {"ndt_messages": len(node.ndt), "ekf_messages": len(node.ekf),
                             "sim_odometry_messages": len(node.truth), "scene_timing_messages": len(node.scene_records),
                             "max_ndt_receipt_gap_sec": max_ndt_gap, "max_ekf_receipt_gap_sec": max_ekf_gap,
                             "max_scene_callback_gap_sec": max((x.get("callback_gap_sec") or 0 for x in node.scene_records), default=None),
                             "scene_skips_after_stationary_observation_start": skips,
                             "watchdog_health_violations": node.health_violations,
                             "base_watchdog_history_records": len(node.base_history),
                             "combined_watchdog_history_records": len(node.combined_history),
                             "engagement_requests_observed": node.engage_messages,
                             "final_base_status": node.base, "final_combined_status": node.combined,
                             "simulator_max_speed_mps": max(item[2] for item in stationary_truth),
                             "simulator_max_displacement_from_start_m": max_displacement,
                             "map_to_base_link_available": True}
        for name, gap in (("NDT", max_ndt_gap), ("EKF", max_ekf_gap)):
            if gap is None:
                raise Inconclusive(f"{name} receipt gap could not be computed")
            if gap >= 0.3:
                raise RuntimeError(f"{name} receipt gap reached the 300 ms contract")
        if skips:
            raise RuntimeError("Scene skipped one or more scans after startup")

        if not args.without_rviz:
            if rviz_pids():
                raise RuntimeError("RViz appeared before the planned visualization phase")
            rviz_log = (run/"rviz.log").open("w")
            rviz = subprocess.Popen(["rviz2", "-d", str(root/"src/atlas_full_stack_demo/config/full_stack_lightweight.rviz")],
                                    cwd=root, env=env, stdout=rviz_log, stderr=subprocess.STDOUT,
                                    start_new_session=True)
            children.append(rviz)
            time.sleep(2.0)
            if len(rviz_pids()) != 1:
                raise RuntimeError("Expected exactly one RViz process")
            reason = spin_for(node, args.hold_sec, rviz)
            if reason:
                raise RuntimeError(reason)
            report["phases"]["lightweight_rviz_120s"] = "PASS"
            rviz_truth = node.truth[stationary_truth_index:]
            if not rviz_truth:
                raise Inconclusive("Simulator odometry was missing during RViz observation")
            if max(item[2] for item in rviz_truth) > 0.001:
                raise RuntimeError("Simulator speed exceeded 0.001 m/s during RViz observation")
            if max(math.dist(sample[3], start_position) for sample in rviz_truth) > 0.001:
                raise RuntimeError("Simulator displacement exceeded 0.001 m during RViz observation")
            if node.engage_messages:
                raise RuntimeError("An engagement request was observed during RViz observation")
            if node.health_violations:
                raise RuntimeError("Watchdog health violation occurred during RViz observation")
            if not node.base.get("armed") or not node.combined.get("armed"):
                raise RuntimeError("Watchdog became unarmed during RViz observation")
            if not fresh_status(node.base, time.monotonic()) or not fresh_status(node.combined, time.monotonic()):
                raise RuntimeError("Watchdog status became stale during RViz observation")

        report["result"] = "PASS"
    except KeyboardInterrupt:
        report["result"] = "INTERRUPTED"
        report["failure"] = "Operator interrupted validation; spawned processes will be stopped safely."
    except Inconclusive as exc:
        report["result"] = "INCONCLUSIVE"
        report["failure"] = str(exc)
    except Exception as exc:
        report["result"] = "FAIL"
        report["failure"] = str(exc)
        report["faults"] = [s for s in (getattr(node, "base", None), getattr(node, "combined", None))
                            if s and (s.get("latched") or s.get("currently_stale"))]
    finally:
        if node is not None:
            report.setdefault("metrics", {})
            report["metrics"]["base_watchdog_history_records"] = len(node.base_history)
            report["metrics"]["combined_watchdog_history_records"] = len(node.combined_history)
            report["metrics"]["final_base_status"] = node.base
            report["metrics"]["final_combined_status"] = node.combined
            (run / "watchdog_history.json").write_text(json.dumps({
                "base": node.base_history, "combined": node.combined_history}, indent=2))
        (run/"summary.json").write_text(json.dumps(report, indent=2))
        # Stop only children created by this workflow, allowing rosbag finalization.
        for proc in reversed(children):
            if proc.poll() is None:
                try: os.killpg(proc.pid, signal.SIGINT)
                except ProcessLookupError: pass
                try: proc.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    try: os.killpg(proc.pid, signal.SIGTERM)
                    except ProcessLookupError: pass
                    try: proc.wait(timeout=5)
                    except subprocess.TimeoutExpired: pass
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()
        print(json.dumps(report, indent=2))
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
