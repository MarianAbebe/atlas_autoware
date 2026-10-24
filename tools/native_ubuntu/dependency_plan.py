#!/usr/bin/env python3
"""Print the exact rosdep command for the Atlas plus pinned Autoware closures."""
import os
from pathlib import Path
import shlex
import subprocess
import sys

from catkin_pkg.package import parse_package


def main():
    root = Path(__file__).resolve().parents[2]
    autoware = Path(os.environ.get("ATLAS_AUTOWARE_ROOT", root.parent / "references/autoware"))
    bases = [autoware / "src/core", autoware / "src/universe", autoware / "src/external",
             root / "src"]
    if not all(p.is_dir() for p in bases):
        raise SystemExit("Pinned Autoware source or Atlas src is missing; run fetch_autoware.sh first.")
    rows = subprocess.check_output(["colcon", "list", "--base-paths", *map(str, bases)], text=True)
    packages = {}
    for line in rows.splitlines():
        name, path, *_ = line.split("\t")
        packages[name] = Path(path)
    roots = {
        "autoware_map_loader", "autoware_map_projection_loader", "autoware_lanelet2_map_visualizer",
        "autoware_path_generator", "autoware_planning_topic_converter", "autoware_velocity_smoother",
        "autoware_simple_planning_simulator", "autoware_trajectory_follower_node",
        "autoware_vehicle_cmd_gate", "autoware_ndt_scan_matcher", "autoware_ekf_localizer",
        "autoware_motion_velocity_planner", "autoware_motion_velocity_obstacle_stop_module",
        "autoware_ground_filter", "autoware_euclidean_cluster_object_detector",
        "atlas_full_stack_demo", "atlas_localization_closed_loop_demo", "atlas_obstacle_stopping_demo",
        "atlas_perception_planning_demo", "atlas_closed_loop_demo", "atlas_sensor_kit",
        "atlas_distortion_corrector_test", "atlas_lidar_perception_test", "atlas_map_integration_demo",
    }
    missing = sorted(roots - packages.keys())
    if missing:
        raise SystemExit("Packages absent from imported source: " + ", ".join(missing))
    selected = set()

    def visit(name):
        if name in selected:
            return
        selected.add(name)
        pkg = parse_package(packages[name] / "package.xml")
        pkg.evaluate_conditions(dict(os.environ))
        for field in ("build_depends", "buildtool_depends", "build_export_depends",
                      "buildtool_export_depends", "exec_depends"):
            for edge in getattr(pkg, field):
                if edge.evaluated_condition and edge.name in packages:
                    visit(edge.name)

    for name in sorted(roots):
        visit(name)
    paths = sorted((str(packages[name]) for name in selected), key=str)
    command = ["rosdep", "install", "--rosdistro", "humble", "--ignore-src", "-r", "-y",
               "--from-paths", *paths]
    print("Review then run this command to install missing system dependencies:")
    print(shlex.join(command))
    print(f"# Closure contains {len(paths)} source packages. No package installation was run.")


if __name__ == "__main__":
    main()
