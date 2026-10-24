#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import tempfile
import unittest


MODULE = Path(__file__).with_name("scheduler_telemetry.py")
SPEC = importlib.util.spec_from_file_location("scheduler_telemetry", MODULE)
telemetry = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(telemetry)


class SchedulerTelemetryTests(unittest.TestCase):
    def make_process(self, root, pid, comm, cmdline):
        base = root / str(pid); base.mkdir()
        fields = ["S", "1", "1", "1", "0", "0", "0", "0", "0", "0", "0", "4", "6",
                  "0", "0", "0", "0", "3", "0", "12345", "0", "25"]
        (base / "stat").write_text(f"{pid} ({comm}) " + " ".join(fields) + "\n")
        (base / "cmdline").write_bytes(cmdline.replace(" ", "\0").encode() + b"\0")
        (base / "schedstat").write_text("1000 2000 30\n")
        (base / "status").write_text("State:\tS (sleeping)\nvoluntary_ctxt_switches:\t7\nnonvoluntary_ctxt_switches:\t8\n")

    def test_discovery_and_process_sample(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); self.make_process(root, 42, "python3", "/install/perception_guard.py --ros-args")
            found = telemetry.discover_processes(root)
            self.assertEqual(found[42][0], "combined_watchdog")
            row = telemetry.process_sample(root, 42, *found[42], (10, 20, 1), 50, 4096)
            self.assertEqual(row["cpu_runtime_ns"], 1000)
            self.assertEqual(row["runqueue_wait_ns"], 2000)
            self.assertEqual(row["timeslices"], 30)
            self.assertEqual(row["rss_bytes"], 25 * 4096)
            self.assertEqual(row["involuntary_context_switches"], 8)

    def test_component_and_recorder_classification(self):
        self.assertIn("simulator_container", telemetry.classify_process(
            "component_conta", "/opt/ros/humble/lib/rclcpp_components/component_container --ros-args -r __node:=simulator_container"))
        self.assertIn("rosbag_recorder", telemetry.classify_process(
            "record", "/opt/ros/humble/lib/rosbag2_transport/record --ros-args"))
        self.assertNotIn("rosbag_recorder", telemetry.classify_process(
            "bash", "bash -lc echo ros2 bag record"))

    def test_pressure_and_system_parsing(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); (root / "pressure").mkdir()
            (root / "stat").write_text("cpu  1 2 3 4 5 6 7 8 0 0\nctxt 99\nprocs_running 3\nprocs_blocked 1\n")
            (root / "loadavg").write_text("1.25 0.75 0.50 2/100 123\n")
            (root / "meminfo").write_text("MemAvailable: 1000 kB\nMemFree: 500 kB\nSwapTotal: 200 kB\nSwapFree: 50 kB\nSwapCached: 10 kB\nDirty: 4 kB\n")
            (root / "vmstat").write_text("pgmajfault 9\npswpin 7\npswpout 8\n")
            for name in ("cpu", "memory", "io"):
                (root / "pressure" / name).write_text("some avg10=1.00 avg60=2.00 avg300=3.00 total=4\nfull avg10=0.10 avg60=0.20 avg300=0.30 total=5\n")
            row = telemetry.system_sample(root, (1, 2, 3), 4, 5, 6)
            self.assertEqual(row["cpu_steal_ticks"], 8)
            self.assertEqual(row["context_switches"], 99)
            self.assertEqual(row["mem_available_kib"], 1000)
            self.assertEqual(row["pswpout"], 8)
            self.assertEqual(row["cpu_pressure_some_total_us"], 4)
            self.assertEqual(row["memory_pressure_full_avg10"], 0.1)


if __name__ == "__main__":
    unittest.main()
