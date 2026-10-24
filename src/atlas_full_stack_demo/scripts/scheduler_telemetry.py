#!/usr/bin/env python3
"""Low-overhead /proc scheduler telemetry for an existing Atlas full-stack graph."""

import argparse
import csv
import json
import os
from pathlib import Path
import platform
import signal
import time


PROCESS_RULES = (
    ("scene", (("scene.py",),)),
    ("combined_watchdog", (("perception_guard.py",), ("obstacle_guard.py",))),
    ("base_watchdog", (("localization_watchdog.py",),)),
    ("ndt", (("autoware_ndt_scan_matcher_node",),)),
    ("ekf", (("autoware_ekf_localizer_node",),)),
    ("motion_planner", (("autoware_motion_velocity_planner_node",),)),
    ("velocity_smoother", (("velocity_smoother_node",),)),
    ("simulator_container", (("component_container", "simulator_container"),)),
    ("perception_container", (("component_container", "perception_container"),)),
    ("controller", (("controller_node_exe",),)),
    # Match the native recorder child, not an arbitrary shell whose command text
    # happens to mention "ros2 bag record".
    ("rosbag_recorder", (("rosbag2_transport/record",),)),
    ("rviz", (("rviz2",),)),
)

PROCESS_FIELDS = (
    "monotonic_ns", "realtime_ns", "clock_read_uncertainty_ns", "sample_gap_ns",
    "label", "pid", "comm", "state", "starttime_ticks", "cpu_runtime_ns",
    "runqueue_wait_ns", "timeslices", "voluntary_context_switches",
    "involuntary_context_switches", "rss_bytes", "threads", "cmdline",
)

SYSTEM_FIELDS = (
    "monotonic_ns", "realtime_ns", "clock_read_uncertainty_ns", "sample_gap_ns",
    "sample_duration_ns", "matched_processes", "loadavg_1", "loadavg_5", "loadavg_15",
    "loadavg_runnable", "loadavg_entities", "cpu_user_ticks", "cpu_nice_ticks",
    "cpu_system_ticks", "cpu_idle_ticks", "cpu_iowait_ticks", "cpu_irq_ticks",
    "cpu_softirq_ticks", "cpu_steal_ticks", "context_switches", "procs_running",
    "procs_blocked", "mem_available_kib", "mem_free_kib", "swap_total_kib",
    "swap_free_kib", "swap_cached_kib", "dirty_kib", "pgmajfault", "pswpin",
    "pswpout", "cpu_pressure_some_avg10", "cpu_pressure_some_total_us",
    "cpu_pressure_full_avg10", "cpu_pressure_full_total_us",
    "memory_pressure_some_avg10", "memory_pressure_some_total_us",
    "memory_pressure_full_avg10", "memory_pressure_full_total_us",
    "io_pressure_some_avg10", "io_pressure_some_total_us", "io_pressure_full_avg10",
    "io_pressure_full_total_us",
)


def read_text(path):
    try:
        return Path(path).read_text()
    except (OSError, UnicodeError):
        return None


def parse_key_values(text):
    result = {}
    for line in (text or "").splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
        else:
            fields = line.split(None, 1)
            if len(fields) != 2:
                continue
            key, value = fields
        result[key] = value.strip()
    return result


def first_int(value):
    try:
        return int(value.split()[0])
    except (AttributeError, ValueError, IndexError):
        return None


def parse_pressure(text):
    result = {}
    for line in (text or "").splitlines():
        fields = line.split()
        if not fields:
            continue
        category = fields[0]
        for item in fields[1:]:
            key, value = item.split("=", 1)
            result[f"{category}_{key}"] = float(value) if key != "total" else int(value)
    return result


def parse_process_stat(text):
    if not text:
        return None
    left, right = text.find("("), text.rfind(")")
    if left < 0 or right <= left:
        return None
    fields = text[right + 2:].split()
    try:
        return {
            "comm": text[left + 1:right], "state": fields[0], "threads": int(fields[17]),
            "starttime_ticks": int(fields[19]), "rss_pages": int(fields[21]),
        }
    except (IndexError, ValueError):
        return None


def classify_process(comm, cmdline):
    haystack = f"{comm} {cmdline}"
    return [label for label, alternatives in PROCESS_RULES
            if any(all(token in haystack for token in tokens) for tokens in alternatives)]


def discover_processes(proc_root=Path("/proc"), explicit=()):
    found = {}
    for entry in proc_root.glob("[0-9]*"):
        stat = parse_process_stat(read_text(entry / "stat"))
        if stat is None:
            continue
        cmdline = (read_text(entry / "cmdline") or "").replace("\0", " ").strip()
        for label in classify_process(stat["comm"], cmdline):
            found[int(entry.name)] = (label, cmdline)
    for label, pid in explicit:
        found[pid] = (label, "explicit")
    return found


def process_sample(proc_root, pid, label, discovered_cmdline, clocks, sample_gap_ns, page_size):
    base = proc_root / str(pid)
    stat = parse_process_stat(read_text(base / "stat"))
    sched = (read_text(base / "schedstat") or "").split()
    status = parse_key_values(read_text(base / "status"))
    if stat is None or len(sched) < 3:
        return None
    return {
        "monotonic_ns": clocks[0], "realtime_ns": clocks[1],
        "clock_read_uncertainty_ns": clocks[2], "sample_gap_ns": sample_gap_ns,
        "label": label, "pid": pid, "comm": stat["comm"], "state": stat["state"],
        "starttime_ticks": stat["starttime_ticks"], "cpu_runtime_ns": int(sched[0]),
        "runqueue_wait_ns": int(sched[1]), "timeslices": int(sched[2]),
        "voluntary_context_switches": first_int(status.get("voluntary_ctxt_switches")),
        "involuntary_context_switches": first_int(status.get("nonvoluntary_ctxt_switches")),
        "rss_bytes": stat["rss_pages"] * page_size, "threads": stat["threads"],
        "cmdline": (read_text(base / "cmdline") or discovered_cmdline.replace(" ", "\0"))
                   .replace("\0", " ").strip(),
    }


def system_sample(proc_root, clocks, sample_gap_ns, sample_duration_ns, matched):
    result = {key: None for key in SYSTEM_FIELDS}
    result.update(monotonic_ns=clocks[0], realtime_ns=clocks[1],
                  clock_read_uncertainty_ns=clocks[2], sample_gap_ns=sample_gap_ns,
                  sample_duration_ns=sample_duration_ns, matched_processes=matched)
    stat_lines = (read_text(proc_root / "stat") or "").splitlines()
    cpu = next((x.split() for x in stat_lines if x.startswith("cpu ")), [])
    cpu_names = ("cpu_user_ticks", "cpu_nice_ticks", "cpu_system_ticks", "cpu_idle_ticks",
                 "cpu_iowait_ticks", "cpu_irq_ticks", "cpu_softirq_ticks", "cpu_steal_ticks")
    for key, value in zip(cpu_names, cpu[1:9]):
        result[key] = int(value)
    for line in stat_lines:
        fields = line.split()
        if fields and fields[0] in {"ctxt", "procs_running", "procs_blocked"}:
            result[{"ctxt": "context_switches"}.get(fields[0], fields[0])] = int(fields[1])
    load = (read_text(proc_root / "loadavg") or "").split()
    if len(load) >= 4:
        result.update(loadavg_1=float(load[0]), loadavg_5=float(load[1]), loadavg_15=float(load[2]))
        runnable, entities = load[3].split("/", 1)
        result.update(loadavg_runnable=int(runnable), loadavg_entities=int(entities))
    mem = parse_key_values(read_text(proc_root / "meminfo"))
    for source, target in (("MemAvailable", "mem_available_kib"), ("MemFree", "mem_free_kib"),
                           ("SwapTotal", "swap_total_kib"), ("SwapFree", "swap_free_kib"),
                           ("SwapCached", "swap_cached_kib"), ("Dirty", "dirty_kib")):
        result[target] = first_int(mem.get(source))
    vm = parse_key_values(read_text(proc_root / "vmstat"))
    for key in ("pgmajfault", "pswpin", "pswpout"):
        result[key] = first_int(vm.get(key))
    for resource in ("cpu", "memory", "io"):
        pressure = parse_pressure(read_text(proc_root / "pressure" / resource))
        for category in ("some", "full"):
            result[f"{resource}_pressure_{category}_avg10"] = pressure.get(f"{category}_avg10")
            result[f"{resource}_pressure_{category}_total_us"] = pressure.get(f"{category}_total")
    return result


def clock_pair():
    before = time.monotonic_ns()
    realtime = time.time_ns()
    after = time.monotonic_ns()
    return ((before + after) // 2, realtime, after - before)


def parse_explicit(values):
    result = []
    for value in values:
        try:
            label, pid = value.rsplit("=", 1)
            result.append((label, int(pid)))
        except (ValueError, TypeError):
            raise argparse.ArgumentTypeError(f"expected LABEL=PID, got {value!r}")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--interval-ms", type=float, default=50.0)
    parser.add_argument("--duration-sec", type=float, default=0.0,
                        help="Zero records until Ctrl+C; positive values provide a bounded capture")
    parser.add_argument("--pid", action="append", default=[], metavar="LABEL=PID",
                        help="Additionally sample an explicit process")
    args = parser.parse_args(argv)
    if not 20 <= args.interval_ms <= 1000:
        parser.error("--interval-ms must be between 20 and 1000")
    if args.duration_sec < 0:
        parser.error("--duration-sec must be nonnegative")
    explicit = parse_explicit(args.pid)
    args.output.mkdir(parents=True, exist_ok=False)
    proc_root = Path("/proc")
    interval_ns = int(args.interval_ms * 1_000_000)
    page_size = os.sysconf("SC_PAGE_SIZE")
    stop = False

    def request_stop(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    began = clock_pair()
    metadata = {
        "format_version": 1, "interval_ms": args.interval_ms,
        "clock_correlation": "Each CSV row contains adjacent CLOCK_MONOTONIC and CLOCK_REALTIME readings; rosbag receipt timestamps use realtime_ns.",
        "initial_monotonic_ns": began[0], "initial_realtime_ns": began[1],
        "initial_clock_read_uncertainty_ns": began[2], "clock_ticks": os.sysconf("SC_CLK_TCK"),
        "page_size": page_size, "boot_id": (read_text(proc_root / "sys/kernel/random/boot_id") or "").strip(),
        "kernel": platform.release(), "machine": platform.machine(), "cpu_count": os.cpu_count(),
        "rules": [{"label": label, "alternatives": alternatives} for label, alternatives in PROCESS_RULES],
        "explicit_pids": explicit,
    }
    (args.output / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(f"Scheduler telemetry output: {args.output}", flush=True)
    print(f"Sampling every {args.interval_ms:g} ms; press Ctrl+C once to stop.", flush=True)
    with (args.output / "processes.csv").open("w", newline="") as process_file, \
            (args.output / "system.csv").open("w", newline="") as system_file:
        process_writer = csv.DictWriter(process_file, fieldnames=PROCESS_FIELDS)
        system_writer = csv.DictWriter(system_file, fieldnames=SYSTEM_FIELDS)
        process_writer.writeheader(); system_writer.writeheader()
        previous = began[0]; deadline = began[0]; discovered = {}; next_discovery = 0; next_flush = 0
        while not stop:
            sample_start = time.monotonic_ns()
            if args.duration_sec and sample_start - began[0] >= args.duration_sec * 1_000_000_000:
                break
            if sample_start >= next_discovery:
                discovered = discover_processes(proc_root, explicit)
                next_discovery = sample_start + 1_000_000_000
            clocks = clock_pair(); gap = clocks[0] - previous; previous = clocks[0]
            matched = 0
            for pid, (label, cmdline) in sorted(discovered.items()):
                row = process_sample(proc_root, pid, label, cmdline, clocks, gap, page_size)
                if row is not None:
                    process_writer.writerow(row); matched += 1
            system_row = system_sample(proc_root, clocks, gap, 0, matched)
            system_row["sample_duration_ns"] = time.monotonic_ns() - sample_start
            system_writer.writerow(system_row)
            if sample_start >= next_flush:
                process_file.flush(); system_file.flush(); next_flush = sample_start + 1_000_000_000
            deadline += interval_ns
            after = time.monotonic_ns()
            if deadline <= after:
                deadline += ((after - deadline) // interval_ns + 1) * interval_ns
            time.sleep(max(0, deadline - time.monotonic_ns()) / 1_000_000_000)
        process_file.flush(); system_file.flush()
    ended = clock_pair()
    metadata.update(final_monotonic_ns=ended[0], final_realtime_ns=ended[1],
                    final_clock_read_uncertainty_ns=ended[2])
    (args.output / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
