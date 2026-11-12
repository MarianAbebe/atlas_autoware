#!/usr/bin/env python3
"""Run sequential full-graph cancellations with lightweight host/process sampling."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


WATCH = ('safety_gate', 'ackermann_adapter', 'ackermann_simulator',
         'controller_server', 'planner_server', 'bt_navigator',
         'behavior_server', 'ackermann_full_graph_fault_trial')
CLK_TCK = os.sysconf(os.sysconf_names['SC_CLK_TCK'])
PAGE_SIZE = os.sysconf('SC_PAGE_SIZE')


def read_host():
    mem = {}
    with open('/proc/meminfo', encoding='ascii') as stream:
        for line in stream:
            key, value = line.split(':', 1)
            mem[key] = int(value.strip().split()[0]) * 1024
    vm = {}
    with open('/proc/vmstat', encoding='ascii') as stream:
        for line in stream:
            key, value = line.split()
            if key in ('pswpin', 'pswpout'):
                vm[key] = int(value)
    with open('/proc/stat', encoding='ascii') as stream:
        cpu = [int(value) for value in next(stream).split()[1:]]
    pressure = {}
    for resource in ('cpu', 'memory', 'io'):
        try:
            pressure[resource] = Path(f'/proc/pressure/{resource}').read_text().strip()
        except OSError:
            pressure[resource] = None
    return {
        'mem_available_bytes': mem.get('MemAvailable'),
        'swap_free_bytes': mem.get('SwapFree'),
        'swap_total_bytes': mem.get('SwapTotal'),
        'swap_pages_in': vm.get('pswpin'),
        'swap_pages_out': vm.get('pswpout'),
        'cpu_ticks': sum(cpu),
        'cpu_idle_ticks': cpu[3] + (cpu[4] if len(cpu) > 4 else 0),
        'loadavg': [float(value) for value in Path('/proc/loadavg').read_text().split()[:3]],
        'pressure': pressure,
    }


def read_processes(previous, now):
    output = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        base = Path('/proc') / entry
        try:
            stat = (base / 'stat').read_text()
            end = stat.rfind(')')
            command_name = stat[stat.find('(') + 1:end]
            fields = stat[end + 2:].split()
            command = (base / 'cmdline').read_bytes().replace(
                b'\0', b' ').decode(errors='replace').strip()
            if not any(token in command or token in command_name for token in WATCH):
                continue
            pid = int(entry)
            ticks = int(fields[11]) + int(fields[12])
            rss = int(fields[21]) * PAGE_SIZE
            old = previous.get(pid)
            cpu_pct = (None if old is None else
                       max(0.0, (ticks - old[0]) / CLK_TCK / (now - old[1]) * 100.0))
            previous[pid] = (ticks, now)
            schedstat = None
            try:
                values = [int(value) for value in (base / 'schedstat').read_text().split()]
                schedstat = {'runtime_ns': values[0], 'runqueue_wait_ns': values[1],
                             'timeslices': values[2]}
            except (OSError, ValueError, IndexError):
                pass
            context_switches = {}
            try:
                for line in (base / 'status').read_text().splitlines():
                    if line.startswith(('voluntary_ctxt_switches:',
                                        'nonvoluntary_ctxt_switches:')):
                        key, value = line.split(':', 1)
                        context_switches[key] = int(value.strip())
            except (OSError, ValueError):
                pass
            output.append({'pid': pid, 'name': command_name, 'command': command[:240],
                           'rss_bytes': rss, 'cpu_pct_one_core': cpu_pct,
                           'schedstat': schedstat,
                           'context_switches': context_switches})
        except (OSError, ProcessLookupError, PermissionError, IndexError, ValueError):
            continue
    return output


def percentile(values, fraction):
    values = sorted(values)
    if not values:
        return None
    index = (len(values) - 1) * fraction
    lower = int(index)
    upper = min(lower + 1, len(values) - 1)
    return values[lower] + (values[upper] - values[lower]) * (index - lower)


def terminate(process):
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def active_nav2_processes():
    markers = ('/safety_gate ', '/ackermann_adapter ', '/ackermann_simulator ',
               '/controller_server ', '/planner_server ', '/bt_navigator ',
               '/behavior_server ', 'ackermann_full_graph_fault_trial')
    found = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        try:
            command = Path('/proc', entry, 'cmdline').read_bytes().replace(
                b'\0', b' ').decode(errors='replace').strip()
            if any(marker in command for marker in markers):
                found.append({'pid': int(entry), 'command': command[:240]})
        except (OSError, ProcessLookupError, PermissionError):
            continue
    return found


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trials', type=int, default=20)
    parser.add_argument('--domain-start', type=int, default=80)
    parser.add_argument('--sample-period', type=float, default=0.5)
    parser.add_argument('--evidence-root', type=Path, required=True)
    args = parser.parse_args(argv)
    if args.trials < 1 or args.domain_start < 0 or \
            args.domain_start + args.trials - 1 > 232:
        parser.error('trial domains must be a nonempty range within 0..232')
    if args.sample_period < 0.1:
        parser.error('sample period must be at least 0.1 seconds')
    if not os.environ.get('COLCON_PREFIX_PATH'):
        parser.error('source the ROS 2 Humble underlay and development overlay first')
    args.evidence_root.mkdir(parents=True, exist_ok=True)
    summary = {'requested_trials': args.trials, 'domain_start': args.domain_start,
               'sample_period_sec': args.sample_period, 'trials': [],
               'resource_guard_triggered': False}
    for index in range(args.trials):
        domain = args.domain_start + index
        trial_dir = args.evidence_root / f'trial_{index + 1:02d}_domain{domain}'
        trial_dir.mkdir(exist_ok=False)
        capture_root = trial_dir / 'fault_evidence'
        resource_path = trial_dir / 'resources.jsonl'
        log_path = trial_dir / 'runner.log'
        env = os.environ.copy()
        env.update({'ROS_DOMAIN_ID': '211', 'ROS_LOCALHOST_ONLY': '1'})
        command = ['ros2', 'run', 'atlas_nav2_demo', 'validate_ackermann_faults.py',
                   '--fault', 'cancel', '--domain-start', str(domain),
                   '--evidence-root', str(capture_root)]
        previous_processes = {}
        previous_host = None
        child = None
        guard_hit = False
        start = time.monotonic()
        try:
            with log_path.open('w', encoding='utf-8') as log, \
                    resource_path.open('w', encoding='utf-8', buffering=1) as samples:
                child = subprocess.Popen(command, env=env, stdout=log,
                                         stderr=subprocess.STDOUT, start_new_session=True)
                while child.poll() is None:
                    now = time.monotonic()
                    host = read_host()
                    processes = read_processes(previous_processes, now)
                    if previous_host is not None:
                        tick_delta = host['cpu_ticks'] - previous_host['ticks']
                        idle_delta = host['cpu_idle_ticks'] - previous_host['idle']
                        host['cpu_busy_pct_all_cores'] = (
                            100.0 * (tick_delta - idle_delta) / tick_delta
                            if tick_delta > 0 else None)
                    else:
                        host['cpu_busy_pct_all_cores'] = None
                    previous_host = {'ticks': host['cpu_ticks'],
                                     'idle': host['cpu_idle_ticks']}
                    host.update({'monotonic_ns': time.monotonic_ns(), 'trial': index + 1,
                                 'domain': domain, 'processes': processes})
                    samples.write(json.dumps(host, separators=(',', ':')) + '\n')
                    if (host['mem_available_bytes'] is not None and
                            host['mem_available_bytes'] < 350 * 1024 * 1024) or \
                            (host['swap_free_bytes'] is not None and
                             host['swap_free_bytes'] < 256 * 1024 * 1024):
                        guard_hit = True
                        summary['resource_guard_triggered'] = True
                        terminate(child)
                        break
                    time.sleep(args.sample_period)
                return_code = child.wait()
        except KeyboardInterrupt:
            if child is not None:
                terminate(child)
            summary['interrupted'] = True
            return_code = 130
        result_path = capture_root / 'fault_summary.json'
        result = None
        if result_path.exists():
            try:
                result = json.loads(result_path.read_text())
            except json.JSONDecodeError:
                pass
        trial_result = {'trial': index + 1, 'domain': domain,
                        'duration_sec': time.monotonic() - start,
                        'exit_code': return_code, 'resource_guard_hit': guard_hit,
                        'passed': bool(result and result.get('passed')),
                        'fault_summary': str(result_path),
                        'resources': str(resource_path), 'runner_log': str(log_path)}
        if result and result.get('results'):
            metrics = result['results'][0].get('metrics', {})
            trial_result['metrics'] = {
                name: metrics.get(name) for name in (
                    'cancel_to_action_result_monotonic_s',
                    'cancel_to_gate_internal_zero_publish_monotonic_s',
                    'cancel_to_first_safe_zero_receipt_monotonic_s',
                    'cancel_to_adapter_zero_control_publish_monotonic_s',
                    'cancel_to_simulator_zero_target_applied_monotonic_s',
                    'cancel_to_simulated_standstill_monotonic_s',
                    'post_fault_travel_distance_m',
                    'conservative_stopping_envelope_m',
                    'stopping_envelope_respected')}
            for key in ('cancel_to_gate_internal_zero_publish_monotonic_s',
                        'cancel_to_simulator_zero_target_applied_monotonic_s'):
                value = metrics.get(key)
                if value is None or value > 0.25:
                    trial_result['passed'] = False
        summary['trials'].append(trial_result)
        quiet_deadline = time.monotonic() + 10.0
        active = active_nav2_processes()
        while active and time.monotonic() < quiet_deadline:
            time.sleep(0.2)
            active = active_nav2_processes()
        if active:
            trial_result['orphan_nav2_processes'] = active
            trial_result['passed'] = False
            summary['orphan_process_check_failed_after_trial'] = index + 1
        (args.evidence_root / 'campaign_summary.json').write_text(
            json.dumps(summary, indent=2) + '\n', encoding='utf-8')
        print(json.dumps(trial_result), flush=True)
        if not trial_result['passed'] or guard_hit:
            break
    latencies = [trial.get('metrics', {}).get(
        'cancel_to_gate_internal_zero_publish_monotonic_s')
        for trial in summary['trials']
        if trial.get('metrics', {}).get(
            'cancel_to_gate_internal_zero_publish_monotonic_s') is not None]
    summary['completed_trials'] = len(summary['trials'])
    summary['gate_zero_latency_sec'] = {
        'n': len(latencies), 'median': percentile(latencies, 0.5),
        'p95': percentile(latencies, 0.95), 'maximum': max(latencies, default=None),
        'exceed_250ms': sum(value > 0.25 for value in latencies)}
    summary['passed'] = (len(summary['trials']) == args.trials and
                         not summary['resource_guard_triggered'] and
                         all(trial['passed'] for trial in summary['trials']))
    (args.evidence_root / 'campaign_summary.json').write_text(
        json.dumps(summary, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(summary, indent=2))
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
