#!/usr/bin/env python3
"""Sample host and Atlas Nav2 process CPU, memory, and scheduler counters."""
import argparse
import json
import signal
import sys
import time

from run_ackermann_timing_campaign import read_host, read_processes


stopping = False


def stop(_signal, _frame):
    global stopping
    stopping = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--period', type=float, default=0.5)
    args = parser.parse_args()
    if args.period < 0.1:
        parser.error('period must be at least 0.1 seconds')
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    previous_processes = {}
    previous_host = None
    try:
        with open(args.output, 'w', encoding='utf-8', buffering=1) as stream:
            while not stopping:
                now = time.monotonic()
                host = read_host()
                host['processes'] = read_processes(previous_processes, now)
                host['monotonic_ns'] = time.monotonic_ns()
                if previous_host:
                    ticks = host['cpu_ticks'] - previous_host['ticks']
                    idle = host['cpu_idle_ticks'] - previous_host['idle']
                    host['cpu_busy_pct_all_cores'] = (
                        100.0 * (ticks - idle) / ticks if ticks > 0 else None)
                else:
                    host['cpu_busy_pct_all_cores'] = None
                previous_host = {'ticks': host['cpu_ticks'],
                                 'idle': host['cpu_idle_ticks']}
                stream.write(json.dumps(host, separators=(',', ':')) + '\n')
                time.sleep(args.period)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
