#!/usr/bin/env python3
"""Read-only runtime timing/CPU/pressure evidence around an existing bounded test.
Usage: python3 scripts/trace_regression_runtime.py --output-dir /tmp/fresh -- bash ...
Does not change scheduling, ROS settings, thresholds, or the child command.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def read(path):
    try:return Path(path).read_text()
    except OSError:return None


def processes():
    found=[]
    for p in Path('/proc').glob('[0-9]*'):
        text=read(p/'stat')
        if not text:continue
        right=text.rfind(')'); fields=text[right+2:].split()
        try:found.append(dict(pid=int(p.name),comm=text[text.find('(')+1:right],state=fields[0],
            ppid=int(fields[1]),ticks=int(fields[11])+int(fields[12]),threads=int(fields[17]),
            rss_pages=int(fields[21])))
        except (ValueError,IndexError):continue
    return found


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output-dir',type=Path,required=True)
    ap.add_argument('--cpu-workers',type=int,default=0,help='Diagnostic positive control only; at most 8 workers')
    ap.add_argument('--cpu-after-sec',type=float,default=20)
    ap.add_argument('--cpu-duration-sec',type=float,default=8)
    ap.add_argument('command',nargs=argparse.REMAINDER)
    args=ap.parse_args(); command=args.command
    if command and command[0]=='--':command=command[1:]
    if not command:ap.error('child command required')
    if not 0<=args.cpu_workers<=8 or not 0<args.cpu_duration_sec<=10 or not 0<=args.cpu_after_sec<=45:
        ap.error('CPU control bounds: 0..8 workers, 0..45 s delay, duration >0 and <=10 s')
    args.output_dir.mkdir(parents=True,exist_ok=False)
    meta=dict(command=command,affinity=sorted(os.sched_getaffinity(0)),clock_ticks=os.sysconf('SC_CLK_TCK'),
        page_size=os.sysconf('SC_PAGE_SIZE'),environment={k:v for k,v in os.environ.items()
            if k.startswith(('ROS_','RMW_','FAST','OMP_','OPENBLAS_','MKL_'))},
        cgroup=read('/proc/self/cgroup'),cpu_max=read('/sys/fs/cgroup/cpu.max'),
        cpu_control=dict(workers=args.cpu_workers,delay_sec=args.cpu_after_sec,duration_sec=args.cpu_duration_sec))
    (args.output_dir/'metadata.json').write_text(json.dumps(meta,indent=2))
    with (args.output_dir/'command.log').open('w') as log, (args.output_dir/'timing.jsonl').open('w') as timing:
        began=time.monotonic(); previous=began; detail=0
        child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
        workers=[]; load_started=False
        while True:
            now=time.monotonic()
            row=dict(elapsed=now-began,monotonic=now,realtime=time.time(),sample_gap_sec=now-previous)
            previous=now
            if args.cpu_workers and not load_started and now-began>=args.cpu_after_sec:
                worker_code="import time,sys\nend=time.monotonic()+float(sys.argv[1])\nx=1\nwhile time.monotonic()<end:\n for _ in range(1000):x=(x*13+7)%10000019"
                workers=[subprocess.Popen([sys.executable,'-c',worker_code,str(args.cpu_duration_sec)])
                         for _ in range(args.cpu_workers)]
                row['cpu_control_start']=dict(workers=args.cpu_workers,pids=[p.pid for p in workers])
                load_started=True
            if now>=detail:
                row.update(cpu_stat=read('/proc/stat'),cpu_pressure=read('/proc/pressure/cpu'),
                    memory_pressure=read('/proc/pressure/memory'),meminfo=read('/proc/meminfo'),
                    vmstat=read('/proc/vmstat'),processes=processes(),cgroup_cpu=read('/sys/fs/cgroup/cpu.stat'))
                detail=now+.5
            timing.write(json.dumps(row)+'\n')
            status=child.poll()
            if status is not None:break
            time.sleep(.02)
        for worker in workers:
            if worker.poll() is None:worker.terminate()
            worker.wait(timeout=2)
        (args.output_dir/'exit.json').write_text(json.dumps(dict(exit_code=status,elapsed=time.monotonic()-began)))
    return status

if __name__=='__main__':raise SystemExit(main())
