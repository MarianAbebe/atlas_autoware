#!/usr/bin/env python3
"""Offline evaluation only: interpolate independently recorded 50 Hz truth."""
import argparse
import bisect
import csv
import json
import math
from pathlib import Path


def read(path):
    return [list(map(float,r)) for r in csv.reader(path.open())]


def analyze(run):
    verdict = json.loads((run/'validation.json').read_text())
    metrics = verdict['metrics']
    truth = read(run/'truth.csv')
    stamps = [r[1] for r in truth]
    result = dict(passed=verdict['passed'],failed_checks=verdict['failed_checks'],metrics=metrics)
    result['interpolated_truth_errors'] = {}
    for name in ['ndt','ekf']:
        errors, yaw_errors, intervals = [], [], []
        for r in read(run/(name+'.csv')):
            if metrics['engaged'] is None or r[0] < metrics['engaged']:
                continue
            i=bisect.bisect_left(stamps,r[1])
            if i==0 or i==len(truth):
                continue
            a,b=truth[i-1],truth[i]
            dt=b[1]-a[1]
            if not 0 < dt <= .04:
                continue
            u=(r[1]-a[1])/dt
            p=[a[k]+u*(b[k]-a[k]) for k in [2,3,4]]
            heading=a[5]+u*math.remainder(b[5]-a[5],2*math.pi)
            errors.append([math.hypot(r[2]-p[0],r[3]-p[1]),math.dist(r[2:5],p)])
            yaw_errors.append(abs(math.remainder(r[5]-heading,2*math.pi)))
            intervals.append(dt)
        result['interpolated_truth_errors'][name]=dict(count=len(errors),
            max_xy_m=max((e[0] for e in errors),default=None),
            rms_xy_m=math.sqrt(sum(e[0]**2 for e in errors)/len(errors)) if errors else None,
            max_xyz_m=max((e[1] for e in errors),default=None),
            max_yaw_rad=max(yaw_errors,default=None), max_truth_bracket_sec=max(intervals,default=None))
    if metrics['injected'] is not None:
        injected=metrics['injected']
        safety=json.loads((run/'safety.json').read_text())
        first=next((s for s in safety if s['latched'] and s['t']>=injected),None)
        after=[r for r in truth if r[0]>=injected]
        stopped=next((r for r in after if abs(r[6])<.001),None)
        result['fault']=dict(latch_latency_sec=first['t']-injected if first else None,
            fault_inputs=first['fault_inputs'] if first else [],
            stop_distance_m=math.dist(stopped[2:4],after[0][2:4]) if stopped else None,
            stop_settling_sec=stopped[0]-injected if stopped else None,
            final_speed_mps=truth[-1][6])
    for name in ['truth','ekf','ndt','command']:
        rows=read(run/(name+'.csv'))
        result[name+'_hz']=(len(rows)-1)/(rows[-1][0]-rows[0][0]) if len(rows)>1 else None
    return result


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('suite',type=Path)
    args=ap.parse_args()
    cases=[args.suite] if (args.suite/'validation.json').exists() else sorted(args.suite.iterdir())
    results={p.name:analyze(p) for p in cases if (p/'validation.json').exists()}
    (args.suite/'analysis.json').write_text(json.dumps(results,indent=2))
    print(json.dumps({k:dict(passed=v['passed'],errors=v['interpolated_truth_errors'],fault=v.get('fault')) for k,v in results.items()},indent=2))


if __name__ == '__main__':
    main()
