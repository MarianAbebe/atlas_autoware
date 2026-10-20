#!/usr/bin/env python3
"""Independent numerical analysis of recorded simulator state, no ROS outputs."""
import argparse,json,math
from pathlib import Path
import numpy as np

def main():
    ap=argparse.ArgumentParser();ap.add_argument('recording',type=Path);args=ap.parse_args();p=args.recording
    r=json.loads((p/'validation.json').read_text());a=np.genfromtxt(p/'odometry.csv',delimiter=',',names=True)
    route=np.genfromtxt(p/'route.csv',delimiter=',',names=True);poly=np.column_stack([route['x'],route['y']]);seg=np.diff(poly,axis=0);lat=[]
    for x,y in zip(a['x'],a['y']):
        rel=np.array([x,y])-poly[:-1];u=np.clip(np.sum(rel*seg,axis=1)/np.sum(seg*seg,axis=1),0,1)
        dist=np.linalg.norm(poly[:-1]+u[:,None]*seg-[x,y],axis=1);i=np.argmin(dist)
        lat.append(abs(seg[i,0]*rel[i,1]-seg[i,1]*rel[i,0])/np.linalg.norm(seg[i]))
    start_gap=math.hypot(a['x'][0]-route['x'][0],a['y'][0]-route['y'][0]);tail=a[a['wall_sec']>=a['wall_sec'][-1]-5]
    diff=[]
    for d in r['controller_debug']:
        i=np.argmin(abs(a['wall_sec']-d['t']))
        if abs(a['wall_sec'][i]-d['t'])<.04:diff.append(abs(a['speed'][i]-d['current_speed']))
    res={'max_lateral_error_m':max(lat),'rms_lateral_error_m':float(np.sqrt(np.mean(np.square(lat)))),
         'recorded_distance_m':r['metrics']['total_distance_m'],'initial_unobserved_start_chord_m':start_gap,
         'distance_including_initial_chord_estimate_m':r['metrics']['total_distance_m']+start_gap,
         'terminal_5s_displacement_m':math.hypot(tail['x'][-1]-tail['x'][0],tail['y'][-1]-tail['y'][0]),
         'terminal_5s_max_abs_speed_mps':float(np.max(np.abs(tail['speed']))),
         'controller_vs_live_speed_rms_mps':float(np.sqrt(np.mean(np.square(diff)))),
         'matched_controller_speed_samples':len(diff),'strict_standstill_speed_threshold_mps':.001,
         'strict_standstill_passed':bool(np.max(np.abs(tail['speed']))<.001)}
    (p/'analysis.json').write_text(json.dumps(res,indent=2));print(json.dumps(res,indent=2))
if __name__=='__main__':main()
