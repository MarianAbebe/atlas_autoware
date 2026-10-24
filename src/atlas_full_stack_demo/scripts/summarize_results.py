#!/usr/bin/env python3
"""Recheck persisted graph/lineage and compute observer latency and tracking metrics."""
import argparse,json,math
from pathlib import Path
import numpy as np


def stats(v):
    return dict(samples=len(v),median=float(np.median(v)),p95=float(np.percentile(v,95)),maximum=float(max(v))) if v else None


def summarize(root):
    result={};counts={}
    edges=[('lidar/raw','full_stack_scene','livox_converter'),('lidar/extended','livox_converter','distortion_corrector'),('lidar/deskewed','distortion_corrector','compact_converter'),('lidar/compact','compact_converter','ndt/ndt_scan_matcher'),('lidar/compact','compact_converter','ground_filter'),('obstacles/pointcloud','ground_filter','euclidean_cluster'),('clusters','euclidean_cluster','static_objects_adapter'),('obstacles/detected_objects','euclidean_cluster','static_objects_adapter'),('obstacles/predicted_objects','static_objects_adapter','motion_velocity_planner'),('obstacle_trajectory','motion_velocity_planner','velocity_smoother'),('trajectory','velocity_smoother','controller'),('ekf/ekf_odom','ekf/ekf_localizer','controller'),('ndt/ndt_pose_with_covariance','ndt/ndt_scan_matcher','ekf/ekf_localizer'),('gated/control_cmd','gate/vehicle_cmd_gate','sim/simple_planning_simulator')]
    for file in sorted(root.glob('*/validation.json')):
        p=file.parent;validation=json.loads(file.read_text());m=validation['metrics'];graph=json.loads((p/'graph.json').read_text());checks={}
        for topic,pub,sub in edges:
            g=graph.get('/atlas_loc/'+topic,{})
            checks['edge:'+topic+'->'+sub]=g.get('publishers')==['/atlas_loc/'+pub] and '/atlas_loc/'+sub in g.get('subscribers',[])
        forbidden=['controller','ekf/ekf_localizer','ndt/ndt_scan_matcher','motion_velocity_planner','velocity_smoother','gate/vehicle_cmd_gate','static_objects_adapter']
        checks['truth_not_estimator_controller_or_planner']=not any('/atlas_loc/'+n in graph['/atlas_loc/sim/output/odometry']['subscribers'] for n in forbidden)
        stages={k:json.loads((p/(k+'.json')).read_text()) for k in ['raw','extended','deskewed','compact']}
        bystamp={k:{r[1]:r for r in v} for k,v in stages.items()};common=set.intersection(*(set(v) for v in bystamp.values()))
        checks['conversion_keeps_point_count']=len(common)>10 and all(len({bystamp[k][s][2] for k in bystamp})==1 for s in common)
        checks['conversion_keeps_sensor_frame']=all(len({bystamp[k][s][4] for k in bystamp})==1 for s in common)
        raw=bystamp['raw'];ndt=json.loads((p/'ndt.json').read_text());ndt_lat=[r[0]-raw[r[1]][0]+raw[r[1]][3] for r in ndt if r[1] in raw]
        det=json.loads((p/'detected.json').read_text());pred=json.loads((p/'predicted.json').read_text());truth=json.loads((p/'truth.json').read_text());paths=json.loads((p/'trajectory.json').read_text())
        tracking=[]
        if paths:
            line=np.array([v[:2] for v in paths[0]['points']]);a=line[:-1];d=line[1:]-a;good=np.sum(d*d,axis=1)>1e-10;a=a[good];d=d[good]
            for r in truth:
                if m['engaged'] is None or r[0]<m['engaged']:continue
                u=np.clip(np.sum((np.array(r[2:4])-a)*d,axis=1)/np.sum(d*d,axis=1),0,1)
                tracking.append(float(np.linalg.norm(a+u[:,None]*d-r[2:4],axis=1).min()))
        control=json.loads((p/'command.json').read_text());checks['bounded_finite_control']=bool(control) and all(all(math.isfinite(x) for x in r[2:]) and abs(r[2])<=.45 and abs(r[4])<=.501 for r in control)
        checks['bounded_actual_speed']=bool(truth) and all(-.05<=r[6]<=1.1 for r in truth)
        a_safety=json.loads((p/'base_safety.json').read_text());b_safety=json.loads((p/'safety.json').read_text());latched=[dict(stage='localization',**s) for s in a_safety if s.get('latched')]+[dict(stage='perception',**s) for s in b_safety if s.get('latched')]
        first=min(latched,key=lambda r:r['t']) if latched else None
        if m['scenario']=='localization_loss' and truth and m['injected'] is not None:
            checks['ekf_still_publishes_fresh_predictions']=truth[-1][1]-json.loads((p/'ekf.json').read_text())[-1][1]<.3
            checks['perception_available_at_measurement_end']=all(truth[-1][1]-(values[-1]['stamp'] if isinstance(values[-1],dict) else values[-1][1])<.3 for values in [json.loads((p/(k+'.json')).read_text()) for k in ['raw','ground','detected','predicted']])
        row=dict(validation=validation,additional_checks=checks,additional_checks_passed=all(checks.values()),matched_conversion_scans=len(common),first_watchdog_latch=first,latency_sec=dict(acquisition_to_ndt_observer=stats(ndt_lat),acquisition_to_detector_observer=stats([r['age'] for r in det]),acquisition_to_static_objects_observer=stats([r['age'] for r in pred])),tracking=dict(max_distance_to_reference_polyline_m=max(tracking,default=None),rms_distance_to_reference_polyline_m=math.sqrt(sum(x*x for x in tracking)/len(tracking)) if tracking else None),last_accepted_ndt_age_at_measurement_end_sec=truth[-1][1]-ndt[-1][1] if truth and ndt else None)
        if m['first_stop_plan'] is not None and m['first_matched_detection'] is not None:row['first_observed_detection_to_first_observed_stop_plan_sec']=m['first_stop_plan']-m['first_matched_detection']
        result[p.name]=row;scenario=m['scenario'];counts.setdefault(scenario,dict(passed=0,total=0));counts[scenario]['total']+=1;counts[scenario]['passed']+=int(validation['passed'] and all(checks.values()))
    out=dict(scope='Fixed six-case full-stack batch; no reliability guarantee',counts=counts,runs=result,latency_scope='Acquisition to this observer callback includes pipeline, transport and callback scheduling; stop reaction interval is not a traced causal duration.')
    (root/'summary.json').write_text(json.dumps(out,indent=2));return out

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('--root',type=Path,required=True);args=a.parse_args();r=summarize(args.root);print(json.dumps(r['counts'],indent=2))
