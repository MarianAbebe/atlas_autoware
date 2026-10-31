#!/usr/bin/env python3
"""Bounded independent full-stack evaluator. Never publishes obstacle results/control."""
import argparse,bisect,csv,hashlib,json,math,os,signal,subprocess,time,sys
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import SetBool
from autoware_vehicle_msgs.msg import Engage
from autoware_control_msgs.msg import Control
from autoware_planning_msgs.msg import Trajectory
from autoware_perception_msgs.msg import DetectedObjects,PredictedObjects
from autoware_internal_debug_msgs.msg import Float32Stamped
ROOT=Path(__file__).resolve().parents[3];NS='/atlas_loc/'
sys.path.insert(0,str(ROOT/'src/atlas_localization_closed_loop_demo/scripts'))
from measurement_cadence import MeasurementCadence
from run_validation import stamp,state
# This file's module name is __main__; imported run_validation above is the existing localization evaluator.


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--scenario',choices=['obstacle','clear','loss','localization_loss'],default='obstacle');ap.add_argument('--output',type=Path,required=True);ap.add_argument('--timeout',type=float,default=60.)
    args=ap.parse_args()
    if not 25<=args.timeout<=90:ap.error('timeout must be 25..90 s')
    args.output.mkdir(parents=True,exist_ok=False)
    source_files=list((ROOT/'src/atlas_full_stack_demo').rglob('*.py'))+list((ROOT/'src/atlas_full_stack_demo').rglob('*.yaml'))+list((ROOT/'src/atlas_full_stack_demo').rglob('*.sh'))
    (args.output/'source_sha256.json').write_text(json.dumps({str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in source_files},indent=2))
    rclpy.init();node=rclpy.create_node('full_stack_evaluator');start=time.monotonic();capturing=True
    rows={k:[] for k in ['truth','ekf','ndt','command','gate','raw','extended','deskewed','compact','ground','clusters','detected','predicted','plan','trajectory','ndt_ms','ndt_score']}
    safety=[];base_safety=[];pipeline_diagnostics={'scene':[],'object_adapter':[]};scene={};events=[];graph={};samples={};finite=True
    def receive(name,m):
        nonlocal finite
        if not capturing:return
        t=time.monotonic()-start;now=node.get_clock().now().nanoseconds*1e-9
        if name in ['truth','ekf','ndt']:
            values=state(m.pose.pose);speed=m.twist.twist.linear.x if name!='ndt' else 0.
            finite &= all(math.isfinite(v) for v in values+[speed]);rows[name].append([t,stamp(m.header.stamp),*values,speed])
        elif name in ['command','gate']:rows[name].append([t,stamp(m.stamp),m.lateral.steering_tire_angle,m.longitudinal.velocity,m.longitudinal.acceleration])
        elif name in ['raw','extended','deskewed','compact','ground','clusters']:
            fields={f.name:f for f in m.fields};dtype=np.dtype(dict(names=['x','y','z'],formats=['<f4']*3,offsets=[fields[k].offset for k in ['x','y','z']],itemsize=m.point_step))
            a=np.frombuffer(m.data,dtype=dtype);finite &= all(np.isfinite(a[k]).all() for k in ['x','y','z'])
            rows[name].append([t,stamp(m.header.stamp),m.width*m.height,now-stamp(m.header.stamp),m.header.frame_id])
            if name=='raw' and (not samples or t-next(reversed(samples.values()))['receipt']>.5):samples[str(stamp(m.header.stamp))]=dict(receipt=t,points=[[float(a[k][i]) for k in ['x','y','z']] for i in [0,len(a)//2,len(a)-1]])
        elif name in ['detected','predicted']:
            poses=[]
            for o in m.objects:
                p=(o.kinematics.pose_with_covariance if name=='detected' else o.kinematics.initial_pose_with_covariance).pose.position
                vals=[p.x,p.y,p.z,o.shape.dimensions.x,o.shape.dimensions.y,o.shape.dimensions.z];finite &= all(math.isfinite(v) for v in vals);poses.append(vals)
            rows[name].append(dict(t=t,stamp=stamp(m.header.stamp),frame=m.header.frame_id,count=len(m.objects),objects=poses,age=now-stamp(m.header.stamp)))
        elif name in ['plan','trajectory']:
            rows[name].append(dict(t=t,stamp=stamp(m.header.stamp),frame=m.header.frame_id,points=[[p.pose.position.x,p.pose.position.y,p.longitudinal_velocity_mps] for p in m.points]))
        else:rows[name].append([t,stamp(m.stamp),m.data])
    mapping={'truth':('sim/output/odometry',Odometry),'ekf':('ekf/ekf_odom',Odometry),'ndt':('ndt/ndt_pose_with_covariance',PoseWithCovarianceStamped),'command':('control_cmd',Control),'gate':('gated/control_cmd',Control),'raw':('lidar/raw',PointCloud2),'extended':('lidar/extended',PointCloud2),'deskewed':('lidar/deskewed',PointCloud2),'compact':('lidar/compact',PointCloud2),'ground':('obstacles/pointcloud',PointCloud2),'clusters':('clusters',PointCloud2),'detected':('obstacles/detected_objects',DetectedObjects),'predicted':('obstacles/predicted_objects',PredictedObjects),'plan':('obstacle_trajectory',Trajectory),'trajectory':('trajectory',Trajectory),'ndt_ms':('ndt/exe_time_ms',Float32Stamped),'ndt_score':('ndt/nearest_voxel_transformation_likelihood',Float32Stamped)}
    subs=[node.create_subscription(typ,NS+topic,lambda m,k=k:receive(k,m),qos_profile_sensor_data if typ==PointCloud2 else 100) for k,(topic,typ) in mapping.items()]
    node.create_subscription(String,NS+'safety_status',lambda m:safety.append(dict(json.loads(m.data),t=time.monotonic()-start)) if capturing else None,100)
    node.create_subscription(String,NS+'base_safety_status',lambda m:base_safety.append(dict(json.loads(m.data),t=time.monotonic()-start)) if capturing else None,100)
    node.create_subscription(String,NS+'diagnostics/scene_timing',lambda m:pipeline_diagnostics['scene'].append(dict(json.loads(m.data),observer_receipt_monotonic_sec=time.monotonic())) if capturing else None,10)
    node.create_subscription(String,NS+'diagnostics/object_adapter_timing',lambda m:pipeline_diagnostics['object_adapter'].append(dict(json.loads(m.data),observer_receipt_monotonic_sec=time.monotonic())) if capturing else None,10)
    node.create_subscription(String,NS+'scene_info',lambda m:scene.update(json.loads(m.data)),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    boot=node.create_publisher(PoseWithCovarianceStamped,NS+'bootstrap',10);engage=node.create_publisher(Engage,NS+'sim/input/engage',10);fault=node.create_publisher(String,NS+'sensor_fault',10);shutdown=node.create_publisher(String,NS+'test_shutdown',10)
    clients={n:node.create_client(SetBool,NS+n+'/trigger_node_srv') for n in ['ndt','ekf']}
    p=None;error=None;engaged=injected=hold=None;reason='timeout';peak=0;cadence=MeasurementCadence();last_progress=0
    def spin_until(pred,seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.01)
            if pred():return True
            if p.poll() is not None:return False
        return False
    with (args.output/'runtime.log').open('w') as log:
        p=subprocess.Popen(['ros2','launch','atlas_full_stack_demo','full_stack.launch.py',f'obstacle:={str(args.scenario=="obstacle").lower()}',f'results_dir:={args.output}'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            if not spin_until(lambda:bool(scene) and bool(rows['truth']),15):raise RuntimeError('campus scene/simulator discovery timeout')
            for name in ['ekf','ndt']:
                if not clients[name].wait_for_service(timeout_sec=6):raise RuntimeError(name+' activation missing')
                f=clients[name].call_async(SetBool.Request(data=True))
                if not spin_until(f.done,5) or not f.result().success:raise RuntimeError(name+' activation failed')
            b=PoseWithCovarianceStamped();b.header.frame_id='map';b.header.stamp=node.get_clock().now().to_msg();b.pose.pose.position.x=scene['origin'][0]+.1;b.pose.pose.position.y=scene['origin'][1]+.1;b.pose.pose.position.z=scene['origin'][2];b.pose.pose.orientation.z=math.sin(scene['yaw']/2);b.pose.pose.orientation.w=math.cos(scene['yaw']/2)
            for k,v in [(0,.25),(7,.25),(14,.0025),(21,.0006),(28,.0006),(35,.06)]:b.pose.covariance[k]=v
            boot.publish(b);events.append(dict(kind='single_biased_bootstrap',t=time.monotonic()-start))
            while time.monotonic()-start<args.timeout and p.poll() is None:
                rclpy.spin_once(node,timeout_sec=.01);now=time.monotonic()-start
                if now-last_progress>5:
                    print(json.dumps(dict(t=now,scenario=args.scenario,counts={k:len(rows[k]) for k in ['truth','ndt','detected','plan']},armed=safety[-1].get('armed') if safety else False)),flush=True);last_progress=now
                if not graph and now>5:
                    for topic in [NS+t for t,_ in mapping.values()]+['/pcd_loader_service']:
                        if topic=='/pcd_loader_service':continue
                        graph[topic]={kind:sorted(set(i.node_namespace.rstrip('/')+'/'+i.node_name for i in (node.get_publishers_info_by_topic(topic) if kind=='publishers' else node.get_subscriptions_info_by_topic(topic)))) for kind in ['publishers','subscribers']}
                    graph['controller_subscriptions']=[t for t,_ in node.get_subscriber_names_and_types_by_node('controller','/atlas_loc')]
                if cadence.due(now):
                    total=0
                    for st in Path('/proc').glob('[0-9]*/status'):
                        try:
                            if os.getpgid(int(st.parent.name))==p.pid:total+=next((int(l.split()[1]) for l in st.read_text().splitlines() if l.startswith('VmRSS:')),0)
                        except (OSError,ProcessLookupError):pass
                    peak=max(peak,total)
                if engaged is None and safety and safety[-1]['armed'] and len(rows['ndt'])>=10:
                    engage.publish(Engage(stamp=node.get_clock().now().to_msg(),engage=True));engaged=now;events.append(dict(kind='engaged',t=now))
                if engaged is None or not rows['truth']:continue
                r=rows['truth'][-1];travel=math.dist(r[2:4],scene['origin'][:2])
                if args.scenario=='obstacle':
                    delta=np.array(scene['obstacle_center'][:2])-np.array(r[2:4])
                    if float(delta@np.array([math.cos(r[5]),math.sin(r[5])]))-1.9<.1:
                        reason='clearance_abort';break
                if args.scenario in ['loss','localization_loss'] and injected is None and travel>1.:
                    injected=now
                    if args.scenario=='loss':
                        fault.publish(String(data='drop_ndt'));events.append(dict(kind='drop_shared_lidar',t=now))
                    else:
                        f=clients['ndt'].call_async(SetBool.Request(data=False))
                        if not spin_until(f.done,3) or not f.result().success:raise RuntimeError('NDT deactivation failed')
                        events.append(dict(kind='pinned_ndt_deactivated_lidar_continues',t=now))
                if args.scenario in ['loss','localization_loss']:
                    if injected is not None and now-injected>12:reason='loss_stop_observed';break
                    continue
                if args.scenario=='obstacle':ready=travel>1. and abs(r[6])<.001
                else:ready=math.dist(r[2:4],scene['goal'][:2])<.15 and abs(r[6])<.001
                if ready:
                    if hold is None:hold=r
                    if r[0]-hold[0]>=10 and r[1]-hold[1]>=10:reason='stopped_hold_completed';break
                else:hold=None
        except Exception as exc:error=repr(exc)
        finally:
            log.write('\nATLAS_MEASUREMENT_COMPLETE\n');log.flush()
            capturing=False;engage.publish(Engage(stamp=node.get_clock().now().to_msg(),engage=False));shutdown.publish(String(data='measurement_complete'))
            if p.poll() is None:
                os.kill(p.pid,signal.SIGINT)
                try:p.wait(timeout=12)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=6)
    def pub(topic):return graph.get(NS+topic,{}).get('publishers',[])
    checks=dict(no_runtime_error=error is None,all_stage_outputs=all(rows[k] for k in rows),finite=bool(finite),engaged=engaged is not None,
      mpc_exclusively_ekf=pub('ekf/ekf_odom')==['/atlas_loc/ekf/ekf_localizer'] and '/atlas_loc/controller' in graph.get(NS+'ekf/ekf_odom',{}).get('subscribers',[]),
      no_truth_fallback=not any(x in graph.get(NS+'sim/output/odometry',{}).get('subscribers',[]) for x in ['/atlas_loc/controller','/atlas_loc/ekf/ekf_localizer','/atlas_loc/ndt/ndt_scan_matcher','/atlas_loc/motion_velocity_planner']),
      ndt_from_preprocessing=pub('lidar/compact')==['/atlas_loc/compact_converter'] and '/atlas_loc/ndt/ndt_scan_matcher' in graph.get(NS+'lidar/compact',{}).get('subscribers',[]),
      detection_exclusively_cluster=pub('obstacles/detected_objects')==['/atlas_loc/euclidean_cluster'],objects_exclusively_adapter=pub('obstacles/predicted_objects')==['/atlas_loc/static_objects_adapter'],
      plan_exclusively_pinned=pub('obstacle_trajectory')==['/atlas_loc/motion_velocity_planner'],ndt_converged=bool(rows['ndt_score']) and min(r[2] for r in rows['ndt_score'])>=2.3)
    truth=rows['truth'];stamps=[r[1] for r in truth];errors={}
    for name in ['ndt','ekf']:
        e=[]
        for r in rows[name]:
            if engaged is None or r[0]<engaged:continue
            i=bisect.bisect_left(stamps,r[1]);cand=truth[max(0,i-1):min(len(truth),i+1)]
            if cand:
                q=min(cand,key=lambda q:abs(q[1]-r[1]))
                if abs(q[1]-r[1])<.025:e.append(math.dist(r[2:4],q[2:4]))
        errors[name]=dict(matched=len(e),max_xy_m=max(e,default=None),rms_xy_m=math.sqrt(sum(x*x for x in e)/len(e)) if e else None)
    checks['localization_accuracy']=all(v['matched']>10 and v['max_xy_m']<.15 for v in errors.values())
    cloud_sets={k:{r[1] for r in rows[k]} for k in ['raw','extended','deskewed','compact','ground','clusters']}
    checks['source_stamp_lineage']=all(bool(cloud_sets[a]&cloud_sets[b]) for a,b in [('raw','extended'),('extended','deskewed'),('deskewed','compact'),('compact','ground'),('ground','clusters')])
    checks['ndt_acquisition_lineage']=bool(rows['ndt']) and all(any(abs(r[1]-t)<1e-6 for t in cloud_sets['compact']) for r in rows['ndt'])
    detections_by_stamp={r['stamp']:r for r in rows['detected']}
    checks['predictions_from_real_detections']=bool(rows['predicted']) and all(r['stamp'] in detections_by_stamp and r['count']==detections_by_stamp[r['stamp']]['count'] for r in rows['predicted'])
    checks['measured_positive_shapes']=all(all(v>0 for v in o[3:]) for r in rows['predicted'] for o in r['objects'])
    checks['ekf_receives_ndt']='/atlas_loc/ekf/ekf_localizer' in graph.get(NS+'ndt/ndt_pose_with_covariance',{}).get('subscribers',[])
    checks['planner_receives_objects']='/atlas_loc/motion_velocity_planner' in graph.get(NS+'obstacles/predicted_objects',{}).get('subscribers',[])
    checks['controller_receives_planned_trajectory']='/atlas_loc/controller' in graph.get(NS+'trajectory',{}).get('subscribers',[]) and pub('trajectory')==['/atlas_loc/velocity_smoother']
    checks['ground_exclusively_pinned']=pub('obstacles/pointcloud')==['/atlas_loc/ground_filter']
    checks['raw_exclusively_fixture']=pub('lidar/raw')==['/atlas_loc/full_stack_scene']
    world_samples=[]
    for ts,record in samples.items():
        cand=[q for q in truth if abs(q[1]-float(ts))<1e-6]
        if not cand:continue
        q=cand[0];c,si=math.cos(q[5]),math.sin(q[5]);world_samples.append([[q[2]+c*x-si*y,q[3]+si*x+c*y,q[4]+z] for x,y,z in record['points']])
    residual=max((math.dist(a,b) for w in world_samples for a,b in zip(w,world_samples[0])),default=None)
    first_sample=next(iter(samples.values()))['points'] if samples else []
    displacement=max((math.dist(a,b) for r in samples.values() for a,b in zip(r['points'],first_sample)),default=0)
    checks['map_consistent_evolving_scans']=residual is not None and residual<.001 and displacement>.5
    checks['no_child_exception_during_measurement']='Traceback' not in (args.output/'runtime.log').read_text().split('ATLAS_MEASUREMENT_COMPLETE')[0]
    first_latch=next((s for s in base_safety+safety if s.get('latched')),None)
    detected_match=[r for r in rows['predicted'] if scene and any(math.dist(o[:2],scene['obstacle_center'][:2])<1. for o in r['objects'])]
    clearance=None;tracking=None;stop_insertions=[]
    if truth and scene:
        last=truth[-1];q=np.array(last[2:4]);ref=rows['trajectory'][0]['points'] if rows['trajectory'] else [];path=np.array([x[:2] for x in ref])
        if len(path):tracking=max(float(np.linalg.norm(path-np.array(r[2:4]),axis=1).min()) for r in truth)
        a=last[5];delta=np.array(scene['obstacle_center'][:2])-q;clearance=float(delta@np.array([math.cos(a),math.sin(a)]))-1.5-.4
        for r in rows['plan']:
            if any(p[2]==0 and math.dist(p[:2],scene['goal'][:2])>2 and math.dist(p[:2],scene['origin'][:2])>1 for p in r['points']):stop_insertions.append(r['t'])
    if args.scenario in ['loss','localization_loss']:
        f=next((s for s in base_safety if injected is not None and s.get('latched') and s['t']>=injected),None)
        checks.update(loss_injected=injected is not None,loss_latched=bool(f),accepted_ndt_became_stale=injected is not None and any(s['t']>=injected and 'ndt' in s.get('currently_stale',[]) for s in base_safety),no_latch_before_injection=injected is not None and not any(s.get('latched') and s['t']<injected for s in base_safety+safety),loss_stop=bool(truth) and max(abs(r[6]) for r in truth if r[0]>truth[-1][0]-5)<.001,loss_observed=reason=='loss_stop_observed')
        if args.scenario=='localization_loss':
            checks['perception_continues_after_ndt_loss']=injected is not None and all(any((r['t'] if isinstance(r,dict) else r[0])>injected+8 for r in rows[k]) for k in ['raw','ground','detected','predicted'])
    else:
        checks.update(hold_complete=reason=='stopped_hold_completed',no_watchdog_latch=first_latch is None)
        if args.scenario=='obstacle':checks.update(obstacle_detected=bool(detected_match),planner_inserted_stop=bool(stop_insertions),safe_clearance=clearance is not None and clearance>.1)
        else:checks['clear_has_no_spurious_early_stop']=not stop_insertions
    def gap_summary(records):
        receipt=lambda r:r.get('t') if isinstance(r,dict) else r[0]
        age=lambda r:r.get('age_sec',r.get('age')) if isinstance(r,dict) else r[3]
        gaps=[receipt(b)-receipt(a) for a,b in zip(records,records[1:]) if receipt(b)>=receipt(a)]
        ages=[age(r) for r in records if age(r) is not None]
        return {'messages':len(records),'max_receipt_gap_sec':max(gaps,default=None),'p95_receipt_gap_sec':float(np.percentile(gaps,95)) if gaps else None,'gaps_over_freshness_sec':sum(g>=.3 for g in gaps),'max_header_age_sec':max(ages,default=None)}
    stage_timing={k:gap_summary(rows[k]) for k in ['raw','extended','deskewed','compact','ground','clusters','detected','predicted']}
    scene_timing=pipeline_diagnostics['scene']
    adapter_timing=pipeline_diagnostics['object_adapter']
    guard_timing=[s.get('instrumentation',{}) for s in safety]
    guard_input_names=set(name for q in guard_timing for name in (q.get('inputs') or {}))
    guard_inputs={name:{'max_stamp_age_at_receipt_sec':max((q['inputs'][name].get('stamp_age_at_receipt_sec') for q in guard_timing if name in (q.get('inputs') or {}) and q['inputs'][name].get('stamp_age_at_receipt_sec') is not None),default=None),'max_receipt_age_sec':max((q['inputs'][name].get('receipt_age_sec') for q in guard_timing if name in (q.get('inputs') or {}) and q['inputs'][name].get('receipt_age_sec') is not None),default=None),'max_interreceipt_gap_sec':max((q['inputs'][name].get('receipt_gap_sec') for q in guard_timing if name in (q.get('inputs') or {}) and q['inputs'][name].get('receipt_gap_sec') is not None),default=None)} for name in sorted(guard_input_names)}
    metrics=dict(scenario=args.scenario,termination=reason,error=error,engaged=engaged,injected=injected,peak_process_rss_kib=peak,localization_errors=errors,final_goal_error_m=math.dist(truth[-1][2:4],scene['goal'][:2]) if truth and scene else None,obstacle_clearance_m=clearance if args.scenario=='obstacle' else None,scan_world_residual_m=residual,scan_sample_displacement_m=displacement,max_tracking_sample_distance_m=tracking,counts={k:len(v) for k,v in rows.items()},first_latch=first_latch,first_matched_detection=detected_match[0]['t'] if detected_match else None,first_stop_plan=min(stop_insertions,default=None),stage_timing=stage_timing,scene_timer={'records':len(scene_timing),'max_timer_gap_sec':max((r.get('callback_gap_sec') or 0 for r in scene_timing),default=None),'max_schedule_delay_sec':max((r.get('timer_schedule_delay_sec') or 0 for r in scene_timing),default=None),'max_previous_callback_overrun_sec':max((r.get('previous_callback_overrun_sec') or 0 for r in scene_timing),default=None),'max_odometry_callback_duration_sec':max((r.get('odometry_callback_max_duration_sec') or 0 for r in scene_timing),default=None),'max_point_generation_sec':max((r.get('point_generation_sec') or 0 for r in scene_timing),default=None),'max_coordinate_transform_sec':max((r.get('coordinate_transform_sec') or 0 for r in scene_timing),default=None),'max_pointcloud_serialization_sec':max((r.get('pointcloud_serialization_sec') or 0 for r in scene_timing),default=None),'max_callback_duration_sec':max((r.get('callback_duration_sec') or 0 for r in scene_timing),default=None),'max_scan_generation_sec':max((r.get('scan_generation_duration_sec') or 0 for r in scene_timing),default=None),'max_scan_publish_gap_sec':max((r.get('scan_publish_gap_sec') or 0 for r in scene_timing),default=None),'skipped_scans':sum(bool(r.get('scan_skipped')) for r in scene_timing),'skip_reasons':{reason:sum(r.get('skip_reason')==reason for r in scene_timing) for reason in sorted(set(r.get('skip_reason') for r in scene_timing if r.get('skip_reason')))}},object_adapter={'events':len(adapter_timing),'max_bounds_sec':max((r.get('bounds_processing_sec') or 0 for r in adapter_timing),default=None),'max_tf_lookup_sec':max((r.get('tf_lookup_sec') or 0 for r in adapter_timing),default=None),'max_detected_to_publish_sec':max((r.get('detected_to_publish_sec') or 0 for r in adapter_timing),default=None),'max_header_age_at_publication_sec':max((r.get('header_age_at_publication_sec') or 0 for r in adapter_timing),default=None),'pending_events':sum(r.get('event')=='detected_objects_waiting_for_membership' for r in adapter_timing),'rejected_events':sum(r.get('event')=='adapter_input_rejected' for r in adapter_timing)},guard_timing={'ticks':len(guard_timing),'max_timer_gap_sec':max((r.get('tick_schedule_gap_sec') or 0 for r in guard_timing),default=None),'max_tick_work_sec':max((r.get('tick_work_sec') or 0 for r in guard_timing),default=None),'inputs':guard_inputs},stage_acquisition_age_max_sec={k:max((r[3] for r in rows[k]),default=None) for k in ['raw','extended','deskewed','compact','ground']},detector_acquisition_age_max_sec=max((r['age'] for r in rows['detected']),default=None),ndt_max_update_gap_sec=max((b[1]-a[1] for a,b in zip(rows['ndt'],rows['ndt'][1:])),default=None))
    for name,data in rows.items():(args.output/(name+'.json')).write_text(json.dumps(data))
    for name,data in [('graph',graph),('safety',safety),('base_safety',base_safety),('scene',scene),('events',events),('raw_samples',samples),('pipeline_diagnostics',pipeline_diagnostics)]: (args.output/(name+'.json')).write_text(json.dumps(data,indent=2))
    result=dict(passed=all(checks.values()),checks=checks,failed_checks=[k for k,v in checks.items() if not v],metrics=metrics)
    (args.output/'validation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
    node.destroy_node();rclpy.try_shutdown();return int(not result['passed'])
if __name__=='__main__':raise SystemExit(main())
