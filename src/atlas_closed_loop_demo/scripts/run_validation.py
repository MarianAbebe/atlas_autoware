#!/usr/bin/env python3
"""Measure a bounded actual Autoware-controller/simulator loop, including faults."""
import argparse, csv, hashlib, json, math, os, re, signal, subprocess, time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from autoware_internal_debug_msgs.msg import Float32MultiArrayStamped
from autoware_control_msgs.msg import Control
from autoware_planning_msgs.msg import Trajectory, LaneletRoute
from autoware_vehicle_msgs.msg import SteeringReport, GearReport, ControlModeReport, Engage
from nav_msgs.msg import Odometry, Path as RosPath
from geometry_msgs.msg import AccelWithCovarianceStamped
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from std_srvs.srv import Trigger
from composition_interfaces.srv import ListNodes, UnloadNode


def stamp(s):return s.sec+s.nanosec*1e-9
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def wrap(a):return math.atan2(math.sin(a),math.cos(a))
def project(x,y,poly):
    a=poly[:-1];d=poly[1:]-a
    u=np.clip(np.sum((np.array([x,y])-a)*d,axis=1)/np.sum(d*d,axis=1),0,1)
    closest=a+u[:,None]*d
    dist=np.linalg.norm(closest-[x,y],axis=1);i=int(np.argmin(dist))
    relative=np.array([x,y])-a[i]
    lateral=abs(d[i,0]*relative[1]-d[i,1]*relative[0])/np.linalg.norm(d[i])
    return float(lateral),math.atan2(d[i,1],d[i,0]),i,float(u[i]),float(dist[i])


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--fault',choices=['none','drop','freeze'],default='none')
    parser.add_argument('--gated',action='store_true',default=True);parser.add_argument('--ungated-baseline',dest='gated',action='store_false');parser.add_argument('--vehicle-model',choices=['IDEAL_STEER_ACC','IDEAL_STEER_ACC_GEARED'],default='IDEAL_STEER_ACC_GEARED');
    parser.add_argument('--results-root',type=Path,default=Path('/home/marian/roam/atlas_autoware/src/atlas_closed_loop_demo/results/current_runs'));parser.add_argument('--no-rviz',action='store_true',default=True);parser.add_argument('--rviz',dest='no_rviz',action='store_false');parser.add_argument('--timeout',type=float,default=180.)
    parser.add_argument('--fault-input',choices=['odometry','steering','command'],default='odometry')
    args=parser.parse_args()
    limits=json.loads((Path(__file__).resolve().parents[1]/'results/safety_validation/thresholds.json').read_text())
    run=args.results_root/(time.strftime('run_%Y%m%d_%H%M%S')+'_'+args.fault+'_'+args.fault_input+'_'+args.vehicle_model+('_gated' if args.gated else ''));run.mkdir(parents=True)
    (run/'thresholds.json').write_text(json.dumps(limits,indent=2))
    demo=Path(__file__).resolve().parents[1]
    (run/'source_sha256.json').write_text(json.dumps({str(p.relative_to(demo)):hashlib.sha256(p.read_bytes()).hexdigest() for directory in ['scripts','config','launch'] for p in (demo/directory).glob('*') if p.is_file()},indent=2))
    rclpy.init();node=rclpy.create_node('closed_loop_validation')
    latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    capturing=True
    latest={};odom=[];commands=[];gate_commands=[];feedback=[];raw_commands=[];steering_feedback=[];safety=[];trajectories=[];diagnostics=[];tf=[];subscribers=set();first={}
    start=time.monotonic();last_progress=0.;last_graph=0.;fault_at=None;stop_at=None;reason='timeout';started=False;fault_restore_at=None;fault_ros_stamp=None;near_stop_at=None;stop_ros_stamp=None
    def record(n,m):
        if not capturing:return
        latest[n]=m;first.setdefault(n,time.monotonic()-start)
        if n=='odom':
            p=m.pose.pose.position;odom.append([time.monotonic()-start,stamp(m.header.stamp),p.x,p.y,p.z,yaw(m.pose.pose.orientation),m.twist.twist.linear.x,m.twist.twist.angular.z])
        if n=='feedback':feedback.append([time.monotonic()-start,stamp(m.header.stamp),m.pose.pose.position.x,m.pose.pose.position.y])
        if n=='raw_command':raw_commands.append([time.monotonic()-start,stamp(m.stamp),m.lateral.steering_tire_angle,m.longitudinal.velocity,m.longitudinal.acceleration])
        if n=='steering_feedback':steering_feedback.append([time.monotonic()-start,stamp(m.stamp),m.steering_tire_angle])
        if n=='safety':safety.append(dict(json.loads(m.data),t=time.monotonic()-start))
        if n=='command':commands.append([time.monotonic()-start,stamp(m.stamp),m.lateral.steering_tire_angle,m.longitudinal.velocity,m.longitudinal.acceleration])
        if n=='gate_command':gate_commands.append([time.monotonic()-start,stamp(m.stamp),m.lateral.steering_tire_angle,m.longitudinal.velocity,m.longitudinal.acceleration])
        if n=='trajectory':
            trajectories.append({'t':time.monotonic()-start,'stamp':stamp(m.header.stamp),'points':len(m.points),'max_speed':max((p.longitudinal_velocity_mps for p in m.points),default=-1),'terminal_speed':m.points[-1].longitudinal_velocity_mps if m.points else None})
            if 'initial_trajectory' not in latest and len(m.points)>150:latest['initial_trajectory']=m
        if n=='diag':
            for s in m.status:
                if 'controller' in s.name or 'gate' in s.name:diagnostics.append({'t':time.monotonic()-start,'name':s.name,'level':s.level,'message':s.message,'values':{v.key:v.value for v in s.values}})
        if n=='debug':debug.append({'t':time.monotonic()-start,'state':int(m.data[13]),'current_speed':m.data[1]})
        if n=='tf':
            for t in m.transforms:
                if t.child_frame_id=='atlas_closed_loop_base_link':tf.append([time.monotonic()-start,stamp(t.header.stamp),t.header.frame_id,t.child_frame_id,t.transform.translation.x,t.transform.translation.y,yaw(t.transform.rotation)])
    topics={
      'odom':('/atlas_closed_loop/sim/output/odometry',Odometry,10),
      'feedback':('/atlas_closed_loop/feedback/odometry',Odometry,10),
      'steering':('/atlas_closed_loop/sim/output/steering',SteeringReport,10),
      'accel':('/atlas_closed_loop/sim/output/acceleration',AccelWithCovarianceStamped,10),
      'gear':('/atlas_closed_loop/sim/output/gear_report',GearReport,10),
      'mode':('/atlas_closed_loop/sim/output/control_mode_report',ControlModeReport,10),
      'command':('/atlas_closed_loop/control_cmd',Control,10),
      'raw_command':('/atlas_closed_loop/controller_raw/control_cmd',Control,10),
      'steering_feedback':('/atlas_closed_loop/feedback/steering',SteeringReport,10),
      'safety':('/atlas_closed_loop/safety_status',String,10),
      'trajectory':('/atlas_closed_loop/trajectory',Trajectory,10),
      'route':('/atlas_closed_loop/route',LaneletRoute,latched),
      'route_path':('/atlas_closed_loop/route_path',RosPath,latched),
      'pcd':('/atlas_map_demo/pointcloud_map',PointCloud2,latched),
      'roads':('/atlas_map_demo/road_markers',MarkerArray,latched),
      'markers':('/atlas_closed_loop/markers',MarkerArray,10),
      'vehicle_markers':('/atlas_closed_loop/vehicle_markers',MarkerArray,10),
      'driven_path':('/atlas_closed_loop/driven_path',RosPath,10),
      'traj_view':('/atlas_closed_loop/trajectory_visualization',RosPath,10),
      'debug':('/atlas_closed_loop/controller/output/longitudinal_diagnostic',Float32MultiArrayStamped,10),
      'diag':('/diagnostics',DiagnosticArray,10),'tf':('/tf',TFMessage,10)}
    if args.gated:topics['gate_command']=('/atlas_closed_loop/gated/control_cmd',Control,10)
    subs=[node.create_subscription(typ,topic,lambda m,n=n:record(n,m),qos) for n,(topic,typ,qos) in topics.items()]
    rearm=node.create_client(Trigger,'/atlas_closed_loop/feedback_watchdog/rearm');rearm_future=None
    inject=node.create_publisher(String,'/atlas_closed_loop/inject_feedback',10)
    shutdown_relay=node.create_publisher(String,'/atlas_closed_loop/test_shutdown',10)
    abort=node.create_publisher(Engage,'/atlas_closed_loop/sim/input/engage',10)
    originals=[Path('/home/marian/Downloads')/f for f in ['finalMap_repaired.osm','map_projector_info.yaml','pointcloud_map_georeferenced.pcd']]
    hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in originals}
    env=dict(os.environ,ROS_LOG_DIR=str(run/'ros_logs'),QT_QPA_PLATFORM='xcb',RCUTILS_COLORIZED_OUTPUT='0')
    nodes=set();alive=False;graph={};debug=[]
    with (run/'runtime.log').open('w') as log:
      process=subprocess.Popen(['ros2','launch','atlas_closed_loop_demo','closed_loop.launch.py',f'results_dir:={run}',f'launch_rviz:={str(not args.no_rviz).lower()}',f'gated:={str(args.gated).lower()}',f'vehicle_model:={args.vehicle_model}'],stdout=log,stderr=subprocess.STDOUT,env=env,start_new_session=True)
      try:
        while time.monotonic()-start<args.timeout and process.poll() is None:
            rclpy.spin_once(node,timeout_sec=.02)
            elapsed=time.monotonic()-start
            if odom and abs(odom[-1][6])>.05:started=True
            if elapsed-last_graph>=5:
                last_graph=elapsed
                nodes={(ns.rstrip('/')+'/'+n) for n,ns in node.get_node_names_and_namespaces()}
                for topic in ['/atlas_closed_loop/control_cmd','/atlas_closed_loop/feedback/odometry','/atlas_closed_loop/feedback/steering','/atlas_closed_loop/feedback/acceleration','/atlas_closed_loop/trajectory','/atlas_closed_loop/gated/control_cmd','/atlas_closed_loop/controller_raw/control_cmd']:
                    graph[topic]={'publishers':[i.node_namespace.rstrip('/')+'/'+i.node_name for i in node.get_publishers_info_by_topic(topic)],'subscribers':[i.node_namespace.rstrip('/')+'/'+i.node_name for i in node.get_subscriptions_info_by_topic(topic)]}
                if not args.no_rviz:
                    for name in ['pcd','roads','route_path','markers','vehicle_markers','driven_path','traj_view']:
                        if any(s.node_name=='closed_loop_rviz' for s in node.get_subscriptions_info_by_topic(topics[name][0])):subscribers.add(name)
            if 'route' not in latest or not odom:continue
            goal=latest['route'].goal_pose.position
            distance=math.hypot(odom[-1][2]-goal.x,odom[-1][3]-goal.y)
            if elapsed-last_progress>=10:
                last_progress=elapsed
                print(json.dumps({'elapsed':elapsed,'goal_distance':distance,'speed':odom[-1][6],'command':commands[-1] if commands else None,'pid_state':debug[-1] if debug else None}),flush=True)
            if args.fault!='none':
                if fault_at is None and started and elapsed>12 and odom[-1][6]>.5:
                    msg=String();msg.data=args.fault+':'+args.fault_input;inject.publish(msg);fault_at=elapsed;fault_ros_stamp=node.get_clock().now().nanoseconds*1e-9
                    print('FAULT',args.fault,'at',elapsed,'speed',odom[-1][6],flush=True)
                if fault_at and elapsed-fault_at>=2 and rearm_future is None and rearm.service_is_ready():
                    rearm_future=rearm.call_async(Trigger.Request())
                if fault_at and elapsed-fault_at>=8 and fault_restore_at is None:
                    msg=String();msg.data='live:'+args.fault_input;inject.publish(msg);fault_restore_at=elapsed
                    print('RESTORED valid inputs; explicit rearm withheld',flush=True)
                if fault_restore_at and elapsed-fault_restore_at>=limits['recovery_observation_sec']:
                    reason='fault_and_recovery_window_completed';break
            elif started and distance<limits['goal_distance_max_m'] and abs(odom[-1][6])<.001:
                if stop_at is None:stop_at=odom[-1][0];stop_ros_stamp=odom[-1][1]
                if odom[-1][0]-stop_at>=limits['terminal_hold_sec']+.05 and odom[-1][1]-stop_ros_stamp>=limits['terminal_hold_sec']+.05:reason='terminal_hold_completed';break
            else:stop_at=None
            if near_stop_at is None and started and distance<.75 and abs(odom[-1][6])<.03:near_stop_at=elapsed
            if started and distance>3 and 'route_path' in latest and np.linalg.norm([odom[-1][2]-latest['route'].start_pose.position.x,odom[-1][3]-latest['route'].start_pose.position.y])>70:
                reason='goal_overshoot_abort';break
            if odom and (abs(odom[-1][6])>1.6 or (started and odom[-1][6]<-.10)):
                reason='unsafe_speed_abort';break
            if elapsed>25 and not started:reason='no_motion_abort';break
        alive=process.poll() is None
      finally:
        measurement_ended=time.monotonic()
        log.write('\nATLAS_MEASUREMENT_COMPLETE\n');log.flush()
        capturing=False
        # Harness abort pauses integration; never counted as a controlled stop.
        engage=Engage();engage.stamp=node.get_clock().now().to_msg();engage.engage=False
        abort.publish(engage)
        shutdown_relay.publish(String(data='measurement_complete'))
        cleanup_start=time.monotonic()
        while time.monotonic()-cleanup_start<.3:rclpy.spin_once(node,timeout_sec=.02)
        # Release composable nodes through their supported lifecycle before DDS shutdown.
        # Recording is already closed; this cleanup cannot count as a stopped vehicle.
        for container in ['/atlas_closed_loop/planning_container','/atlas_closed_loop/sim/simulator_container','/atlas_map_demo/map_container']:
            client=node.create_client(ListNodes,container+'/_container/list_nodes')
            if client.service_is_ready():
                future=client.call_async(ListNodes.Request())
                rclpy.spin_until_future_complete(node,future,timeout_sec=1.)
                if future.done() and future.result() is not None:
                    unloader=node.create_client(UnloadNode,container+'/_container/unload_node')
                    for unique_id in future.result().unique_ids:
                        req=UnloadNode.Request();req.unique_id=unique_id
                        done=unloader.call_async(req)
                        rclpy.spin_until_future_complete(node,done,timeout_sec=1.)
                    node.destroy_client(unloader)
            node.destroy_client(client)
        if process.poll() is None:
            os.kill(process.pid,signal.SIGINT)
            try:process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
    cleanup_duration=time.monotonic()-measurement_ended
    checks={};metrics={'termination_reason':reason,'runtime_sec':time.monotonic()-start,'fault':args.fault,'fault_injected_at_sec':fault_at,'gated':args.gated,'vehicle_model':args.vehicle_model,'fault_input':args.fault_input,'fault_restored_at_sec':fault_restore_at}
    required={'route_adapter','path_generator','path_to_trajectory_converter','velocity_smoother','feedback_relay','controller','simple_planning_simulator'}
    if args.gated:required|={'vehicle_cmd_gate','feedback_watchdog'}
    checks['required_nodes_alive']=alive and required.issubset({n.rsplit('/',1)[-1] for n in nodes})
    checks['actual_map_publications']='pcd' in latest and latest['pcd'].width*latest['pcd'].height==1004344 and latest['pcd'].header.frame_id=='map' and 'roads' in latest
    checks['actual_route_two_lanelets']='route' in latest and [s.preferred_primitive.id for s in latest['route'].segments]==[38184,23892]
    checks['actual_smoother_outputs']=len(trajectories)>20 and max((t['max_speed'] for t in trajectories),default=99)<=1.0001 and all(t['terminal_speed']==0. for t in trajectories)
    checks['simulator_forward_drive_autonomous']='gear' in latest and latest['gear'].report==GearReport.DRIVE and latest['mode'].mode==ControlModeReport.AUTONOMOUS
    checks['map_base_frames']='odom' in latest and latest['odom'].header.frame_id=='map' and latest['odom'].child_frame_id=='atlas_closed_loop_base_link'
    if not args.no_rviz:checks['rviz_subscriptions']=subscribers==set(['pcd','roads','route_path','markers','vehicle_markers','driven_path','traj_view'])
    if args.fault!='none':checks['fault_experiment_exercised']=fault_at is not None and fault_restore_at is not None
    rows=np.array(odom)
    if len(rows)>10 and 'route_path' in latest:
        route_poly=np.array([[p.pose.position.x,p.pose.position.y] for p in latest['route_path'].poses])
        poly=np.array([[p.pose.position.x,p.pose.position.y] for p in latest['initial_trajectory'].points]) if 'initial_trajectory' in latest else route_poly
        # Remove coincident samples before projecting onto line segments.
        poly=poly[np.r_[True,np.linalg.norm(np.diff(poly,axis=0),axis=1)>1e-8]]
        metrics['route_length_m']=float(np.linalg.norm(np.diff(route_poly,axis=0),axis=1).sum())
        metrics['initial_trajectory_length_m']=float(np.linalg.norm(np.diff(poly,axis=0),axis=1).sum())
        metrics['initial_trajectory_points']=len(poly)
        checks['validated_route_length']=abs(metrics['route_length_m']-69.77868082882111)<.001
        checks['validated_initial_geometry']=abs(metrics['initial_trajectory_length_m']-limits['initial_trajectory_length_m'])<limits['initial_trajectory_length_tolerance_m'] and len(poly)==limits['initial_trajectory_points'] and np.linalg.norm(poly[-1]-route_poly[-1])<.001
        metrics['lateral_error_reference']='initial pinned smoothed trajectory, perpendicular segment distance'
        metrics['initial_recorded_start_gap_m']=float(np.linalg.norm(rows[0,2:4]-route_poly[0]))
        checks['initial_pose_recorded']=metrics['initial_recorded_start_gap_m']<.01
        errors=[project(row[2],row[3],poly) for row in rows]
        lat=np.array([e[0] for e in errors]);heading=np.array([abs(wrap(row[5]-e[1])) for row,e in zip(rows,errors)])
        ds=np.linalg.norm(np.diff(rows[:,2:4],axis=0),axis=1);dt=np.diff(rows[:,1]);dv=np.diff(rows[:,6]);acc=dv[dt>1e-6]/dt[dt>1e-6]
        goal=latest['route'].goal_pose.position
        metrics.update(total_distance_m=float(ds.sum()),final_distance_to_goal_m=math.hypot(rows[-1,2]-goal.x,rows[-1,3]-goal.y),max_lateral_error_m=float(lat.max()),rms_lateral_error_m=float(np.sqrt(np.mean(lat*lat))),max_heading_error_rad=float(heading.max()),max_actual_speed_mps=float(rows[:,6].max()),min_actual_speed_mps=float(rows[:,6].min()),max_actual_acceleration_mps2=float(acc.max()),max_actual_deceleration_mps2=float(acc.min()),final_speed_mps=float(rows[-1,6]),odometry_samples=len(rows),terminal_hold_sec=0 if stop_at is None else rows[-1,0]-stop_at)
        checks['measured_vehicle_motion']=metrics['total_distance_m']>1.
        checks['odometry_finite_advancing']=bool(np.isfinite(rows).all() and np.all(dt>0))
        checks['bounded_tracking']=metrics['max_lateral_error_m']<.3 and metrics['rms_lateral_error_m']<.1 and metrics['max_heading_error_rad']<.15
        checks['actual_speed_bound']=metrics['max_actual_speed_mps']<1.1 and metrics['min_actual_speed_mps']>-.02
        checks['actual_acceleration_bound']=metrics['max_actual_acceleration_mps2']<.55 and metrics['max_actual_deceleration_mps2']>-.55
        if args.fault=='none':
            checks['complete_route']=metrics['total_distance_m']>limits['total_travel_min_m'] and metrics['final_distance_to_goal_m']<limits['goal_distance_max_m']
            checks['strict_standstill']=abs(metrics['final_speed_mps'])<.001
            checks['verified_terminal_stop']=reason=='terminal_hold_completed' and abs(metrics['final_speed_mps'])<.001 and metrics['terminal_hold_sec']>=limits['terminal_hold_sec']-.1
            metrics['terminal_stopping_accuracy_m']=metrics['final_distance_to_goal_m']
            tail=rows[rows[:,0]>=rows[-1,0]-limits['terminal_hold_sec']]
            metrics['terminal_5s_max_abs_speed_mps']=float(np.max(np.abs(tail[:,6])))
            metrics['terminal_5s_displacement_m']=float(np.linalg.norm(np.diff(tail[:,2:4],axis=0),axis=1).sum())
            metrics['terminal_hold_max_speed_mps']=metrics['terminal_5s_max_abs_speed_mps']
            metrics['terminal_hold_creep_m']=metrics['terminal_5s_displacement_m']
            metrics['stop_settling_sec']=None if near_stop_at is None or stop_at is None else stop_at-near_stop_at
            checks['verified_terminal_stop'] &= metrics['terminal_hold_max_speed_mps']<.001 and metrics['terminal_hold_creep_m']<limits['terminal_creep_distance_max_m']
            hold_commands=[c for c in commands if stop_at is not None and c[0]>=stop_at]
            metrics['positive_acceleration_commands_during_hold']=sum(c[4]>0 for c in hold_commands)
            metrics['unexpected_motion_after_stop_m']=metrics['terminal_hold_creep_m']
            checks['no_terminal_command_oscillation']=bool(hold_commands) and all(c[4]<=0 and abs(c[3])<.001 for c in hold_commands)
            checks['stop_settling']=metrics['stop_settling_sec'] is not None and metrics['stop_settling_sec']<=limits['stop_settling_max_sec']
        elif fault_at:
            window=rows[rows[:,0]>=fault_at]
            delivered={'odometry':feedback,'steering':steering_feedback,'command':commands}[args.fault_input]
            fwindow=[f for f in delivered if fault_at+1<=f[0]<fault_at+7]
            metrics.update(fault_distance_travelled_m=float(np.linalg.norm(np.diff(window[:,2:4],axis=0),axis=1).sum()),fault_start_speed_mps=float(window[0,6]),fault_end_speed_mps=float(window[-1,6]),fault_feedback_samples=len(fwindow),fault_feedback_unique_stamps=len({f[1] for f in fwindow}))
            checks['feedback_fault_injected']=len(fwindow)==0 if args.fault=='drop' else len(fwindow)>10 and len({f[1] for f in fwindow})==1
            detected=[e for e in safety if e['latched'] and e['fault_stamp'] is not None and e['t']>=fault_at]
            metrics['fault_detection_latency_sec']=detected[0]['fault_stamp']-fault_ros_stamp if detected else None
            checks['fault_detection_latency']=bool(detected and 0<=metrics['fault_detection_latency_sec']<=limits['fault_detection_latency_max_sec'])
            checks['rearm_refused_with_invalid_inputs']=bool(rearm_future is not None and rearm_future.done() and not rearm_future.result().success)
            checks['specific_input_detected']=bool(detected and args.fault_input in detected[0]['fault_inputs'])
            stopped=np.flatnonzero(np.abs(window[:,6])<.001)
            settle=next((int(i) for i in stopped if np.max(np.abs(window[i:,6]))<.001),None)
            metrics['fault_stop_settling_sec']=None if settle is None else float(window[settle,0]-fault_at)
            tail=window[window[:,0]>=window[-1,0]-limits['fault_final_hold_sec']]
            metrics['fault_final_3s_max_abs_speed_mps']=float(np.max(np.abs(tail[:,6])))
            metrics['fault_final_3s_displacement_m']=float(np.linalg.norm(np.diff(tail[:,2:4],axis=0),axis=1).sum())
            metrics['unexpected_motion_after_stop_m']=None if settle is None else float(np.linalg.norm(np.diff(window[settle:,2:4],axis=0),axis=1).sum())
            checks['feedback_loss_safe_stop']=metrics['fault_final_3s_max_abs_speed_mps']<.001 and metrics['fault_distance_travelled_m']<limits['fault_stop_distance_max_m'] and metrics['fault_stop_settling_sec'] is not None and metrics['fault_stop_settling_sec']<=limits['fault_stop_settling_max_sec']
            restored=[e for e in safety if fault_restore_at and e['t']>fault_restore_at+1]
            checks['valid_inputs_restored']=bool(restored and all(not e['currently_stale'] for e in restored[-20:]))
            checks['no_automatic_rearm']=bool(restored and all(e['latched'] and not e['armed'] for e in restored))
            checks['no_unexpected_motion_after_stop']=metrics['unexpected_motion_after_stop_m'] is not None and metrics['unexpected_motion_after_stop_m']<.003
        if tf:
            matched=[]
            for t in tf:
                i=int(np.argmin(np.abs(rows[:,1]-t[1])))
                if abs(rows[i,1]-t[1])<.015:matched.append((t,rows[i]))
            metrics['tf_samples']=len(tf);metrics['tf_matched_odometry_samples']=len(matched)
            metrics['tf_max_xy_residual_m']=max((math.hypot(t[4]-r[2],t[5]-r[3]) for t,r in matched),default=99)
            metrics['tf_max_stamp_offset_sec']=max((abs(t[1]-r[1]) for t,r in matched),default=99)
            metrics['tf_max_yaw_residual_rad']=max((abs(wrap(t[6]-r[5])) for t,r in matched),default=99)
            checks['tf_matches_odometry']=len(matched)>100 and metrics['tf_max_xy_residual_m']<.03 and metrics['tf_max_yaw_residual_rad']<.01 and all(t[2]=='map' for t in tf)
    if gate_commands:
        metrics['gate_commands']=len(gate_commands)
        metrics['gate_command_min_acceleration_mps2']=min(c[4] for c in gate_commands)
        metrics['gate_command_hz']=(len(gate_commands)-1)/(gate_commands[-1][1]-gate_commands[0][1])
    if len(commands)>10:
        cmd=np.array(raw_commands if raw_commands else commands);freq=(len(cmd)-1)/(cmd[-1,1]-cmd[0,1]);metrics['controller_command_hz']=float(freq);metrics['controller_commands']=len(cmd)
        metrics['delivered_command_hz']=(len(commands)-1)/(commands[-1][0]-commands[0][0])
        metrics['max_command_steering_rad']=float(np.abs(cmd[:,2]).max())
        checks['controller_command_frequency']=25<freq<40
        checks['controller_commands_finite_steering_bound']=bool(np.isfinite(cmd).all() and np.abs(cmd[:,2]).max()<=.4501)
        checks['live_feedback_advances']=len(feedback)>100 and len({f[1] for f in feedback})>100
    text=(run/'runtime.log').read_text()
    runtime_text,_,shutdown_text=text.partition('ATLAS_MEASUREMENT_COMPLETE')
    shutdown_escalations=[line for line in shutdown_text.splitlines() if 'failed to terminate' in line and 'SIGTERM' in line]
    metrics['shutdown_sigterm_escalations']=len(shutdown_escalations)
    metrics['shutdown_duration_sec']=cleanup_duration
    checks['bounded_shutdown']=cleanup_duration<=30. and 'SIGKILL' not in shutdown_text
    checks['no_critical_shutdown_errors']=not any('[ERROR]' in line for line in shutdown_text.splitlines() if line not in shutdown_escalations)
    checks['no_controller_emergency']='-> EMERGENCY' not in text and not any('emergency' in d['message'].lower() for d in diagnostics if 'controller' in d['name'])
    checks['clean_launch_shutdown']=process.returncode==0
    filter_events=[line for line in text.splitlines() if '[VehicleCmdFilter]' in line and '[ERROR]' in line]
    metrics['gate_filter_limit_events']=len(filter_events)
    metrics['dds_transport_error_lines']=sum('TRANSPORT_UDP' in line or 'RTPS_PARTICIPANT' in line for line in text.splitlines())
    critical='\n'.join(line for line in runtime_text.splitlines() if '[VehicleCmdFilter]' not in line)
    checks['no_critical_runtime_errors']=not bool(re.search(r'\[ERROR\]|terminate called|Traceback',critical))
    if args.gated and args.fault!='none':checks['gate_emergency_activated']=any('vehicle_cmd_gate' in d['name'] and 'heartbeat is timeout' in d['message'] for d in diagnostics)
    checks['original_maps_unchanged']=all(hashlib.sha256(p.read_bytes()).hexdigest()==hashes[str(p)] for p in originals)
    # Graph evidence after shutdown would be unavailable; actual command QoS/source
    # is checked during runtime and persisted through discovery below.
    if args.gated:
        checks['controller_exclusive_command_source']=graph.get('/atlas_closed_loop/control_cmd',{}).get('publishers')==['/atlas_closed_loop/feedback_relay'] and '/atlas_closed_loop/gate/vehicle_cmd_gate' in graph.get('/atlas_closed_loop/control_cmd',{}).get('subscribers',[])
        checks['pinned_gate_exclusive_simulator_control']=graph.get('/atlas_closed_loop/gated/control_cmd',{}).get('publishers')==['/atlas_closed_loop/gate/vehicle_cmd_gate'] and '/atlas_closed_loop/sim/simple_planning_simulator' in graph.get('/atlas_closed_loop/gated/control_cmd',{}).get('subscribers',[])
    else:
        checks['controller_exclusive_command_source']=graph.get('/atlas_closed_loop/control_cmd',{}).get('publishers')==['/atlas_closed_loop/feedback_relay'] and '/atlas_closed_loop/sim/simple_planning_simulator' in graph.get('/atlas_closed_loop/control_cmd',{}).get('subscribers',[])
    checks['pinned_controller_command_origin']=graph.get('/atlas_closed_loop/controller_raw/control_cmd',{}).get('publishers')==['/atlas_closed_loop/controller']
    if raw_commands:
        raw_payloads={tuple(c[1:]) for c in raw_commands}
        checks['relay_never_generates_commands']=all(tuple(c[1:]) in raw_payloads for c in commands)
    checks['live_feedback_topology']=all('/atlas_closed_loop/controller' in graph.get(t,{}).get('subscribers',[]) for t in ['/atlas_closed_loop/feedback/odometry','/atlas_closed_loop/feedback/steering','/atlas_closed_loop/feedback/acceleration'])
    checks['no_controller_emergency']=checks['no_controller_emergency'] and not any(d['state']==3 for d in debug)
    metrics['controller_states_observed']=sorted({d['state'] for d in debug})
    checks['controller_debug_received']=len(debug)>100
    if len(rows)>10 and debug:
        diffs=[]
        for d in debug:
            if args.fault!='none' and fault_at and d['t']>=fault_at:continue
            i=int(np.argmin(np.abs(rows[:,0]-d['t'])))
            if abs(rows[i,0]-d['t'])<.04:diffs.append(abs(rows[i,6]-d['current_speed']))
        metrics['controller_vs_live_speed_rms_mps']=float(np.sqrt(np.mean(np.square(diffs)))) if diffs else None
        checks['controller_responds_to_live_speed']=len(diffs)>100 and metrics['controller_vs_live_speed_rms_mps']<.03
    if args.fault=='none':
        checks['controller_reached_stopped_state']=bool(debug and debug[-1]['state']==2)
        checks['no_nominal_safety_latch']=bool(safety) and all(not e['latched'] for e in safety) if args.gated else True
    report={'passed':all(checks.values()),'checks':{k:bool(v) for k,v in checks.items()},'failed_checks':[k for k,v in checks.items() if not v],'skipped_checks':['rendered_image_inspection','physical_vehicle_execution','PCD_localization_registration']+(['rviz_runtime'] if args.no_rviz else []),'metrics':metrics,'topic_first_receipt_sec':first,'rviz_subscriptions':sorted(subscribers),'nodes':sorted(nodes),'original_sha256':hashes,'results_dir':str(run),'graph':graph,'controller_debug':debug,'safety_events':safety,'thresholds':limits,'shutdown_warnings':shutdown_escalations,'diagnostics':diagnostics,'trajectories':trajectories}
    (run/'safety_events.json').write_text(json.dumps(safety,indent=2))
    (run/'validation.json').write_text(json.dumps(report,indent=2,default=lambda v: int.from_bytes(v,'little') if isinstance(v,bytes) else str(v)))
    for name,data,header in [('odometry',odom,['wall_sec','stamp','x','y','z','yaw','speed','yaw_rate']),('commands',commands,['wall_sec','stamp','steer','target_speed','acceleration']),('feedback',feedback,['wall_sec','stamp','x','y']),('raw_commands',raw_commands,['wall_sec','stamp','steer','target_speed','acceleration']),('steering_feedback',steering_feedback,['wall_sec','stamp','steer']),('gate_commands',gate_commands,['wall_sec','stamp','steer','target_speed','acceleration'])]:
        with (run/(name+'.csv')).open('w') as f:
            w=csv.writer(f);w.writerow(header);w.writerows(data)
    if 'initial_trajectory' in latest:
        with (run/'initial_trajectory.csv').open('w') as f:
            w=csv.writer(f);w.writerow(['x','y','z','yaw','velocity','acceleration'])
            for p in latest['initial_trajectory'].points:w.writerow([p.pose.position.x,p.pose.position.y,p.pose.position.z,yaw(p.pose.orientation),p.longitudinal_velocity_mps,p.acceleration_mps2])
    if 'route_path' in latest:
        with (run/'route.csv').open('w') as f:
            w=csv.writer(f);w.writerow(['x','y','z','yaw'])
            for p in latest['route_path'].poses:w.writerow([p.pose.position.x,p.pose.position.y,p.pose.position.z,yaw(p.pose.orientation)])
    print(json.dumps({'results_dir':str(run),'metrics':metrics,'failed_checks':report['failed_checks']},indent=2),flush=True)
    node.destroy_node();rclpy.shutdown()
    return 0 if report['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
