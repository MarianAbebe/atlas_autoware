#!/usr/bin/env python3
"""Read-only clock/QoS audit of an unengaged isolated localization launch."""
import argparse,json,os,signal,subprocess,time
from pathlib import Path
import rclpy
from rcl_interfaces.srv import GetParameters
from nav_msgs.msg import Odometry


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    rclpy.init();node=rclpy.create_node('timing_contract_audit')
    motion=[]
    sub=node.create_subscription(Odometry,'/atlas_loc/sim/output/odometry',lambda m:motion.append(m.twist.twist.linear.x),10)
    names=['/atlas_loc/sim/simple_planning_simulator','/atlas_loc/controller','/atlas_loc/gate/vehicle_cmd_gate',
           '/atlas_loc/ndt/ndt_scan_matcher','/atlas_loc/ekf/ekf_localizer','/atlas_loc/velocity_smoother',
           '/atlas_loc/localization_watchdog','/atlas_loc/synthetic_sensor_fixture']
    result=dict(nodes={},clock_publishers=[],endpoint_qos={})
    with (a.output/'launch.log').open('w') as log:
        proc=subprocess.Popen(['ros2','launch','atlas_localization_closed_loop_demo','localization_loop.launch.py'],
                              stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            for name in names:
                client=node.create_client(GetParameters,name+'/get_parameters')
                if not client.wait_for_service(timeout_sec=5):raise RuntimeError('parameter service missing: '+name)
                future=client.call_async(GetParameters.Request(names=['use_sim_time']))
                rclpy.spin_until_future_complete(node,future,timeout_sec=3)
                if not future.done():raise RuntimeError('parameter timeout: '+name)
                result['nodes'][name]={'use_sim_time':future.result().values[0].bool_value}
                node.destroy_client(client)
            result['clock_publishers']=[i.node_namespace+'/'+i.node_name for i in node.get_publishers_info_by_topic('/clock')]
            for topic in ['/atlas_loc/sim/output/odometry','/atlas_loc/ekf/ekf_odom',
                          '/atlas_loc/ndt/ndt_pose_with_covariance','/atlas_loc/sensors/points','/atlas_loc/control_cmd']:
                result['endpoint_qos'][topic]={}
                for kind,func in [('publishers',node.get_publishers_info_by_topic),('subscribers',node.get_subscriptions_info_by_topic)]:
                    result['endpoint_qos'][topic][kind]=[dict(node=i.node_namespace.rstrip('/')+'/'+i.node_name,
                        depth=i.qos_profile.depth,reliability=i.qos_profile.reliability.name,
                        durability=i.qos_profile.durability.name) for i in func(topic)]
            result['truth_samples']=len(motion)
            result['maximum_observed_speed']=max(map(abs,motion),default=None)
            result['passed']=all(not x['use_sim_time'] for x in result['nodes'].values()) and not result['clock_publishers'] and bool(motion) and not any(motion)
        finally:
            if proc.poll() is None:
                os.kill(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=12)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
            node.destroy_node();rclpy.try_shutdown()
    (a.output/'audit.json').write_text(json.dumps(result,indent=2));print(json.dumps(result['nodes'],indent=2))
    return 0 if result.get('passed') else 1


if __name__=='__main__':raise SystemExit(main())
