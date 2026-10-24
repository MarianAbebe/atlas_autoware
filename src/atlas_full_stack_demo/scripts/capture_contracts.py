#!/usr/bin/env python3
"""Read-only contract/parameter capture; run alongside the full-stack launch."""
import argparse,json,time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from rcl_interfaces.srv import GetParameters


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    rclpy.init();n=rclpy.create_node('full_stack_contract_inspector');stages={k:{} for k in ['raw','extended','deskewed','compact','ground','clusters']}
    def receive(k,m):
        key=(m.header.stamp.sec,m.header.stamp.nanosec)
        stages[k][key]=dict(stamp=list(key),frame=m.header.frame_id,points=m.width*m.height,height=m.height,width=m.width,point_step=m.point_step,row_step=m.row_step,bigendian=m.is_bigendian,fields=[dict(name=f.name,offset=f.offset,datatype=f.datatype,count=f.count) for f in m.fields])
        if len(stages[k])>20:stages[k].pop(next(iter(stages[k])))
    subs=[n.create_subscription(PointCloud2,'/atlas_loc/'+topic,lambda m,k=k:receive(k,m),qos_profile_sensor_data) for k,topic in [('raw','lidar/raw'),('extended','lidar/extended'),('deskewed','lidar/deskewed'),('compact','lidar/compact'),('ground','obstacles/pointcloud'),('clusters','clusters')]]
    end=time.monotonic()+15;common=set()
    while time.monotonic()<end:
        rclpy.spin_once(n,timeout_sec=.02);common=set.intersection(*(set(v) for v in stages.values()))
        if common:break
    result=dict(matched_acquisition=bool(common))
    if common:
        key=min(common);result['cloud_formats']={k:v[key] for k,v in stages.items()}
    requests={'planner':['obstacle_stop.obstacle_filtering.check_inside.pointcloud'],'localization_watchdog':['freshness_sec'],'obstacle_guard':['freshness_sec'],'controller':['use_sim_time']}
    nodes={'planner':'motion_velocity_planner'};result['parameters']={}
    for name,names in requests.items():
        c=n.create_client(GetParameters,'/atlas_loc/'+nodes.get(name,name)+'/get_parameters')
        if not c.wait_for_service(timeout_sec=3):raise RuntimeError('Missing parameter service: '+name)
        future=c.call_async(GetParameters.Request(names=names));rclpy.spin_until_future_complete(n,future,timeout_sec=3)
        if not future.done():raise RuntimeError('Parameter callback deadline: '+name)
        result['parameters'][name]={key:dict(type=v.type,bool_value=v.bool_value,double_value=v.double_value) for key,v in zip(names,future.result().values)}
    result['map_service_servers']=[]
    for name,ns in n.get_node_names_and_namespaces():
        if any(s=='/pcd_loader_service' for s,_ in n.get_service_names_and_types_by_node(name,ns)):result['map_service_servers'].append(ns.rstrip('/')+'/'+name)
    p=result['parameters'];result['passed']=bool(common) and not p['planner']['obstacle_stop.obstacle_filtering.check_inside.pointcloud']['bool_value'] and p['localization_watchdog']['freshness_sec']['double_value']==.3 and p['obstacle_guard']['freshness_sec']['double_value']==.3 and not p['controller']['use_sim_time']['bool_value']
    a.output.write_text(json.dumps(result,indent=2));print('Read-only contract capture passed:',result['passed']);n.destroy_node();rclpy.try_shutdown()
    return int(not result['passed'])
if __name__=='__main__':raise SystemExit(main())
