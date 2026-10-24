#!/usr/bin/env python3
"""One bounded operator start: activate pinned estimators, seed once, request engagement.
Run after full_stack.launch.py. Does not publish odometry, objects or controls.
"""
import json,math,time
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool
from autoware_vehicle_msgs.msg import Engage

def main():
    rclpy.init();n=rclpy.create_node('full_stack_operator_start');scene={};status={};seen=[];motion=[]
    n.create_subscription(String,'/atlas_loc/scene_info',lambda m:scene.update(json.loads(m.data)),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    n.create_subscription(String,'/atlas_loc/safety_status',lambda m:status.update(json.loads(m.data)),10)
    n.create_subscription(PoseWithCovarianceStamped,'/atlas_loc/ndt/ndt_pose_with_covariance',lambda m:seen.append(m.header.stamp),10)
    n.create_subscription(Odometry,'/atlas_loc/sim/output/odometry',lambda m:motion.append(m.twist.twist.linear.x),10)
    boot=n.create_publisher(PoseWithCovarianceStamped,'/atlas_loc/bootstrap',10);engage=n.create_publisher(Engage,'/atlas_loc/sim/input/engage',10)
    def wait(pred,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.02)
            if pred():return True
        return False
    try:
        if not wait(lambda:bool(scene) and bool(motion),15):raise RuntimeError('No scene/truth: inspect launch log')
        if abs(motion[-1])>.001:raise RuntimeError('Refusing bootstrap while moving')
        if status.get('latched'):raise RuntimeError('Latched startup: preserve evidence and restart the launch; no auto-rearm')
        for name in ['ekf','ndt']:
            c=n.create_client(SetBool,'/atlas_loc/'+name+'/trigger_node_srv')
            if not c.wait_for_service(timeout_sec=5):raise RuntimeError(name+' service missing')
            f=c.call_async(SetBool.Request(data=True))
            if not wait(f.done,5) or not f.result().success:raise RuntimeError(name+' activation failed')
        p=PoseWithCovarianceStamped();p.header.frame_id='map';p.header.stamp=n.get_clock().now().to_msg();p.pose.pose.position.x=scene['origin'][0]+.1;p.pose.pose.position.y=scene['origin'][1]+.1;p.pose.pose.position.z=scene['origin'][2];p.pose.pose.orientation.z=math.sin(scene['yaw']/2);p.pose.pose.orientation.w=math.cos(scene['yaw']/2)
        for k,v in [(0,.25),(7,.25),(14,.0025),(21,.0006),(28,.0006),(35,.06)]:p.pose.covariance[k]=v
        boot.publish(p)
        if not wait(lambda:status.get('armed') and len(seen)>=10,20):raise RuntimeError('Localization/perception authorization unavailable: '+json.dumps(status))
        engage.publish(Engage(stamp=n.get_clock().now().to_msg(),engage=True))
        if not wait(lambda:bool(motion) and motion[-1]>.02,8):raise RuntimeError('Engagement requested but no verified motion: inspect safety and gate')
        print('Engaged with NDT/EKF and perception authorization. Launch remains running; inspect topics and RViz.')
    finally:n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
