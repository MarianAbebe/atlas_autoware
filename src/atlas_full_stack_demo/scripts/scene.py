#!/usr/bin/env python3
"""Campus-vector-coordinate synthetic scene and ideal sensors. No estimators/detections."""
import math
import time
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped, AccelWithCovarianceStamped, TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from autoware_vehicle_msgs.msg import GearCommand, Engage
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_map_msgs.srv import GetDifferentialPointCloudMap
from autoware_map_msgs.msg import PointCloudMapCellWithID
from tf2_ros import StaticTransformBroadcaster
import json


def angle(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def box(x,y):
    a,b,c=np.meshgrid(np.linspace(x-.4,x+.4,9),np.linspace(y-.3,y+.3,7),np.linspace(.2,1.,5))
    return np.column_stack([a.ravel(),b.ravel(),c.ravel()])


class SceneTimingHistory:
    """Bounded callback history; diagnostics must not affect sensor behavior."""
    def __init__(self, limit=256):
        self.records=deque(maxlen=limit)

    def append(self, record):
        self.records.append(record)

    def json(self):
        return json.dumps(list(self.records),separators=(',',':'))


class Scene(Node):
    def __init__(self):
        super().__init__('full_stack_scene',namespace='/atlas_loc')
        self.obstacle=self.declare_parameter('obstacle',True).value
        self.request_engagement=self.declare_parameter('request_engagement',True).value
        self.path=None;self.truth=None;self.initial_sent=False;self.drop=False;self.stopping=False
        self.timing=SceneTimingHistory();self.last_tick_monotonic=None;self.accel_since_tick=0
        self.raw=self.create_publisher(PointCloud2,'lidar/raw',qos_profile_sensor_data)
        self.motion=self.create_publisher(TwistWithCovarianceStamped,'sensors/twist',10)
        self.accel=self.create_publisher(AccelWithCovarianceStamped,'sensors/acceleration',10)
        self.route=self.create_publisher(Trajectory,'raw_trajectory',10)
        self.init=self.create_publisher(PoseWithCovarianceStamped,'sim/input/initialpose',10)
        self.gear=self.create_publisher(GearCommand,'sim/input/gear_command',10)
        self.auto_gear=self.create_publisher(GearCommand,'auto_gear',10)
        self.engage=self.create_publisher(Engage,'gate/input/engage',10)
        self.mode=self.create_publisher(OperationModeState,'operation_mode',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.info=self.create_publisher(String,'scene_info',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.map_view=self.create_publisher(PointCloud2,'synthetic_map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.timing_pub=self.create_publisher(String,'diagnostics/scene_timing',10)
        self.create_subscription(Path,'route_path',self.on_path,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry,'sim/output/odometry',self.on_truth,10)
        self.create_subscription(AccelWithCovarianceStamped,'sim/output/acceleration',self.on_accel,10)
        self.create_subscription(String,'sensor_fault',lambda m:setattr(self,'drop',m.data=='drop_ndt'),10)
        self.create_subscription(String,'test_shutdown',lambda m:setattr(self,'stopping',True),10)
        self.create_service(GetDifferentialPointCloudMap,'/pcd_loader_service',self.map_request)
        self.create_service(Trigger,'~/dump_scene_timing',self.dump_scene_timing)
        self.tf=StaticTransformBroadcaster(self)
        t=TransformStamped();t.header.frame_id='base_link';t.child_frame_id='synthetic_lidar';t.transform.rotation.w=1.;self.tf.sendTransform(t)
        self.create_timer(.1,self.tick)

    def on_path(self,msg):
        if self.path is not None:return
        pts=np.array([[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in msg.poses])
        dist=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(pts[:,:2],axis=0),axis=1))]
        if dist[-1]<16.:raise RuntimeError('Campus route too short')
        self.s=np.arange(0,16.001,.2)
        self.path=np.column_stack([np.interp(self.s,dist,pts[:,i]) for i in range(3)])
        # Explicit flat virtual terrain at the route start, not supplied PCD registration.
        self.path[:,2]=pts[0,2]
        self.yaw=math.atan2(self.path[1,1]-self.path[0,1],self.path[1,0]-self.path[0,0])
        c,s=math.cos(self.yaw),math.sin(self.yaw);self.rot=np.array([[c,-s],[s,c]])
        gx,gy=np.meshgrid(np.arange(-5,26.01,.5),np.arange(-8,8.01,.5))
        terrain=np.column_stack([gx.ravel(),gy.ravel(),np.zeros(gx.size)])
        landmarks=np.vstack([box(x,y) for x,y in [(1,-5),(5,5),(12,-6),(19,5),(24,-5)]])
        local=np.vstack([terrain,landmarks]);self.world=local.copy();self.world[:,:2]=local[:,:2]@self.rot.T+self.path[0,:2];self.world[:,2]+=self.path[0,2]
        at=self.path[50];self.obstacle_world=box(0,0);self.obstacle_world[:,:2]=self.obstacle_world[:,:2]@self.rot.T+at[:2];self.obstacle_world[:,2]+=at[2]
        self.map_cloud=point_cloud2.create_cloud_xyz32(Header(frame_id='map'),self.world.tolist())
        self.map_view.publish(self.map_cloud)
        self.info.publish(String(data=json.dumps(dict(origin=self.path[0].tolist(),yaw=self.yaw,goal=self.path[-1].tolist(),obstacle_center=at.tolist(),obstacle=self.obstacle,scene='synthetic geometry in campus vector coordinates; original PCD unused',route_length_m=16.,world_points=len(self.world)))))

    def on_truth(self,msg):
        self.truth=msg
        t=TwistWithCovarianceStamped();t.header=msg.header;t.header.frame_id='base_link';t.twist=msg.twist
        t.twist.covariance[0]=.0001;t.twist.covariance[35]=.000025;self.motion.publish(t)

    def on_accel(self,msg):
        msg.header.frame_id='base_link';self.accel.publish(msg);self.accel_since_tick+=1

    def dump_scene_timing(self,_,response):
        response.success=True;response.message=self.timing.json();return response

    def record_timing(self,record,start):
        record['callback_duration_sec']=time.monotonic()-start
        self.timing.append(record)
        if self.timing_pub.get_subscription_count():
            try:self.timing_pub.publish(String(data=json.dumps(record,separators=(',',':'))))
            except Exception:pass  # Diagnostics must never change scene execution.

    def map_request(self,req,res):
        res.header.frame_id='map'
        if self.path is None:return res
        cell=PointCloudMapCellWithID();cell.cell_id='campus_synthetic_0';cell.pointcloud=self.map_cloud
        cell.metadata.min_x=float(self.world[:,0].min());cell.metadata.max_x=float(self.world[:,0].max())
        cell.metadata.min_y=float(self.world[:,1].min());cell.metadata.max_y=float(self.world[:,1].max())
        if cell.cell_id not in req.cached_ids:res.new_pointcloud_with_ids=[cell]
        return res

    def tick(self):
        start=time.monotonic();gap=None if self.last_tick_monotonic is None else start-self.last_tick_monotonic
        self.last_tick_monotonic=start
        accel_count=self.accel_since_tick;self.accel_since_tick=0
        ros_now_ns=self.get_clock().now().nanoseconds
        truth_ns=None if self.truth is None else self.truth.header.stamp.sec*1000000000+self.truth.header.stamp.nanosec
        record=dict(callback_monotonic_sec=start,callback_gap_sec=gap,
            cached_odometry_age_sec=None if truth_ns is None else (ros_now_ns-truth_ns)*1e-9,
            scan_published=False,scan_skipped=False,skip_reason=None,acquisition_stamp=None,
            acceleration_publications_since_previous_timer=accel_count,
            acceleration_published_since_previous_timer=accel_count>0,trajectory_published=False)
        if self.stopping:
            record['scan_skipped']=True;record['skip_reason']='stopping';self.record_timing(record,start);return
        if self.path is None:
            record['scan_skipped']=True;record['skip_reason']='path_unavailable';self.record_timing(record,start);return
        stamp=self.get_clock().now().to_msg()
        if not self.initial_sent and self.init.get_subscription_count():
            p=PoseWithCovarianceStamped();p.header=Header(frame_id='map',stamp=stamp)
            p.pose.pose.position.x,p.pose.pose.position.y,p.pose.pose.position.z=map(float,self.path[0]);p.pose.pose.orientation.z=math.sin(self.yaw/2);p.pose.pose.orientation.w=math.cos(self.yaw/2)
            self.init.publish(p);self.initial_sent=True
        path=Trajectory();path.header=Header(frame_id='map',stamp=stamp)
        # Test reference comes from actual loaded campus centerline; pinned planner/smoother profiles it.
        for i,xyz in enumerate(self.path):
            p=TrajectoryPoint();p.pose.position.x,p.pose.position.y,p.pose.position.z=map(float,xyz)
            j=min(i+1,len(self.path)-1);k=max(i-1,0);a=math.atan2(self.path[j,1]-self.path[k,1],self.path[j,0]-self.path[k,0]);p.pose.orientation.z=math.sin(a/2);p.pose.orientation.w=math.cos(a/2)
            p.longitudinal_velocity_mps=0. if i==len(self.path)-1 else 1.;path.points.append(p)
        self.route.publish(path);record['trajectory_published']=True
        g=GearCommand(stamp=stamp,command=GearCommand.DRIVE);self.gear.publish(g);self.auto_gear.publish(g)
        if self.request_engagement:
            self.engage.publish(Engage(stamp=stamp,engage=True))
        m=OperationModeState();m.stamp=stamp;m.mode=m.AUTONOMOUS;m.is_autoware_control_enabled=True;self.mode.publish(m)
        if self.drop:
            record['scan_skipped']=True;record['skip_reason']='sensor_fault';self.record_timing(record,start);return
        if self.truth is None:
            record['scan_skipped']=True;record['skip_reason']='odometry_unavailable';self.record_timing(record,start);return
        h=self.truth.header.stamp;ns=h.sec*1000000000+h.nanosec
        if self.get_clock().now().nanoseconds-ns>100000000:
            record['scan_skipped']=True;record['skip_reason']='stale_odometry';self.record_timing(record,start);return
        world=np.vstack([self.world,self.obstacle_world]) if self.obstacle else self.world
        p=self.truth.pose.pose.position;a=angle(self.truth.pose.pose.orientation);c,s=math.cos(a),math.sin(a)
        d=world-np.array([p.x,p.y,p.z]);xyz=d.copy();xyz[:,0]=c*d[:,0]+s*d[:,1];xyz[:,1]=-s*d[:,0]+c*d[:,1]
        # FLOAT64 absolute ns requires 256 ns quantization at contemporary epoch.
        ns=ns//256*256
        dtype=np.dtype({'names':['x','y','z','intensity','tag','line','timestamp'],'formats':['<f4','<f4','<f4','<f4','u1','u1','<f8'],'offsets':[0,4,8,12,16,17,18],'itemsize':26})
        packed=np.zeros(len(xyz),dtype=dtype)
        for i,k in enumerate(['x','y','z']):packed[k]=xyz[:,i]
        packed['intensity']=40.;packed['timestamp']=float(ns)
        msg=PointCloud2();msg.header=Header(frame_id='synthetic_lidar');msg.header.stamp.sec=ns//1000000000;msg.header.stamp.nanosec=ns%1000000000
        msg.height=1;msg.width=len(xyz);msg.point_step=26;msg.row_step=26*len(xyz);msg.is_dense=True
        msg.fields=[PointField(name=k,offset=o,datatype=t,count=1) for k,o,t in [('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),('tag',16,2),('line',17,2),('timestamp',18,8)]]
        msg.data=packed.tobytes();self.raw.publish(msg)
        record['scan_published']=True;record['acquisition_stamp']=[msg.header.stamp.sec,msg.header.stamp.nanosec]
        self.record_timing(record,start)


def main():
    rclpy.init();n=Scene()
    try:
        while rclpy.ok() and not n.stopping:rclpy.spin_once(n,timeout_sec=.02)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
