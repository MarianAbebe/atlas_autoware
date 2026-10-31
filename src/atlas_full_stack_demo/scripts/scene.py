#!/usr/bin/env python3
"""Campus-vector-coordinate synthetic scene and ideal sensors. No estimators/detections."""
from array import array
import math
import time
import copy
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
from rclpy.executors import ExternalShutdownException
import json


def angle(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def box(x,y):
    a,b,c=np.meshgrid(np.linspace(x-.4,x+.4,9),np.linspace(y-.3,y+.3,7),np.linspace(.2,1.,5))
    return np.column_stack([a.ravel(),b.ravel(),c.ravel()])


SCAN_DTYPE=np.dtype({'names':['x','y','z','intensity','tag','line','timestamp'],
    'formats':['<f4','<f4','<f4','<f4','u1','u1','<f8'],
    'offsets':[0,4,8,12,16,17,18],'itemsize':26})
SCAN_FIELDS=[PointField(name=k,offset=o,datatype=t,count=1) for k,o,t in
    [('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),('tag',16,2),('line',17,2),('timestamp',18,8)]]

TICK_TIMING_FIELDS=(
    'initial_pose_publication_sec','trajectory_publication_sec',
    'gear_publication_sec','auto_gear_publication_sec',
    'engagement_publication_sec','operation_mode_publication_sec',
    'odometry_freshness_evaluation_sec','lidar_generation_sec',
    'lidar_publication_sec','diagnostic_recording_sec',
    'diagnostic_publication_sec','callback_total_duration_sec')

ODOMETRY_TIMING_FIELDS=(
    'odometry_message_construction_sec','odometry_publication_sec',
    'odometry_callback_total_sec','odometry_source_age_at_receipt_sec')

LIDAR_SERIALIZATION_TIMING_FIELDS=(
    'pointcloud_field_assignment_sec','pointcloud_tobytes_sec',
    'pointcloud_array_construction_sec','pointcloud_data_assignment_sec',
    'pointcloud_serialization_cpu_sec','pointcloud_tobytes_cpu_sec',
    'pointcloud_array_construction_cpu_sec','pointcloud_data_assignment_cpu_sec',
    'lidar_publication_cpu_sec','thread_cpu_timing_supported')


def thread_cpu_time_sec():
    """Return current-thread CPU time, or None when the clock is unavailable."""
    try:
        value=time.thread_time()
        return value if math.isfinite(value) else None
    except (AttributeError,NotImplementedError,OSError):
        return None


def thread_cpu_delta_sec(start,end):
    """Return a same-thread CPU duration, without substituting another clock."""
    if end is None:return None
    if start is None:return None
    elapsed=end-start
    return elapsed if math.isfinite(elapsed) and elapsed>=0. else None


def thread_cpu_elapsed_sec(start):
    return thread_cpu_delta_sec(start,thread_cpu_time_sec())


def source_age_at_receipt_sec(msg,now_ns):
    """Return ROS-clock header age, or None for absent/invalid timestamps."""
    try:
        stamp=msg.header.stamp
        sec=int(stamp.sec);nanosec=int(stamp.nanosec)
        if sec<0 or nanosec<0 or nanosec>=1_000_000_000:return None
        stamp_ns=sec*1_000_000_000+nanosec
        if stamp_ns<=0:return None
        age=(int(now_ns)-stamp_ns)*1e-9
        return age if math.isfinite(age) else None
    except (AttributeError,TypeError,ValueError,OverflowError):
        return None


def transform_world_points(world,output,scratch,px,py,pz,cos_yaw,sin_yaw):
    """Fill a reusable buffer with the inverse planar base pose in lidar axes."""
    np.multiply(world[:,0],cos_yaw,out=output[:,0]);np.multiply(world[:,1],sin_yaw,out=scratch)
    np.add(output[:,0],scratch,out=output[:,0]);np.subtract(output[:,0],cos_yaw*px+sin_yaw*py,out=output[:,0])
    np.multiply(world[:,0],-sin_yaw,out=output[:,1]);np.multiply(world[:,1],cos_yaw,out=scratch)
    np.add(output[:,1],scratch,out=output[:,1]);np.subtract(output[:,1],-sin_yaw*px+cos_yaw*py,out=output[:,1])
    np.subtract(world[:,2],pz,out=output[:,2])
    return output


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
        self.timing=SceneTimingHistory();self.last_tick_monotonic=None;self.last_scan_publish_monotonic=None;self.accel_since_tick=0
        self.last_tick_finished_monotonic=None;self.odom_callback_count=0;self.odom_callback_max_sec=0.;self.odom_callback_gap_sec=None;self.last_odom_callback_monotonic=None
        self.odom_callback_timing_max={field:None for field in ODOMETRY_TIMING_FIELDS}
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
        callback_start=time.monotonic()
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
        self.scan_world=np.vstack([self.world,self.obstacle_world]) if self.obstacle else self.world
        self.scan_xyz=np.empty_like(self.scan_world);self.scan_scratch=np.empty(len(self.scan_world),dtype=np.float64)
        self.scan_points=np.zeros(len(self.scan_world),dtype=SCAN_DTYPE);self.scan_points['intensity']=40.
        self.scan_msg=PointCloud2();self.scan_msg.height=1;self.scan_msg.width=len(self.scan_world)
        self.scan_msg.point_step=26;self.scan_msg.row_step=26*len(self.scan_world);self.scan_msg.is_dense=True;self.scan_msg.fields=SCAN_FIELDS
        self.route_msg=Trajectory();self.route_msg.header.frame_id='map'
        for i,xyz in enumerate(self.path):
            p=TrajectoryPoint();p.pose.position.x,p.pose.position.y,p.pose.position.z=map(float,xyz)
            j=min(i+1,len(self.path)-1);k=max(i-1,0);a=math.atan2(self.path[j,1]-self.path[k,1],self.path[j,0]-self.path[k,0])
            p.pose.orientation.z=math.sin(a/2);p.pose.orientation.w=math.cos(a/2)
            p.longitudinal_velocity_mps=0. if i==len(self.path)-1 else 1.;self.route_msg.points.append(p)
        self.map_cloud=point_cloud2.create_cloud_xyz32(Header(frame_id='map'),self.world.tolist())
        self.map_view.publish(self.map_cloud)
        self.info.publish(String(data=json.dumps(dict(origin=self.path[0].tolist(),yaw=self.yaw,goal=self.path[-1].tolist(),obstacle_center=at.tolist(),obstacle=self.obstacle,scene='synthetic geometry in campus vector coordinates; original PCD unused',route_length_m=16.,world_points=len(self.world)))))
        self.record_timing(dict(event='path_initialization',world_point_count=len(self.world),scan_point_count=len(self.scan_world),
            path_point_count=len(self.path),path_initialization_sec=time.monotonic()-callback_start),callback_start)

    def on_truth(self,msg):
        start=time.monotonic()
        if self.last_odom_callback_monotonic is not None:self.odom_callback_gap_sec=start-self.last_odom_callback_monotonic
        self.last_odom_callback_monotonic=start
        source_age=source_age_at_receipt_sec(msg,self.get_clock().now().nanoseconds)
        self.truth=msg
        construction_start=time.monotonic()
        t=TwistWithCovarianceStamped();t.header=copy.deepcopy(msg.header);t.header.frame_id='base_link';t.twist=msg.twist
        t.twist.covariance[0]=.0001;t.twist.covariance[35]=.000025
        construction_sec=time.monotonic()-construction_start
        publication_start=time.monotonic();self.motion.publish(t)
        publication_sec=time.monotonic()-publication_start
        self.odom_callback_count+=1
        callback_total_sec=time.monotonic()-start
        self.odom_callback_max_sec=max(self.odom_callback_max_sec,callback_total_sec)
        measured=(construction_sec,publication_sec,callback_total_sec,source_age)
        for field,value in zip(ODOMETRY_TIMING_FIELDS,measured):
            if value is not None and math.isfinite(value):
                previous=self.odom_callback_timing_max[field]
                self.odom_callback_timing_max[field]=value if previous is None else max(previous,value)

    def on_accel(self,msg):
        msg.header.frame_id='base_link';self.accel.publish(msg);self.accel_since_tick+=1

    def dump_scene_timing(self,_,response):
        response.success=True;response.message=self.timing.json();return response

    def record_timing(self,record,start):
        record['callback_duration_sec']=time.monotonic()-start
        # Preserve the existing timer scheduling measurement boundary.
        if 'callback_monotonic_sec' in record:
            self.last_tick_finished_monotonic=time.monotonic()
        # The history copy is finalized after publishing. A live diagnostic
        # cannot include the duration of publishing itself, so those two
        # self-measurements are null in the streamed snapshot and complete in
        # the bounded history returned by ~/dump_scene_timing.
        record['diagnostic_recording_sec']=0.
        record['diagnostic_publication_sec']=None
        record['callback_total_duration_sec']=None
        recording_start=time.monotonic()
        self.timing.append(record)
        record['diagnostic_recording_sec']=time.monotonic()-recording_start
        if self.timing_pub.get_subscription_count():
            publication_start=time.monotonic()
            try:self.timing_pub.publish(String(data=json.dumps(record,separators=(',',':'))))
            except Exception:pass  # Diagnostics must never change scene execution.
            record['diagnostic_publication_sec']=time.monotonic()-publication_start
        record['callback_total_duration_sec']=time.monotonic()-start

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
        schedule_delay=None;callback_overrun=None
        if self.last_tick_monotonic is not None:
            expected=self.last_tick_monotonic+.1
            previous_finish=self.last_tick_finished_monotonic or self.last_tick_monotonic
            schedule_delay=max(0.,start-max(expected,previous_finish))
            callback_overrun=max(0.,previous_finish-expected)
        self.last_tick_monotonic=start
        accel_count=self.accel_since_tick;self.accel_since_tick=0
        odom_count=self.odom_callback_count;odom_max=self.odom_callback_max_sec;odom_gap=self.odom_callback_gap_sec
        self.odom_callback_count=0;self.odom_callback_max_sec=0.;self.odom_callback_gap_sec=None
        odom_timing_max=self.odom_callback_timing_max
        self.odom_callback_timing_max={field:None for field in ODOMETRY_TIMING_FIELDS}
        ros_now_ns=self.get_clock().now().nanoseconds
        truth_ns=None if self.truth is None else self.truth.header.stamp.sec*1000000000+self.truth.header.stamp.nanosec
        record=dict(callback_monotonic_sec=start,callback_gap_sec=gap,timer_schedule_delay_sec=schedule_delay,
            previous_callback_overrun_sec=callback_overrun,
            cached_odometry_age_sec=None if truth_ns is None else (ros_now_ns-truth_ns)*1e-9,
            odometry_callbacks_since_previous_timer=odom_count,odometry_callback_max_duration_sec=odom_max,
            odometry_callback_last_gap_sec=odom_gap,
            **odom_timing_max,
            scan_published=False,scan_skipped=False,skip_reason=None,acquisition_stamp=None,
            acceleration_publications_since_previous_timer=accel_count,
            acceleration_published_since_previous_timer=accel_count>0,trajectory_published=False,
            scan_publish_gap_sec=None,scan_generation_duration_sec=None,
            pointcloud_field_assignment_sec=0.,pointcloud_tobytes_sec=0.,
            pointcloud_array_construction_sec=0.,pointcloud_data_assignment_sec=0.,
            pointcloud_serialization_cpu_sec=None,pointcloud_tobytes_cpu_sec=None,
            pointcloud_array_construction_cpu_sec=None,pointcloud_data_assignment_cpu_sec=None,
            lidar_publication_cpu_sec=None,thread_cpu_timing_supported=None,
            **{field:0. for field in TICK_TIMING_FIELDS})
        if self.stopping:
            record['scan_skipped']=True;record['skip_reason']='stopping';self.record_timing(record,start);return
        if self.path is None:
            record['scan_skipped']=True;record['skip_reason']='path_unavailable';self.record_timing(record,start);return
        stamp=self.get_clock().now().to_msg()
        if not self.initial_sent and self.init.get_subscription_count():
            stage_start=time.monotonic()
            p=PoseWithCovarianceStamped();p.header=Header(frame_id='map',stamp=stamp)
            p.pose.pose.position.x,p.pose.pose.position.y,p.pose.pose.position.z=map(float,self.path[0]);p.pose.pose.orientation.z=math.sin(self.yaw/2);p.pose.pose.orientation.w=math.cos(self.yaw/2)
            self.init.publish(p);self.initial_sent=True
            record['initial_pose_publication_sec']=time.monotonic()-stage_start
        self.route_msg.header.stamp=stamp
        stage_start=time.monotonic();self.route.publish(self.route_msg)
        record['trajectory_publication_sec']=time.monotonic()-stage_start;record['trajectory_published']=True
        g=GearCommand(stamp=stamp,command=GearCommand.DRIVE)
        stage_start=time.monotonic();self.gear.publish(g)
        record['gear_publication_sec']=time.monotonic()-stage_start
        stage_start=time.monotonic();self.auto_gear.publish(g)
        record['auto_gear_publication_sec']=time.monotonic()-stage_start
        if self.request_engagement:
            stage_start=time.monotonic()
            self.engage.publish(Engage(stamp=stamp,engage=True))
            record['engagement_publication_sec']=time.monotonic()-stage_start
        m=OperationModeState();m.stamp=stamp;m.mode=m.AUTONOMOUS;m.is_autoware_control_enabled=True
        stage_start=time.monotonic();self.mode.publish(m)
        record['operation_mode_publication_sec']=time.monotonic()-stage_start
        if self.drop:
            record['scan_skipped']=True;record['skip_reason']='sensor_fault';self.record_timing(record,start);return
        if self.truth is None:
            record['scan_skipped']=True;record['skip_reason']='odometry_unavailable';self.record_timing(record,start);return
        h=self.truth.header.stamp;ns=h.sec*1000000000+h.nanosec
        freshness_start=time.monotonic()
        odometry_stale=self.get_clock().now().nanoseconds-ns>100000000
        record['odometry_freshness_evaluation_sec']=time.monotonic()-freshness_start
        if odometry_stale:
            record['scan_skipped']=True;record['skip_reason']='stale_odometry';self.record_timing(record,start);return
        scan_start=time.monotonic()
        point_start=scan_start;world=self.scan_world;point_duration=time.monotonic()-point_start
        p=self.truth.pose.pose.position;a=angle(self.truth.pose.pose.orientation);c,s=math.cos(a),math.sin(a)
        xyz=self.scan_xyz;tmp=self.scan_scratch
        transform_start=time.monotonic()
        transform_world_points(world,xyz,tmp,p.x,p.y,p.z,c,s)
        transform_duration=time.monotonic()-transform_start
        # FLOAT64 absolute ns requires 256 ns quantization at contemporary epoch.
        ns=ns//256*256
        serial_start=time.monotonic();serial_cpu_start=thread_cpu_time_sec();packed=self.scan_points
        field_assignment_start=time.monotonic()
        packed['x']=xyz[:,0];packed['y']=xyz[:,1];packed['z']=xyz[:,2];packed['timestamp']=float(ns)
        field_assignment_duration=time.monotonic()-field_assignment_start
        tobytes_start=time.monotonic();tobytes_cpu_start=thread_cpu_time_sec()
        packed_bytes=packed.tobytes()
        tobytes_cpu_end=thread_cpu_time_sec()
        tobytes_duration=time.monotonic()-tobytes_start
        tobytes_cpu_duration=thread_cpu_delta_sec(tobytes_cpu_start,tobytes_cpu_end)
        array_construction_start=time.monotonic();array_cpu_start=thread_cpu_time_sec()
        packed_data=array('B',packed_bytes)
        array_cpu_end=thread_cpu_time_sec()
        array_construction_duration=time.monotonic()-array_construction_start
        array_cpu_duration=thread_cpu_delta_sec(array_cpu_start,array_cpu_end)
        msg=self.scan_msg;msg.header.frame_id='synthetic_lidar';msg.header.stamp.sec=ns//1000000000;msg.header.stamp.nanosec=ns%1000000000
        data_assignment_start=time.monotonic();data_cpu_start=thread_cpu_time_sec()
        msg.data=packed_data
        data_cpu_end=thread_cpu_time_sec()
        serialization_end=time.monotonic()
        serialization_duration=serialization_end-serial_start
        data_assignment_duration=serialization_end-data_assignment_start
        generation_duration=time.monotonic()-scan_start
        data_cpu_duration=thread_cpu_delta_sec(data_cpu_start,data_cpu_end)
        serialization_cpu_duration=thread_cpu_delta_sec(serial_cpu_start,data_cpu_end)
        cpu_samples=(serial_cpu_start,tobytes_cpu_start,array_cpu_start,data_cpu_start,
            tobytes_cpu_duration,array_cpu_duration,data_cpu_duration,serialization_cpu_duration)
        record['pointcloud_field_assignment_sec']=field_assignment_duration
        record['pointcloud_tobytes_sec']=tobytes_duration
        record['pointcloud_array_construction_sec']=array_construction_duration
        record['pointcloud_data_assignment_sec']=data_assignment_duration
        record['pointcloud_serialization_cpu_sec']=serialization_cpu_duration
        record['pointcloud_tobytes_cpu_sec']=tobytes_cpu_duration
        record['pointcloud_array_construction_cpu_sec']=array_cpu_duration
        record['pointcloud_data_assignment_cpu_sec']=data_cpu_duration
        record['thread_cpu_timing_supported']=all(value is not None for value in cpu_samples)
        record['lidar_generation_sec']=generation_duration
        publication_start=time.monotonic();publication_cpu_start=thread_cpu_time_sec()
        self.raw.publish(msg)
        published=time.monotonic();record['lidar_publication_sec']=published-publication_start
        publication_cpu_duration=thread_cpu_elapsed_sec(publication_cpu_start)
        record['lidar_publication_cpu_sec']=publication_cpu_duration
        record['thread_cpu_timing_supported']=record['thread_cpu_timing_supported'] and publication_cpu_duration is not None
        publication_ros_ns=self.get_clock().now().nanoseconds
        record['scan_published']=True;record['acquisition_stamp']=[msg.header.stamp.sec,msg.header.stamp.nanosec]
        record['header_age_at_publication_sec']=(publication_ros_ns-ns)*1e-9
        record['point_generation_sec']=point_duration
        record['coordinate_transform_sec']=transform_duration
        record['pointcloud_serialization_sec']=serialization_duration
        record['scan_generation_duration_sec']=published-scan_start
        record['scan_publish_gap_sec']=None if self.last_scan_publish_monotonic is None else published-self.last_scan_publish_monotonic
        self.last_scan_publish_monotonic=published
        self.record_timing(record,start)


def main():
    rclpy.init();n=Scene()
    try:
        while rclpy.ok() and not n.stopping:rclpy.spin_once(n,timeout_sec=.02)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
