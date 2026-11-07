#!/usr/bin/env python3
"""Campus-vector-coordinate synthetic scene and ideal sensors. No estimators/detections."""
from array import array
import math
import time
import copy
import os
import resource
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped, AccelWithCovarianceStamped, TransformStamped
from sensor_msgs.msg import Imu, PointCloud2, PointField
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
    'pointcloud_field_assignment_sec','pointcloud_field_assignment_cpu_sec','pointcloud_tobytes_sec',
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


def packed_xyz_view(packed):
    """Return a strided Nx3 float32 view over the PointCloud2 XYZ fields.

    SCAN_DTYPE stores x/y/z at byte offsets 0/4/8 in each 26-byte record.
    The view writes the same float32 values as separate structured-field
    assignments while touching each record's XYZ fields in one vectorized pass.
    """
    if packed.dtype != SCAN_DTYPE or packed.ndim != 1:
        raise ValueError('packed XYZ view requires a one-dimensional SCAN_DTYPE array')
    return np.ndarray((len(packed),3),dtype=np.float32,buffer=packed,
                      strides=(SCAN_DTYPE.itemsize,np.dtype(np.float32).itemsize))


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
        self.route_length_requested=float(self.declare_parameter('route_length_m',16.0).value)
        self.trajectory_repeats=int(self.declare_parameter('trajectory_repeats',1).value)
        self.loop_transition_max_m=float(self.declare_parameter('loop_transition_max_m',6.5).value)
        self.include_terminal_extension=bool(self.declare_parameter('include_terminal_extension',False).value)
        self.map_cloud_topic=str(self.declare_parameter('map_cloud_topic','').value)
        self.lidar_max_range_m=float(self.declare_parameter('lidar_max_range_m',0.0).value)
        if not math.isfinite(self.route_length_requested) or self.route_length_requested==0.0:
            raise ValueError('route_length_m must be positive or -1 for the full route')
        if (self.trajectory_repeats<1 or not math.isfinite(self.lidar_max_range_m) or self.lidar_max_range_m<0.0
                or not math.isfinite(self.loop_transition_max_m) or self.loop_transition_max_m<0.0):
            raise ValueError('trajectory_repeats must be positive; LiDAR range and loop transition must be nonnegative')
        self.path=None;self.lap_path=None;self.pending_path=None;self.external_map_world=None;self.external_map_cloud=None
        self.truth=None;self.initial_sent=False;self.drop=False;self.stopping=False
        self.imu_previous_stamp_ns=None;self.imu_previous_velocity=None
        self.timing=SceneTimingHistory();self.last_tick_monotonic=None;self.last_scan_publish_monotonic=None;self.accel_since_tick=0
        self.last_accel_header_stamp_ns=None;self.last_accel_receipt_monotonic=None;self.last_accel_header_age_ros_sec=None
        self.last_tick_finished_monotonic=None;self.odom_callback_count=0;self.odom_callback_max_sec=0.;self.odom_callback_gap_sec=None;self.last_odom_callback_monotonic=None
        self.odom_callback_timing_max={field:None for field in ODOMETRY_TIMING_FIELDS}
        self.process_pid=os.getpid();self.process_start_monotonic=time.monotonic()
        self.process_start_ros_time_ns=self.get_clock().now().nanoseconds
        self.scene_tick_index=0;self.last_odom_receipt_monotonic=None;self.last_odom_header_stamp_ns=None
        self.last_scan_header_stamp_ns=None;self.process_resource_sample_monotonic=None
        self.process_resource_sample={}
        self.sample_process_resources(self.process_start_monotonic,force=True)
        self.raw=self.create_publisher(PointCloud2,'lidar/raw',qos_profile_sensor_data)
        self.motion=self.create_publisher(TwistWithCovarianceStamped,'sensors/twist',10)
        self.imu=self.create_publisher(Imu,'sensors/imu/data',qos_profile_sensor_data)
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
        if self.map_cloud_topic:
            map_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(PointCloud2,self.map_cloud_topic,self.on_map_cloud,map_qos)
        self.create_subscription(Odometry,'sim/output/odometry',self.on_truth,
            QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(AccelWithCovarianceStamped,'sim/output/acceleration',self.on_accel,10)
        self.create_subscription(String,'sensor_fault',lambda m:setattr(self,'drop',m.data=='drop_ndt'),10)
        self.create_subscription(String,'test_shutdown',lambda m:setattr(self,'stopping',True),10)
        self.create_service(GetDifferentialPointCloudMap,'/pcd_loader_service',self.map_request)
        self.create_service(Trigger,'~/dump_scene_timing',self.dump_scene_timing)
        self.tf=StaticTransformBroadcaster(self)
        lidar_tf=TransformStamped();lidar_tf.header.frame_id='base_link';lidar_tf.child_frame_id='synthetic_lidar';lidar_tf.transform.rotation.w=1.
        imu_tf=TransformStamped();imu_tf.header.frame_id='base_link';imu_tf.child_frame_id='imu_link';imu_tf.transform.rotation.w=1.
        self.tf.sendTransform([lidar_tf,imu_tf])
        self.create_timer(.1,self.tick)
        self.log_lifecycle('process_start')

    def log_lifecycle(self,event,**fields):
        row=dict(event=event,pid=self.process_pid,monotonic_sec=time.monotonic(),
            ros_time_ns=self.get_clock().now().nanoseconds,**fields)
        self.get_logger().info('scene_lifecycle '+json.dumps(row,separators=(',',':')))

    def sample_process_resources(self,now_monotonic,force=False):
        if (not force and self.process_resource_sample_monotonic is not None
                and now_monotonic-self.process_resource_sample_monotonic<1.0):
            return
        previous=self.process_resource_sample
        try:
            usage=resource.getrusage(resource.RUSAGE_SELF)
            cpu_sec=time.process_time()
            interval=None if self.process_resource_sample_monotonic is None else now_monotonic-self.process_resource_sample_monotonic
            cpu_fraction=None
            if interval and interval>0 and previous.get('process_cpu_time_sec') is not None:
                cpu_fraction=max(0.0,(cpu_sec-previous['process_cpu_time_sec'])/interval)
            self.process_resource_sample=dict(process_cpu_time_sec=cpu_sec,
                process_cpu_fraction_since_sample=cpu_fraction,process_max_rss_kib=int(usage.ru_maxrss),
                system_load_1m=float(os.getloadavg()[0]),
                system_available_memory_bytes=int(os.sysconf('SC_AVPHYS_PAGES')*os.sysconf('SC_PAGE_SIZE')))
        except (OSError,ValueError,AttributeError):
            self.process_resource_sample={key:None for key in ('process_cpu_time_sec',
                'process_cpu_fraction_since_sample','process_max_rss_kib','system_load_1m',
                'system_available_memory_bytes')}
        self.process_resource_sample_monotonic=now_monotonic

    def on_map_cloud(self,msg):
        if msg.header.frame_id!='map' or msg.point_step<=0 or not msg.data:
            self.get_logger().error('Ignoring invalid map cloud for synthetic sensor scene')
            return
        try:
            rows=point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=True)
            points=np.asarray([(float(p[0]),float(p[1]),float(p[2])) for p in rows],dtype=np.float64)
        except Exception as exc:
            self.get_logger().error(f'Unable to decode configured map cloud: {exc}')
            return
        if points.ndim!=2 or points.shape[1]!=3 or not len(points) or not np.isfinite(points).all():
            self.get_logger().error('Configured map cloud has no finite XYZ geometry')
            return
        self.external_map_world=points
        self.external_map_cloud=msg
        if self.pending_path is not None and self.path is None:
            pending=self.pending_path;self.pending_path=None;self.on_path(pending)

    def on_path(self,msg):
        if self.path is not None:return
        if self.map_cloud_topic and self.external_map_world is None:
            self.pending_path=msg
            return
        callback_start=time.monotonic()
        pts=np.array([[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in msg.poses])
        if len(pts)<2 or not np.isfinite(pts).all():raise RuntimeError('Route path must contain at least two finite poses')
        dist=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(pts[:,:2],axis=0),axis=1))]
        if dist[-1]<1.:raise RuntimeError('Route path too short')
        route_length=dist[-1] if self.route_length_requested<0. else self.route_length_requested
        if self.route_length_requested>0. and self.route_length_requested>dist[-1]+1e-3:
            raise RuntimeError(f'Configured route length {self.route_length_requested:.3f} m exceeds path length {dist[-1]:.3f} m')
        self.s=np.append(np.arange(0.,route_length,0.2),route_length)
        self.lap_path=np.column_stack([np.interp(self.s,dist,pts[:,i]) for i in range(3)])
        closure_gap=float(np.linalg.norm(self.lap_path[0,:2]-self.lap_path[-1,:2]))
        if self.trajectory_repeats>1 and closure_gap>self.loop_transition_max_m:
            raise RuntimeError(f'Loop transition gap {closure_gap:.3f} m exceeds configured maximum {self.loop_transition_max_m:.3f} m')
        terminal_end_s=dist[-1] if self.include_terminal_extension else route_length
        terminal_s=np.r_[np.arange(route_length+0.2,terminal_end_s,0.2),terminal_end_s] if terminal_end_s>route_length+1e-3 else np.array([route_length])
        terminal_path=np.column_stack([np.interp(terminal_s,dist,pts[:,i]) for i in range(3)])
        self.terminal_path=terminal_path
        path_parts=[self.lap_path]
        for _ in range(1,self.trajectory_repeats):
            connector_count=max(1,int(math.ceil(closure_gap/0.2)))
            connector=np.linspace(self.lap_path[-1],self.lap_path[0],connector_count+1)[1:]
            path_parts.extend((connector,self.lap_path[1:]))
        self.path=np.vstack(path_parts+[terminal_path[1:]])
        self.lap_length_m=float(route_length)
        self.terminal_extension_m=float(max(0.,terminal_end_s-route_length))
        self.yaw=math.atan2(self.lap_path[1,1]-self.lap_path[0,1],self.lap_path[1,0]-self.lap_path[0,0])
        c,s=math.cos(self.yaw),math.sin(self.yaw);self.rot=np.array([[c,-s],[s,c]])
        if self.external_map_world is not None:
            self.world=self.external_map_world.copy()
            self.map_cloud=self.external_map_cloud
        else:
            gx,gy=np.meshgrid(np.arange(-5,26.01,.5),np.arange(-8,8.01,.5))
            terrain=np.column_stack([gx.ravel(),gy.ravel(),np.zeros(gx.size)])
            landmarks=np.vstack([box(x,y) for x,y in [(1,-5),(5,5),(12,-6),(19,5),(24,-5)]])
            local=np.vstack([terrain,landmarks]);self.world=local.copy();self.world[:,:2]=local[:,:2]@self.rot.T+self.lap_path[0,:2];self.world[:,2]+=self.lap_path[0,2]
            self.map_cloud=point_cloud2.create_cloud_xyz32(Header(frame_id='map'),self.world.tolist())
        obstacle_index=min(50,len(self.lap_path)-1);at=self.lap_path[obstacle_index]
        if self.external_map_world is not None:
            self.obstacle_world=box(float(at[0]),float(at[1]));self.obstacle_world[:,2]+=float(at[2])
        else:
            self.obstacle_world=box(0,0);self.obstacle_world[:,:2]=self.obstacle_world[:,:2]@self.rot.T+at[:2];self.obstacle_world[:,2]+=at[2]
        self.scan_world=np.vstack([self.world,self.obstacle_world]) if self.obstacle else self.world
        self.scan_xyz=np.empty_like(self.scan_world);self.scan_scratch=np.empty(len(self.scan_world),dtype=np.float64)
        self.scan_points=np.zeros(len(self.scan_world),dtype=SCAN_DTYPE);self.scan_points['intensity']=40.
        self.scan_xyz_fields=packed_xyz_view(self.scan_points)
        self.scan_msg=PointCloud2();self.scan_msg.height=1;self.scan_msg.width=len(self.scan_world)
        self.scan_msg.point_step=26;self.scan_msg.row_step=26*len(self.scan_world);self.scan_msg.is_dense=True;self.scan_msg.fields=SCAN_FIELDS
        self.route_msg=Trajectory();self.route_msg.header.frame_id='map'
        for i,xyz in enumerate(self.path):
            p=TrajectoryPoint();p.pose.position.x,p.pose.position.y,p.pose.position.z=map(float,xyz)
            j=min(i+1,len(self.path)-1);k=max(i-1,0);a=math.atan2(self.path[j,1]-self.path[k,1],self.path[j,0]-self.path[k,0])
            p.pose.orientation.z=math.sin(a/2);p.pose.orientation.w=math.cos(a/2)
            p.longitudinal_velocity_mps=0. if i==len(self.path)-1 else 1.;self.route_msg.points.append(p)
        self.map_view.publish(self.map_cloud)
        repeated_transition=closure_gap*max(0,self.trajectory_repeats-1)
        planned_length=self.lap_length_m*self.trajectory_repeats+repeated_transition+self.terminal_extension_m
        self.info.publish(String(data=json.dumps(dict(origin=self.lap_path[0].tolist(),yaw=self.yaw,goal=self.path[-1].tolist(),obstacle_center=at.tolist(),obstacle=self.obstacle,scene='synthetic loop with distinct mission start/finish and connected inter-lap road',route_length_m=self.lap_length_m,trajectory_repeats=self.trajectory_repeats,terminal_extension_m=self.terminal_extension_m,trajectory_length_m=planned_length,loop_transition_gap_m=closure_gap,inter_lap_connector_m=repeated_transition,world_points=len(self.world),map_cloud_input=bool(self.map_cloud_topic),endpoint_gap_m=closure_gap))))
        self.record_timing(dict(event='path_initialization',world_point_count=len(self.world),scan_point_count=len(self.scan_world),
            path_point_count=len(self.path),path_initialization_sec=time.monotonic()-callback_start),callback_start)

    def on_truth(self,msg):
        start=time.monotonic()
        try:header_stamp_ns=int(msg.header.stamp.sec)*1_000_000_000+int(msg.header.stamp.nanosec)
        except (AttributeError,TypeError,ValueError,OverflowError):header_stamp_ns=None
        source_age=source_age_at_receipt_sec(msg,self.get_clock().now().nanoseconds)
        previous_receipt=self.last_odom_callback_monotonic
        if previous_receipt is not None:self.odom_callback_gap_sec=start-previous_receipt
        self.last_odom_callback_monotonic=start
        self.last_odom_receipt_monotonic=start
        self.last_odom_header_stamp_ns=header_stamp_ns
        self.truth=msg
        self.odom_callback_count+=1
        construction_start=time.monotonic()
        t=TwistWithCovarianceStamped();t.header=copy.deepcopy(msg.header);t.header.frame_id='base_link';t.twist=msg.twist
        t.twist.covariance[0]=.0001;t.twist.covariance[35]=.000025
        construction_sec=time.monotonic()-construction_start
        publication_start=time.monotonic();self.motion.publish(t)
        publication_sec=time.monotonic()-publication_start
        callback_total_sec=time.monotonic()-start
        measured=(construction_sec,publication_sec,callback_total_sec,source_age)
        self.odom_callback_max_sec=max(self.odom_callback_max_sec,callback_total_sec)
        for field,value in zip(ODOMETRY_TIMING_FIELDS,measured):
            if value is not None and math.isfinite(value):
                previous=self.odom_callback_timing_max[field]
                self.odom_callback_timing_max[field]=value if previous is None else max(previous,value)
        stamp_ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        velocity=np.array([msg.twist.twist.linear.x,msg.twist.twist.linear.y],dtype=np.float64)
        if stamp_ns>0 and (self.imu_previous_stamp_ns is None or stamp_ns>self.imu_previous_stamp_ns):
            imu=Imu();imu.header=copy.deepcopy(msg.header);imu.header.frame_id='imu_link'
            imu.orientation_covariance[0]=-1.0
            omega=float(msg.twist.twist.angular.z)
            imu.angular_velocity.z=omega
            ax=ay=0.0
            if self.imu_previous_stamp_ns is not None:
                dt=(stamp_ns-self.imu_previous_stamp_ns)*1e-9
                previous=self.imu_previous_velocity
                ax=(velocity[0]-previous[0])/dt-omega*velocity[1]
                ay=(velocity[1]-previous[1])/dt+omega*velocity[0]
            imu.linear_acceleration.x=float(ax);imu.linear_acceleration.y=float(ay)
            # Ideal accelerometer specific force in a level, gravity-aligned base frame.
            imu.linear_acceleration.z=9.80665
            self.imu.publish(imu)
            self.imu_previous_stamp_ns=stamp_ns;self.imu_previous_velocity=velocity

    def on_accel(self,msg):
        receipt=time.monotonic();now_ns=self.get_clock().now().nanoseconds
        stamp_ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        msg.header.frame_id='base_link';self.accel.publish(msg);self.accel_since_tick+=1
        self.last_accel_header_stamp_ns=stamp_ns;self.last_accel_receipt_monotonic=receipt
        self.last_accel_header_age_ros_sec=(now_ns-stamp_ns)*1e-9

    def dump_scene_timing(self,_,response):
        response.success=True;response.message=self.timing.json();return response

    def record_timing(self,record,start):
        record['callback_duration_sec']=time.monotonic()-start
        # The history copy is finalized after publishing. A live diagnostic
        # cannot include the duration of publishing itself, so those two
        # self-measurements are null in the streamed snapshot and complete in
        # the bounded history returned by ~/dump_scene_timing.
        record['diagnostic_recording_sec']=0.
        record['diagnostic_publication_sec']=None
        record['callback_total_duration_sec']=None
        record['callback_finish_monotonic_sec']=None
        record['callback_finish_ros_time_ns']=None
        recording_start=time.monotonic()
        self.timing.append(record)
        record['diagnostic_recording_sec']=time.monotonic()-recording_start
        if self.timing_pub.get_subscription_count():
            publication_start=time.monotonic()
            try:self.timing_pub.publish(String(data=json.dumps(record,separators=(',',':'))))
            except Exception:pass  # Diagnostics must never change scene execution.
            record['diagnostic_publication_sec']=time.monotonic()-publication_start
        finish=time.monotonic()
        record['callback_finish_monotonic_sec']=finish
        record['callback_finish_ros_time_ns']=self.get_clock().now().nanoseconds
        record['callback_total_duration_sec']=finish-start
        if 'callback_monotonic_sec' in record:
            self.last_tick_finished_monotonic=finish

    def map_request(self,req,res):
        res.header.frame_id='map'
        if self.path is None:return res
        cell=PointCloudMapCellWithID();cell.cell_id='campus_synthetic_0';cell.pointcloud=self.map_cloud
        cell.metadata.min_x=float(self.world[:,0].min());cell.metadata.max_x=float(self.world[:,0].max())
        cell.metadata.min_y=float(self.world[:,1].min());cell.metadata.max_y=float(self.world[:,1].max())
        if cell.cell_id not in req.cached_ids:res.new_pointcloud_with_ids=[cell]
        return res

    def publish_tick_state(self,stamp,record,publish_trajectory=True):
        """Publish operator-state messages and optionally the route.

        Keep this after a fresh LiDAR scan on the normal path so an occasional
        slow trajectory publish cannot consume the odometry freshness margin.
        A stale-odometry skip defers route refresh until fresh input returns.
        """
        if publish_trajectory:
            self.route_msg.header.stamp=stamp
            stage_start=time.monotonic();stage_cpu_start=thread_cpu_time_sec();self.route.publish(self.route_msg)
            record['trajectory_publication_sec']=time.monotonic()-stage_start;record['trajectory_published']=True
            record['trajectory_publication_cpu_sec']=thread_cpu_elapsed_sec(stage_cpu_start)
        g=GearCommand(stamp=stamp,command=GearCommand.DRIVE)
        stage_start=time.monotonic();self.gear.publish(g)
        record['gear_publication_sec']=time.monotonic()-stage_start
        stage_start=time.monotonic();self.auto_gear.publish(g)
        record['auto_gear_publication_sec']=time.monotonic()-stage_start
        if self.request_engagement:
            stage_start=time.monotonic();self.engage.publish(Engage(stamp=stamp,engage=True))
            record['engagement_publication_sec']=time.monotonic()-stage_start
        m=OperationModeState();m.stamp=stamp;m.mode=m.AUTONOMOUS;m.is_autoware_control_enabled=True
        stage_start=time.monotonic();self.mode.publish(m)
        record['operation_mode_publication_sec']=time.monotonic()-stage_start

    def tick(self):
        start=time.monotonic();self.scene_tick_index+=1;self.sample_process_resources(start)
        gap=None if self.last_tick_monotonic is None else start-self.last_tick_monotonic
        schedule_delay=None;callback_overrun=None
        if self.last_tick_monotonic is not None:
            expected=self.last_tick_monotonic+.1
            previous_finish=self.last_tick_finished_monotonic or self.last_tick_monotonic
            schedule_delay=max(0.,start-max(expected,previous_finish))
            callback_overrun=max(0.,previous_finish-expected)
        self.last_tick_monotonic=start
        accel_count=self.accel_since_tick;self.accel_since_tick=0
        truth=self.truth;odom_receipt=self.last_odom_receipt_monotonic
        odom_header_stamp=self.last_odom_header_stamp_ns
        odom_count=self.odom_callback_count;odom_max=self.odom_callback_max_sec;odom_gap=self.odom_callback_gap_sec
        odom_timing_max=self.odom_callback_timing_max
        self.odom_callback_count=0;self.odom_callback_max_sec=0.;self.odom_callback_gap_sec=None
        self.odom_callback_timing_max={field:None for field in ODOMETRY_TIMING_FIELDS}
        ros_now_ns=self.get_clock().now().nanoseconds
        truth_ns=odom_header_stamp
        record=dict(callback_monotonic_sec=start,callback_entry_monotonic_sec=start,
            callback_finish_monotonic_sec=None,callback_finish_ros_time_ns=None,
            previous_callback_finish_monotonic_sec=self.last_tick_finished_monotonic,
            process_pid=self.process_pid,process_start_monotonic_sec=self.process_start_monotonic,
            process_start_ros_time_ns=self.process_start_ros_time_ns,
            scene_tick_index=self.scene_tick_index,executor_heartbeat_monotonic_sec=start,
            ros_time_at_tick_entry_ns=ros_now_ns,
            last_odom_receipt_monotonic_sec=odom_receipt,
            last_odom_receipt_age_sec=None if odom_receipt is None else start-odom_receipt,
            last_odom_header_stamp_ns=odom_header_stamp,
            last_odom_header_age_ros_sec=None if self.last_odom_header_stamp_ns is None else (ros_now_ns-self.last_odom_header_stamp_ns)*1e-9,
            last_scan_publish_monotonic_sec=self.last_scan_publish_monotonic,
            last_scan_header_stamp_ns=self.last_scan_header_stamp_ns,
            last_scan_publish_age_sec=None if self.last_scan_publish_monotonic is None else start-self.last_scan_publish_monotonic,
            process_resource_sample_monotonic_sec=self.process_resource_sample_monotonic,
            process_resource_sample_age_sec=None if self.process_resource_sample_monotonic is None else start-self.process_resource_sample_monotonic,
            **self.process_resource_sample,
            callback_gap_sec=gap,timer_schedule_delay_sec=schedule_delay,
            previous_callback_overrun_sec=callback_overrun,
            cached_odometry_age_sec=None if truth_ns is None else (ros_now_ns-truth_ns)*1e-9,
            odometry_callbacks_since_previous_timer=odom_count,odometry_callback_max_duration_sec=odom_max,
            odometry_callback_last_gap_sec=odom_gap,
            **odom_timing_max,
            scan_published=False,scan_skipped=False,skip_reason=None,acquisition_stamp=None,
            acceleration_publications_since_previous_timer=accel_count,
            acceleration_published_since_previous_timer=accel_count>0,
            last_acceleration_header_stamp_ns=self.last_accel_header_stamp_ns,
            last_acceleration_receipt_monotonic_sec=self.last_accel_receipt_monotonic,
            last_acceleration_receipt_age_sec=None if self.last_accel_receipt_monotonic is None else start-self.last_accel_receipt_monotonic,
            last_acceleration_header_age_ros_sec=self.last_accel_header_age_ros_sec,trajectory_published=False,
            scan_publish_gap_sec=None,scan_generation_duration_sec=None,trajectory_publication_cpu_sec=None,
            pointcloud_field_assignment_sec=0.,pointcloud_field_assignment_cpu_sec=None,pointcloud_tobytes_sec=0.,
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
        if self.drop:
            self.publish_tick_state(stamp,record)
            record['scan_skipped']=True;record['skip_reason']='sensor_fault';self.record_timing(record,start);return
        if truth is None:
            self.publish_tick_state(stamp,record)
            record['scan_skipped']=True;record['skip_reason']='odometry_unavailable';self.record_timing(record,start);return
        h=truth.header.stamp;ns=h.sec*1000000000+h.nanosec
        freshness_start=time.monotonic()
        odometry_stale=self.get_clock().now().nanoseconds-ns>100000000
        record['odometry_freshness_evaluation_sec']=time.monotonic()-freshness_start
        if odometry_stale:
            # Avoid synchronous route serialization on stale input: in the
            # single-threaded scene executor it can delay callbacks needed to
            # restore fresh odometry. Keep operator-state publications and
            # refresh the route on the next fresh tick.
            self.publish_tick_state(stamp,record,publish_trajectory=False)
            record['scan_skipped']=True;record['skip_reason']='stale_odometry';self.record_timing(record,start);return
        scan_start=time.monotonic()
        point_start=scan_start;p=truth.pose.pose.position
        if self.lidar_max_range_m>0.0:
            dx=self.scan_world[:,0]-p.x;dy=self.scan_world[:,1]-p.y;dz=self.scan_world[:,2]-p.z
            world=self.scan_world[(dx*dx+dy*dy+dz*dz)<=self.lidar_max_range_m*self.lidar_max_range_m]
        else:
            world=self.scan_world
        point_duration=time.monotonic()-point_start;point_count=len(world)
        if point_count==0:
            self.publish_tick_state(stamp,record)
            record['scan_skipped']=True;record['skip_reason']='empty_sensor_view';self.record_timing(record,start);return
        a=angle(truth.pose.pose.orientation);c,s=math.cos(a),math.sin(a)
        xyz=self.scan_xyz[:point_count];tmp=self.scan_scratch[:point_count]
        transform_start=time.monotonic()
        transform_world_points(world,xyz,tmp,p.x,p.y,p.z,c,s)
        transform_duration=time.monotonic()-transform_start
        # FLOAT64 absolute ns requires 256 ns quantization at contemporary epoch.
        ns=ns//256*256
        serial_start=time.monotonic();serial_cpu_start=thread_cpu_time_sec();packed=self.scan_points[:point_count]
        field_assignment_start=time.monotonic();field_assignment_cpu_start=thread_cpu_time_sec()
        self.scan_xyz_fields[:point_count]=xyz
        packed['timestamp']=float(ns)
        field_assignment_cpu_end=thread_cpu_time_sec()
        field_assignment_duration=time.monotonic()-field_assignment_start
        field_assignment_cpu_duration=thread_cpu_delta_sec(field_assignment_cpu_start,field_assignment_cpu_end)
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
        msg=self.scan_msg;msg.width=point_count;msg.row_step=msg.point_step*point_count;msg.header.frame_id='synthetic_lidar';msg.header.stamp.sec=ns//1000000000;msg.header.stamp.nanosec=ns%1000000000
        data_assignment_start=time.monotonic();data_cpu_start=thread_cpu_time_sec()
        msg.data=packed_data
        data_cpu_end=thread_cpu_time_sec()
        serialization_end=time.monotonic()
        serialization_duration=serialization_end-serial_start
        data_assignment_duration=serialization_end-data_assignment_start
        generation_duration=time.monotonic()-scan_start
        data_cpu_duration=thread_cpu_delta_sec(data_cpu_start,data_cpu_end)
        serialization_cpu_duration=thread_cpu_delta_sec(serial_cpu_start,data_cpu_end)
        cpu_samples=(serial_cpu_start,field_assignment_cpu_start,tobytes_cpu_start,array_cpu_start,data_cpu_start,
            field_assignment_cpu_duration,tobytes_cpu_duration,array_cpu_duration,data_cpu_duration,serialization_cpu_duration)
        record['pointcloud_field_assignment_sec']=field_assignment_duration
        record['pointcloud_field_assignment_cpu_sec']=field_assignment_cpu_duration
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
        self.last_scan_header_stamp_ns=ns
        record['header_age_at_publication_sec']=(publication_ros_ns-ns)*1e-9
        record['point_generation_sec']=point_duration
        record['coordinate_transform_sec']=transform_duration
        record['pointcloud_serialization_sec']=serialization_duration
        record['scan_generation_duration_sec']=published-scan_start
        record['scan_publish_gap_sec']=None if self.last_scan_publish_monotonic is None else published-self.last_scan_publish_monotonic
        self.last_scan_publish_monotonic=published
        self.publish_tick_state(stamp,record)
        self.record_timing(record,start)


def main():
    rclpy.init();n=Scene();exit_reason='normal_shutdown'
    try:
        while rclpy.ok() and not n.stopping:rclpy.spin_once(n,timeout_sec=.02)
    except KeyboardInterrupt:exit_reason='keyboard_interrupt'
    except ExternalShutdownException:exit_reason='external_shutdown'
    except RuntimeError as exc:
        # rclpy Humble can surface this specific wait-set conversion error if
        # SIGINT invalidates the context while spin_once is taking a ready DDS
        # message. Treat it as expected only after ROS reports shutdown; the
        # same error while the context is active remains a real failure.
        if not rclpy.ok() and str(exc).startswith('Unable to convert call argument to Python object'):
            exit_reason='shutdown_context_rmw_race'
        else:
            exit_reason='uncaught_exception'
            n.log_lifecycle('process_exception',exception_type=type(exc).__name__,exception=str(exc))
            raise
    except BaseException as exc:
        exit_reason='uncaught_exception'
        n.log_lifecycle('process_exception',exception_type=type(exc).__name__,exception=str(exc))
        raise
    finally:
        n.log_lifecycle('process_exit',exit_reason=exit_reason,
            last_tick_finished_monotonic_sec=n.last_tick_finished_monotonic,
            last_odom_receipt_monotonic_sec=n.last_odom_receipt_monotonic,
            last_scan_publish_monotonic_sec=n.last_scan_publish_monotonic,
            last_scan_header_stamp_ns=n.last_scan_header_stamp_ns)
        n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
