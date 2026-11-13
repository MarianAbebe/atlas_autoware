#!/usr/bin/env python3
"""Measured cluster bounds + EKF TF -> stationary predictions for an explicitly static scene.
Pinned clustering publishes centroids/UNKNOWN only. Membership comes exclusively
from its colored debug cloud. This adapter neither re-clusters nor reads truth.
"""
import copy,math,time,json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer,TransformListener,TransformException
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from autoware_perception_msgs.msg import DetectedObjects,PredictedObjects,PredictedObject,PredictedPath,Shape


def measured_bounds(msg,cloud):
    if msg.header!=cloud.header:raise ValueError('cluster header mismatch')
    if not msg.objects:
        if cloud.width*cloud.height:raise ValueError('unexpected cluster points')
        return []
    f={f.name:f.offset for f in cloud.fields}
    dtype=np.dtype(dict(names=['x','y','z','rgb'],formats=['<f4','<f4','<f4','<u4'],offsets=[f[k] for k in ['x','y','z','rgb']],itemsize=cloud.point_step))
    a=np.frombuffer(cloud.data,dtype=dtype);xyz=np.column_stack([a[k] for k in ['x','y','z']]);rgb=a['rgb']&0xffffff
    if not np.isfinite(xyz).all():raise ValueError('nonfinite cluster geometry')
    edges=np.r_[0,np.flatnonzero(rgb[1:]!=rgb[:-1])+1,len(rgb)]
    if len(edges)-1!=len(msg.objects):raise ValueError('membership count mismatch')
    result=[]
    for obj,lo,hi in zip(msg.objects,edges[:-1],edges[1:]):
        points=xyz[lo:hi];p=obj.kinematics.pose_with_covariance.pose.position
        if np.linalg.norm(points.mean(axis=0)-[p.x,p.y,p.z])>.05:raise ValueError('membership centroid mismatch')
        low=points.min(axis=0);high=points.max(axis=0)
        if np.any(high-low<=0):raise ValueError('degenerate cluster bounds')
        result.append(((low+high)/2,high-low))
    return result


def predictions(msg,transform,bounds):
    q=transform.rotation;a=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));c,s=math.cos(a),math.sin(a)
    out=PredictedObjects();out.header=copy.deepcopy(msg.header);out.header.frame_id='map'
    for i,(d,(center,size)) in enumerate(zip(msg.objects,bounds)):
        p=PredictedObject();p.object_id.uuid[0]=1;p.object_id.uuid[1]=i%256
        # Deterministic observed presence, not learned confidence or tracked identity.
        p.existence_probability=1.;p.classification=copy.deepcopy(d.classification);p.shape.type=Shape.BOUNDING_BOX
        p.shape.dimensions.x,p.shape.dimensions.y,p.shape.dimensions.z=map(float,size)
        pose=p.kinematics.initial_pose_with_covariance.pose;x,y,z=map(float,center)
        pose.position.x=transform.translation.x+c*x-s*y;pose.position.y=transform.translation.y+s*x+c*y;pose.position.z=transform.translation.z+z
        pose.orientation.z=math.sin(a/2);pose.orientation.w=math.cos(a/2)
        path=PredictedPath();path.confidence=1.;path.time_step.nanosec=500000000;path.path=[copy.deepcopy(pose) for _ in range(11)]
        p.kinematics.predicted_paths=[path];out.objects.append(p)
    return out


class Adapter(Node):
    def __init__(self):
        super().__init__('static_objects_adapter',namespace='/atlas_loc');self.clouds={};self.pending={};self.cloud_receipts={}
        self.transform_pending={}
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.tf_retry_timeout=float(self.declare_parameter('tf_retry_timeout_sec',0.12).value)
        if not 0.01<=self.tf_retry_timeout<=0.25:raise ValueError('tf_retry_timeout_sec must be between 0.01 and 0.25')
        self.pub=self.create_publisher(PredictedObjects,'obstacles/predicted_objects',10)
        self.timing_pub=self.create_publisher(String,'diagnostics/object_adapter_timing',10)
        self.last_cloud_receipt=None;self.last_detected_receipt=None;self.last_publish_receipt=None
        self.create_subscription(DetectedObjects,'obstacles/detected_objects',self.detected,10)
        # This reliable depth-1 topic is the exact per-frame membership
        # companion to DetectedObjects. Best-effort sensor QoS could lose the
        # cloud while the reliable detection remained queued, leaving a
        # detection waiting without the geometry needed to publish a valid
        # prediction. Depth 1 prefers the newest pair and does not retain a
        # backlog; message stamps are still matched exactly below.
        self.create_subscription(PointCloud2,'clusters',self.cloud,
            QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE))
        self.create_timer(0.01,self.retry_transforms)
    @staticmethod
    def key(m):return m.header.stamp.sec,m.header.stamp.nanosec
    def timing(self,record):
        record['measurement']='object_adapter';record['callback_finished_monotonic_sec']=time.monotonic()
        try:self.timing_pub.publish(String(data=json.dumps(record,separators=(',',':'))))
        except Exception:pass
    def cloud(self,m):
        entered=time.monotonic();previous=self.last_cloud_receipt;self.last_cloud_receipt=entered
        k=self.key(m);self.clouds[k]=m;self.cloud_receipts[k]=entered
        if len(self.clouds)>20:self.clouds.pop(next(iter(self.clouds)))
        if len(self.cloud_receipts)>20:self.cloud_receipts.pop(next(iter(self.cloud_receipts)))
        waiting=k in self.pending
        if waiting:
            msg,detected_receipt,detected_gap,detected_header_age=self.pending.pop(k)
            self.receive(msg,detected_receipt,detected_gap,detected_header_age,entered)
        else:
            now=self.get_clock().now().nanoseconds*1e-9
            self.timing(dict(event='cluster_membership_cloud_received',header_stamp=stamp(m.header.stamp),
                header_age_at_receipt_sec=now-stamp(m.header.stamp),receipt_gap_sec=None if previous is None else entered-previous,
                pending_detected_object_waiting=False,points=m.width*m.height))
    def detected(self,msg):
        entered=time.monotonic();previous=self.last_detected_receipt;self.last_detected_receipt=entered
        now=self.get_clock().now().nanoseconds*1e-9;header_age=now-stamp(msg.header.stamp)
        gap=None if previous is None else entered-previous
        self.receive(msg,entered,gap,header_age,self.cloud_receipts.get(self.key(msg)))
    def receive(self,msg,detected_receipt=None,detected_gap=None,detected_header_age=None,cloud_receipt=None,
                transform_retry_started=None):
        processing_start=time.monotonic()
        if detected_receipt is None:
            detected_receipt=processing_start
            detected_header_age=self.get_clock().now().nanoseconds*1e-9-stamp(msg.header.stamp)
        k=self.key(msg)
        if k not in self.clouds:
            self.pending[k]=(msg,detected_receipt,detected_gap,detected_header_age)
            if len(self.pending)>20:self.pending.pop(next(iter(self.pending)))
            self.timing(dict(event='detected_objects_waiting_for_membership',header_stamp=stamp(msg.header.stamp),
                header_age_at_receipt_sec=detected_header_age,detected_receipt_gap_sec=detected_gap,
                pending_count=len(self.pending)))
            return
        cloud=self.clouds[k];now=self.get_clock().now().nanoseconds*1e-9
        timing=dict(event='paired_detection_processing',header_stamp=stamp(msg.header.stamp),
            header_age_at_detected_receipt_sec=detected_header_age,detected_receipt_gap_sec=detected_gap,
            detected_to_membership_cloud_wait_sec=None if cloud_receipt is None else cloud_receipt-detected_receipt,
            detected_to_processing_start_sec=processing_start-detected_receipt,
            membership_cloud_points=cloud.width*cloud.height)
        try:
            tf_start=time.monotonic()
            tf=self.buffer.lookup_transform('map',msg.header.frame_id,Time.from_msg(msg.header.stamp)).transform
            timing['tf_lookup_sec']=time.monotonic()-tf_start
            bounds_start=time.monotonic();bounds=measured_bounds(msg,cloud);timing['bounds_processing_sec']=time.monotonic()-bounds_start
        except TransformException as error:
            if transform_retry_started is None:
                self.transform_pending[k]=(msg,detected_receipt,detected_gap,detected_header_age,cloud_receipt,processing_start)
                if len(self.transform_pending)>20:
                    oldest=next(iter(self.transform_pending))
                    dropped=self.transform_pending.pop(oldest)
                    self.timing(dict(event='adapter_input_rejected',reason='transform_retry_queue_capacity',
                        header_stamp=stamp(dropped[0].header.stamp),pending_transform_count=len(self.transform_pending)))
                timing.update(event='waiting_for_timestamped_transform',error=str(error),
                    pending_transform_count=len(self.transform_pending))
                self.timing(timing)
                return
            if time.monotonic()-transform_retry_started<self.tf_retry_timeout:
                return
            self.transform_pending.pop(k,None)
            timing.update(event='adapter_input_rejected',error=str(error),
                tf_tree_yaml=self.buffer.all_frames_as_yaml(),
                transform_retry_duration_sec=time.monotonic()-transform_retry_started,
                total_processing_sec=time.monotonic()-processing_start)
            self.timing(timing)
            self.get_logger().warn(str(error),throttle_duration_sec=2.);return
        except ValueError as error:
            self.transform_pending.pop(k,None)
            timing.update(event='adapter_input_rejected',error=str(error),total_processing_sec=time.monotonic()-processing_start)
            self.timing(timing)
            self.get_logger().warn(str(error),throttle_duration_sec=2.);return
        self.transform_pending.pop(k,None)
        output=predictions(msg,tf,bounds);self.pub.publish(output);published=time.monotonic();ros_published=self.get_clock().now().nanoseconds*1e-9
        timing.update(event='predicted_objects_published',object_count=len(output.objects),
            header_age_at_publication_sec=ros_published-stamp(output.header.stamp),
            detected_to_publish_sec=published-detected_receipt,
            predicted_receipt_gap_sec=None if self.last_publish_receipt is None else published-self.last_publish_receipt,
            total_processing_sec=published-processing_start)
        self.last_publish_receipt=published;self.timing(timing)

    def retry_transforms(self):
        # The TF listener shares this single-threaded executor. A detection
        # callback can therefore run before the corresponding TF subscription
        # callback; retry after returning to the executor, preserving the exact
        # measurement timestamp and bounding both wait and queue size.
        for k,item in list(self.transform_pending.items()):
            if k not in self.clouds:
                self.transform_pending.pop(k,None)
                continue
            msg,receipt,gap,age,cloud_receipt,started=item
            self.receive(msg,receipt,gap,age,cloud_receipt,started)

def stamp(s):return s.sec+s.nanosec*1e-9

def main():
    rclpy.init();n=Adapter()
    try:rclpy.spin(n)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    except RuntimeError:
        if rclpy.ok():raise  # Never suppress a failure while the graph is running.
    finally:n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
