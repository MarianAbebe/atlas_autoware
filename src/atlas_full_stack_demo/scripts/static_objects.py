#!/usr/bin/env python3
"""Measured cluster bounds + EKF TF -> stationary predictions for an explicitly static scene.
Pinned clustering publishes centroids/UNKNOWN only. Membership comes exclusively
from its colored debug cloud. This adapter neither re-clusters nor reads truth.
"""
import copy,math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer,TransformListener,TransformException
from sensor_msgs.msg import PointCloud2
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
        super().__init__('static_objects_adapter',namespace='/atlas_loc');self.clouds={};self.pending={}
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.pub=self.create_publisher(PredictedObjects,'obstacles/predicted_objects',10)
        self.create_subscription(DetectedObjects,'obstacles/detected_objects',self.receive,10)
        self.create_subscription(PointCloud2,'clusters',self.cloud,qos_profile_sensor_data)
    @staticmethod
    def key(m):return m.header.stamp.sec,m.header.stamp.nanosec
    def cloud(self,m):
        k=self.key(m);self.clouds[k]=m
        if len(self.clouds)>20:self.clouds.pop(next(iter(self.clouds)))
        if k in self.pending:self.receive(self.pending.pop(k))
    def receive(self,msg):
        k=self.key(msg)
        if k not in self.clouds:
            self.pending[k]=msg
            if len(self.pending)>20:self.pending.pop(next(iter(self.pending)))
            return
        try:
            bounds=measured_bounds(msg,self.clouds[k])
            tf=self.buffer.lookup_transform('map',msg.header.frame_id,Time.from_msg(msg.header.stamp)).transform
        except (TransformException,ValueError) as error:
            self.get_logger().warn(str(error),throttle_duration_sec=2.);return
        self.pub.publish(predictions(msg,tf,bounds))

def main():
    rclpy.init();n=Adapter()
    try:rclpy.spin(n)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    except RuntimeError:
        if rclpy.ok():raise  # Never suppress a failure while the graph is running.
    finally:n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
