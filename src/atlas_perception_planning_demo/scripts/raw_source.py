#!/usr/bin/env python3
"""Raw Livox-format synthetic acquisition; no segmented-cloud/control publisher."""
import copy
import json
import math
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String
from autoware_perception_msgs.msg import PredictedObjects

class Source(Node):
    def __init__(self):
        super().__init__('obstacle_source')
        self.raw=self.create_publisher(PointCloud2,'lidar/raw',qos_profile_sensor_data)
        self.twist=self.create_publisher(TwistWithCovarianceStamped,'lidar/twist',100)
        self.objects=self.create_publisher(PredictedObjects,'obstacles/predicted_objects',10)
        self.status=self.create_publisher(String,'obstacles/source_status',10)
        self.create_subscription(Odometry,'sim/output/odometry',self.odom,50)
        self.create_subscription(String,'obstacles/inject',self.inject,10)
        self.create_subscription(String,'test_shutdown',lambda _:setattr(self,'shutdown_requested',True),10)
        self.shutdown_requested=False;self.states=deque(maxlen=200);self.mode='clear';self.anchor=None;self.variant='ahead';self.last=None;self.frozen=None
        self.create_timer(.1,self.tick)
    def odom(self,m):
        q=m.pose.pose.orientation;p=m.pose.pose.position
        angle=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        self.states.append([t,p.x,p.y,p.z,angle])
        motion=TwistWithCovarianceStamped();motion.header.stamp=m.header.stamp;motion.header.frame_id=m.child_frame_id;motion.twist=m.twist;self.twist.publish(motion)
    def inject(self,m):
        data=json.loads(m.data);self.mode=data['mode']
        if self.mode=='obstacle':
            self.anchor={k:float(data[k]) for k in ['x','y','z','yaw']};self.variant=data.get('variant','ahead')
        elif self.mode=='freeze':self.frozen=copy.deepcopy(self.last)
        elif self.mode not in ['clear','drop']:raise ValueError('Invalid raw fixture mode')
    def tick(self):
        now=self.get_clock().now();objects=PredictedObjects();objects.header=Header(stamp=now.to_msg(),frame_id='map');self.objects.publish(objects)
        if len(self.states)<8:return
        if self.mode=='drop':return
        if self.mode=='freeze':
            if self.frozen is not None:self.raw.publish(self.frozen)
            return
        first_ns=(now.nanoseconds-80000000)//256*256;first=first_ns*1e-9
        a=np.array(self.states)
        if a[0,0]>first or a[-1,0]<first+.04:return
        pose=np.array([np.interp(first,a[:,0],a[:,i]) for i in range(1,5)])
        gx,gy=np.meshgrid(np.arange(-4,35.001,.5),np.arange(-8,8.001,.5))
        visible=np.hypot(gx.ravel(),gy.ravel())>=1.0
        local=np.column_stack([gx.ravel()[visible],gy.ravel()[visible],np.full(np.count_nonzero(visible),-pose[2])])
        c,s=np.cos(pose[3]),np.sin(pose[3]);rot=np.array([[c,-s],[s,c]])
        world=local.copy();world[:,:2]=local[:,:2]@rot.T+pose[:2];world[:,2]+=pose[2]
        intensity=np.full(len(world),10.,dtype=np.float32);ground_count=len(world)
        if self.mode=='obstacle' and self.anchor:
            anchors=[(self.anchor,200)]
            if self.variant=='ahead':
                b=dict(self.anchor);b['x']-=math.sin(b['yaw'])*4;b['y']+=math.cos(b['yaw'])*4;anchors.append((b,150))
            elif self.variant=='off_route':anchors=[(self.anchor,150)]
            for b,label in anchors:
                x,y,z=np.meshgrid(np.linspace(-.4,.4,9),np.linspace(-.3,.3,7),np.linspace(.2,1.,5))
                xx=x.ravel();yy=y.ravel();zz=z.ravel();bc,bs=np.cos(b['yaw']),np.sin(b['yaw'])
                block=np.column_stack([b['x']+bc*xx-bs*yy,b['y']+bs*xx+bc*yy,b['z']+zz])
                world=np.vstack([world,block]);intensity=np.r_[intensity,np.full(len(block),label,dtype=np.float32)]
        count=len(world);offset=(np.linspace(0,40000000,count)//256*256).astype(np.int64);times=(first_ns+offset)*1e-9
        px=np.interp(times,a[:,0],a[:,1]);py=np.interp(times,a[:,0],a[:,2]);pz=np.interp(times,a[:,0],a[:,3]);angle=np.interp(times,a[:,0],a[:,4]);ca,sa=np.cos(angle),np.sin(angle)
        dx=world[:,0]-px;dy=world[:,1]-py
        xyz=np.column_stack([ca*dx+sa*dy,-sa*dx+ca*dy,world[:,2]-pz])
        dtype=np.dtype({'names':['x','y','z','intensity','tag','line','timestamp'],'formats':['<f4','<f4','<f4','<f4','u1','u1','<f8'],'offsets':[0,4,8,12,16,17,18],'itemsize':26})
        packed=np.zeros(count,dtype=dtype)
        for i,k in enumerate(['x','y','z']):packed[k]=xyz[:,i]
        packed['intensity']=intensity;packed['timestamp']=(first_ns+offset).astype(np.float64)
        msg=PointCloud2();msg.header.frame_id='atlas_closed_loop_base_link';msg.header.stamp.sec=int(first_ns//1000000000);msg.header.stamp.nanosec=int(first_ns%1000000000)
        msg.height=1;msg.width=count;msg.point_step=26;msg.row_step=26*count;msg.is_dense=True
        msg.fields=[PointField(name=k,offset=off,datatype=typ,count=1) for k,off,typ in [('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),('tag',16,2),('line',17,2),('timestamp',18,8)]]
        msg.data=packed.tobytes();self.raw.publish(msg);self.last=copy.deepcopy(msg)
        expected=world.copy();d=world[:,:2]-pose[:2];expected[:,:2]=d@rot;expected[:,2]-=pose[2]
        self.status.publish(String(data=json.dumps({'stamp':now.nanoseconds*1e-9,'scan_stamp':first,'mode':self.mode,'anchor':self.anchor,'variant':self.variant,'ground_points':ground_count,'obstacle_points':count-ground_count,'points':count,'scan_duration_sec':float(offset[-1]*1e-9),'expected_max_motion_correction_m':float(np.linalg.norm(xyz-expected,axis=1).max()),'synthetic_raw':True})))

def main():
    rclpy.init();node=Source()
    try:
        while rclpy.ok() and not node.shutdown_requested:rclpy.spin_once(node,timeout_sec=.02)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
