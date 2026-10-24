#!/usr/bin/env python3
"""Bounded machine-readable live contract validator; launch stack before running."""
import json, time, math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2,Imu,NavSatFix,JointState,Image
from diagnostic_msgs.msg import DiagnosticArray
class V(Node):
 def __init__(self):
  super().__init__('sensor_stack_validator');self.got={};self.times={};self.health=False
  for n,t in [('lidar',PointCloud2),('imu',Imu),('gnss',NavSatFix),('wheel',JointState),('left',Image),('right',Image)]:self.create_subscription(t,'/atlas/sensors/'+({'lidar':'lidar/points','imu':'imu/data','gnss':'gnss/fix','wheel':'wheel_states','left':'stereo/left/image_raw','right':'stereo/right/image_raw'}[n]),lambda m,k=n:self.cb(k,m),10)
  self.create_subscription(DiagnosticArray,'/atlas/sensors/health',lambda m:setattr(self,'health',len(m.status)>=6),10)
 def cb(self,k,m):
  self.got[k]=m;self.times.setdefault(k,[]).append(m.header.stamp.sec+m.header.stamp.nanosec*1e-9)
def main():
 rclpy.init();n=V();end=time.monotonic()+5
 while time.monotonic()<end:rclpy.spin_once(n, timeout_sec=.1)
 checks={k:bool(k in n.got) for k in ('lidar','imu','gnss','wheel','left','right')};checks['health']=bool(n.health)
 if 'lidar' in n.got: checks['finite_lidar']=bool(n.got['lidar'].width>0 and n.got['lidar'].is_dense)
 if 'imu' in n.got: checks['imu_valid']=math.isfinite(n.got['imu'].angular_velocity.z) and n.got['imu'].orientation_covariance[0]==-1
 if 'gnss' in n.got: checks['gnss_plausible']=abs(n.got['gnss'].latitude)<=90 and abs(n.got['gnss'].longitude)<=180
 checks['stereo_sync']=bool('left' in n.got and 'right' in n.got and n.times['left'][-1]==n.times['right'][-1])
 checks['monotonic']=bool(all(all(b>=a for a,b in zip(v,v[1:])) for v in n.times.values()))
 checks={key:bool(value) for key,value in checks.items()}
 out={'result':'PASS' if all(checks.values()) else 'FAIL','checks':checks};print(json.dumps(out));n.destroy_node();rclpy.shutdown();return 0 if out['result']=='PASS' else 1
if __name__=='__main__': raise SystemExit(main())
