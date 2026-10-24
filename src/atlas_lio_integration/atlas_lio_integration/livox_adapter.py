"""PointCloud2-to-Livox CustomMsg adapter for the pinned FAST-LIO2 port.

This is a format adapter, not an estimator. It preserves the simulator's
per-point `time` (seconds from scan start) as Livox `offset_time` nanoseconds.
"""
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
try:
 from livox_interfaces.msg import CustomMsg,CustomPoint
except ImportError:
 CustomMsg=CustomPoint=None
class Adapter(Node):
 def __init__(self):
  if CustomMsg is None: raise RuntimeError('livox_interfaces is absent: build the pinned FAST-LIO ROS 2 source first')
  super().__init__('atlas_livox_adapter');self.pub=self.create_publisher(CustomMsg,'/atlas/lio/livox',10);self.create_subscription(PointCloud2,'/atlas/sensors/lidar/points',self.cb,10)
 def cb(self,m):
  fields={f.name:f.offset for f in m.fields}
  if not all(k in fields for k in ('x','y','z','time')):self.get_logger().error('PointCloud2 needs x,y,z,time fields');return
  out=CustomMsg();out.header=m.header;out.timebase=m.header.stamp.sec*1000000000+m.header.stamp.nanosec;out.point_num=m.width*m.height;out.lidar_id=0
  for i in range(out.point_num):
   base=i*m.point_step;p=CustomPoint();p.x,p.y,p.z=struct.unpack_from('<fff',m.data,base+fields['x']);t=struct.unpack_from('<f',m.data,base+fields['time'])[0];p.offset_time=max(0,int(t*1e9));p.reflectivity=1;p.tag=0;p.line=0;out.points.append(p)
  self.pub.publish(out)
def main():
 rclpy.init();n=Adapter()
 try:rclpy.spin(n)
 finally:n.destroy_node();rclpy.try_shutdown()
