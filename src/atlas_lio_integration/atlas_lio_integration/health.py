import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu,PointCloud2
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
class LioHealth(Node):
 def __init__(self):
  super().__init__('atlas_lio_health');self.timeout=float(self.declare_parameter('timeout_sec',1.0).value);self.seen={};self.pub=self.create_publisher(DiagnosticArray,'/atlas/lio/health',10)
  for key,typ,topic in [('lidar',PointCloud2,'/atlas/sensors/lidar/points'),('imu',Imu,'/atlas/sensors/imu/data'),('odom',Odometry,'/atlas/lio/odometry')]:self.create_subscription(typ,topic,lambda m,k=key:self.seen.__setitem__(k,time.monotonic()),20)
  self.create_timer(.25,self.tick)
 def tick(self):
  now=time.monotonic();a=DiagnosticArray();a.header.stamp=self.get_clock().now().to_msg()
  for k in ('lidar','imu','odom'):
   age=now-self.seen[k] if k in self.seen else float('inf');s=DiagnosticStatus();s.name='atlas_lio/'+k;s.hardware_id='synthetic';s.level=DiagnosticStatus.OK if age<=self.timeout else DiagnosticStatus.ERROR;s.message='healthy' if s.level==DiagnosticStatus.OK else 'missing_or_stale';s.values=[KeyValue(key='age_sec',value='%.3f'%age)];a.status.append(s)
  self.pub.publish(a)
def main():
 rclpy.init();n=LioHealth()
 try:rclpy.spin(n)
 finally:n.destroy_node();rclpy.try_shutdown()
