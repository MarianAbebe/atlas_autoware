import math,time
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
from sensor_msgs.msg import PointCloud2,Imu,NavSatFix,JointState,Image
class HealthMonitor(Node):
 def __init__(self):
  super().__init__('atlas_sensor_health_monitor'); ns=self.declare_parameter('namespace','/atlas/sensors').value.rstrip('/');self.timeout=float(self.declare_parameter('timeout_sec',1.0).value);self.seen={}; self.types={'lidar/points':PointCloud2,'imu/data':Imu,'gnss/fix':NavSatFix,'wheel_states':JointState,'stereo/left/image_raw':Image,'stereo/right/image_raw':Image}
  for name,t in self.types.items():self.create_subscription(t,ns+'/'+name,lambda m,n=name:self.on(m,n),10)
  self.pub=self.create_publisher(DiagnosticArray,ns+'/health',10);self.create_timer(.25,self.report)
 def on(self,m,n):
  valid=m.header.stamp.sec>0
  if isinstance(m,Imu): valid=valid and math.isfinite(m.angular_velocity.z) and math.isfinite(m.linear_acceleration.y)
  if isinstance(m,NavSatFix): valid=valid and math.isfinite(m.latitude) and -90<=m.latitude<=90 and -180<=m.longitude<=180
  self.seen[n]=(time.monotonic(),valid,'invalid timestamp or non-finite value' if not valid else 'fresh')
 def report(self):
  a=DiagnosticArray();a.header.stamp=self.get_clock().now().to_msg();now=time.monotonic()
  for n in self.types:
   age,ok,reason=self.seen.get(n,(float('inf'),False,'no messages received'))
   s=DiagnosticStatus();s.name='atlas_sensor/'+n;s.hardware_id='synthetic';s.level=DiagnosticStatus.OK if ok and age<=self.timeout else (DiagnosticStatus.WARN if age<=2*self.timeout else DiagnosticStatus.ERROR);s.message='healthy' if s.level==0 else reason if not ok else 'stale input';s.values=[KeyValue(key='age_sec',value='%.3f'%age)];a.status.append(s)
  self.pub.publish(a)
def main():
 rclpy.init();n=HealthMonitor()
 try:rclpy.spin(n)
 finally:n.destroy_node();rclpy.try_shutdown()
