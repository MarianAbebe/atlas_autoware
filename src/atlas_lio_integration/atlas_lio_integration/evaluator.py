import json,math,time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from .metrics import pose_error,yaw_from_quaternion
class Evaluator(Node):
 def __init__(self):
  super().__init__('atlas_lio_evaluator');self.truth=[];self.errors=[];self.odom_times=[];self.create_subscription(PoseStamped,'/atlas/sensors/evaluation/ground_truth',self.gt,50);self.create_subscription(Odometry,'/atlas/lio/odometry',self.odom,50)
 def gt(self,m): self.truth.append((m.header.stamp.sec+m.header.stamp.nanosec*1e-9,m.pose.position.x,m.pose.position.y,yaw_from_quaternion(m.pose.orientation.z,m.pose.orientation.w)));self.truth=self.truth[-200:]
 def odom(self,m):
  t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
  if not self.truth:return
  q=m.pose.pose.orientation; e=(m.pose.pose.position.x,m.pose.pose.position.y,yaw_from_quaternion(q.z,q.w)); g=min(self.truth,key=lambda x:abs(x[0]-t));self.errors.append(pose_error(e,g[1:]));self.odom_times.append(t)
 def result(self):
  if not self.errors:return {'result':'FAIL','reason':'no matched estimator odometry'}
  te=[x[0] for x in self.errors];oe=[x[1] for x in self.errors];duration=max(self.odom_times)-min(self.odom_times) if len(self.odom_times)>1 else 0
  return {'result':'PASS' if all(math.isfinite(x) for x in te+oe) else 'FAIL','samples':len(te),'translation_rmse_m':math.sqrt(sum(x*x for x in te)/len(te)),'orientation_rmse_rad':math.sqrt(sum(x*x for x in oe)/len(oe)),'drift_m':te[-1]-te[0],'output_rate_hz':(len(self.odom_times)-1)/duration if duration>0 else 0}
def main():
 rclpy.init();n=Evaluator();end=time.monotonic()+float(n.declare_parameter('duration_sec',15.0).value)
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
 print(json.dumps(n.result()));n.destroy_node();rclpy.shutdown()
