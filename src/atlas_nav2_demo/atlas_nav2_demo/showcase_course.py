"""Markers for the deterministic outdoor course; collision rectangles match world.py."""
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from .world import rectangles

class Course(Node):
 def __init__(self):
  super().__init__('ackermann_showcase_course');self.scenario=self.declare_parameter('scenario','open').value;self.pub=self.create_publisher(MarkerArray,'/atlas_ackermann/showcase/course',1);self.create_timer(1.,self.draw);self.draw()
 def marker(self,i,kind,x,y,z,sx,sy,sz,r,g,b,a=1.):
  m=Marker();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg();m.ns='atlas_course';m.id=i;m.type=kind;m.action=Marker.ADD;m.pose.position.x=x;m.pose.position.y=y;m.pose.position.z=z;m.pose.orientation.w=1.;m.scale.x=sx;m.scale.y=sy;m.scale.z=sz;m.color.r=r;m.color.g=g;m.color.b=b;m.color.a=a;return m
 def draw(self):
  out=MarkerArray();out.markers.append(self.marker(1,Marker.CUBE,2.,0.,-.04,7.6,5.6,.08,.13,.28,.13));out.markers.append(self.marker(2,Marker.CUBE,2.,0.,0.,7.2,2.2,.03,.22,.22,.22));i=10
  for rect in rectangles(self.scenario):
   out.markers.append(self.marker(i,Marker.CUBE,(rect.xmin+rect.xmax)/2,(rect.ymin+rect.ymax)/2,.35,rect.xmax-rect.xmin,rect.ymax-rect.ymin,.7,.95,.25,.03));i+=1
   for x,y in ((rect.xmin,rect.ymin),(rect.xmin,rect.ymax),(rect.xmax,rect.ymin),(rect.xmax,rect.ymax)):
    out.markers.append(self.marker(i,Marker.CYLINDER,x,y,.35,.16,.16,.7,1.,.35,.02));i+=1
  for x,y in ((-1.4,2.1),(0.2,2.2),(3.8,2.0),(5.2,-2.0)):
   trunk=self.marker(i,Marker.CYLINDER,x,y,.45,.18,.18,.9,.30,.16,.06);out.markers.append(trunk);i+=1;out.markers.append(self.marker(i,Marker.SPHERE,x,y,1.25,1.,1.,1.,.05,.42,.10));i+=1
  self.pub.publish(out)
def main():
 rclpy.init();n=Course()
 try:rclpy.spin(n)
 finally:n.destroy_node();rclpy.try_shutdown()
