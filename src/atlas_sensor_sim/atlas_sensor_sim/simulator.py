import math, random, struct, time
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, NavSatFix, NavSatStatus, PointCloud2, PointField
from .model import WHEELS, scan_point_cloud, encoder_ticks, local_to_geodetic, stereo_pixels, trajectory, wheel_speeds, require_positive

class SensorSimulator(Node):
    def __init__(self):
        super().__init__('atlas_sensor_simulator')
        p=lambda n,d:self.declare_parameter(n,d).value
        self.ns=p('namespace','/atlas/sensors').rstrip('/'); self.seed=int(p('seed',42)); self.rng=random.Random(self.seed)
        self.lidar_rate=float(p('lidar_rate_hz',10.0)); self.imu_rate=float(p('imu_rate_hz',200.0)); self.gnss_rate=float(p('gnss_rate_hz',5.0)); self.camera_rate=float(p('camera_rate_hz',10.0)); self.wheel_rate=float(p('wheel_rate_hz',30.0)); self.scan_period=float(p('lidar_scan_period_sec',0.1))
        self.lidar_range=float(p('lidar_range_m',12.0)); self.lidar_noise=float(p('lidar_noise_m',0.0)); self.imu_noise=float(p('imu_noise',0.0)); self.imu_bias=float(p('imu_bias_rad_s',0.0)); self.gnss_noise=float(p('gnss_noise_m',0.0)); self.gnss_dropout=float(p('gnss_dropout_probability',0.0)); self.lidar_dropout=bool(p('lidar_dropout',False)); self.camera_loss=float(p('camera_frame_loss_probability',0.0)); self.encoder_noise=float(p('encoder_noise_rad_s',0.0)); self.timestamp_offset=float(p('timestamp_offset_sec',0.0)); self.jitter=float(p('timestamp_jitter_sec',0.0))
        self.lat0=float(p('latitude_reference_deg',37.4275)); self.lon0=float(p('longitude_reference_deg',-122.1697)); self.width=int(p('image_width',160)); self.height=int(p('image_height',120)); self.fx=float(p('camera_fx_px',120.0)); self.baseline=float(p('camera_baseline_m',0.12)); self.radius=float(p('wheel_radius_m',0.15)); self.track=float(p('track_width_m',0.60)); self.resolution=int(p('encoder_resolution',2048))
        for n,v in [('lidar_rate',self.lidar_rate),('imu_rate',self.imu_rate),('gnss_rate',self.gnss_rate),('camera_rate',self.camera_rate),('wheel_rate',self.wheel_rate),('scan_period',self.scan_period),('wheel_radius',self.radius),('track_width',self.track),('baseline',self.baseline)]: require_positive(n,v)
        if not 0<=self.gnss_dropout<=1 or not 0<=self.camera_loss<=1: raise ValueError('dropout probabilities must be [0,1]')
        self.start=time.monotonic(); self.ros_start=self.get_clock().now().nanoseconds/1e9; self.angles=[0.0]*4
        self.pub={k:self.create_publisher(t, self.ns+s, 10) for k,t,s in [('lidar',PointCloud2,'/lidar/points'),('imu',Imu,'/imu/data'),('gnss',NavSatFix,'/gnss/fix'),('wheels',JointState,'/wheel_states'),('left',Image,'/stereo/left/image_raw'),('right',Image,'/stereo/right/image_raw'),('left_info',CameraInfo,'/stereo/left/camera_info'),('right_info',CameraInfo,'/stereo/right/camera_info'),('truth',PoseStamped,'/evaluation/ground_truth')]}
        self.create_timer(1/self.imu_rate,self.imu); self.create_timer(1/self.lidar_rate,self.lidar); self.create_timer(1/self.gnss_rate,self.gnss); self.create_timer(1/self.camera_rate,self.camera); self.create_timer(1/self.wheel_rate,self.wheels); self.create_timer(0.05,self.truth)
    def elapsed(self): return max(0.0,time.monotonic()-self.start)
    def stamp(self, elapsed=None):
        if elapsed is None: elapsed=self.elapsed()
        now=self.ros_start+elapsed+self.timestamp_offset+self.rng.uniform(-self.jitter,self.jitter)
        from builtin_interfaces.msg import Time
        s=Time(); s.sec=int(now); s.nanosec=int((now-s.sec)*1e9); return s
    def state(self, elapsed=None): return trajectory(self.elapsed() if elapsed is None else elapsed)
    def truth(self):
        t=self.elapsed(); x,y,yaw,_,_=self.state(t); m=PoseStamped(); m.header.stamp=self.stamp(t); m.header.frame_id='map'; m.pose.position.x=x;m.pose.position.y=y;m.pose.orientation.z=math.sin(yaw/2);m.pose.orientation.w=math.cos(yaw/2);self.pub['truth'].publish(m)
    def lidar(self):
        if self.get_parameter('lidar_dropout').value:return
        end=self.elapsed(); start=max(0.0,end-self.scan_period)
        # Do not fabricate a full scan before enough simulated time has elapsed.
        if end-start < self.scan_period*0.99:return
        points=scan_point_cloud(start,self.scan_period,self.lidar_range,self.rng,self.lidar_noise)
        m=PointCloud2();m.header.stamp=self.stamp(start);m.header.frame_id='lidar_link';m.height=1;m.width=len(points);m.fields=[PointField(name=n,offset=o,datatype=PointField.FLOAT32,count=1) for n,o in (('x',0),('y',4),('z',8),('intensity',12),('time',16))];m.is_bigendian=False;m.point_step=20;m.row_step=20*len(points);m.is_dense=True;m.data=b''.join(struct.pack('<fffff',*xyz,1.0,offset) for xyz,offset in points);self.pub['lidar'].publish(m)
    def imu(self):
        t=self.elapsed(); _,_,_,v,w=self.state(t);m=Imu();m.header.stamp=self.stamp(t);m.header.frame_id='imu_link';m.orientation_covariance[0]=-1.0;m.angular_velocity.z=w+self.imu_bias+self.rng.gauss(0,self.imu_noise);m.linear_acceleration.y=v*w+self.rng.gauss(0,self.imu_noise);m.linear_acceleration.z=9.80665+self.rng.gauss(0,self.imu_noise);m.angular_velocity_covariance[8]=self.imu_noise**2;m.linear_acceleration_covariance[0]=m.linear_acceleration_covariance[4]=m.linear_acceleration_covariance[8]=self.imu_noise**2;self.pub['imu'].publish(m)
    def gnss(self):
        if self.rng.random()<self.gnss_dropout:return
        x,y,_,_,_=self.state(); x+=self.rng.gauss(0,self.gnss_noise);y+=self.rng.gauss(0,self.gnss_noise);lat,lon=local_to_geodetic(x,y,self.lat0,self.lon0);m=NavSatFix();m.header.stamp=self.stamp();m.header.frame_id='gnss_link';m.status.status=NavSatStatus.STATUS_FIX;m.latitude=lat;m.longitude=lon;m.altitude=0.0;m.position_covariance[0]=m.position_covariance[4]=self.gnss_noise**2;m.position_covariance[8]=4*self.gnss_noise**2;m.position_covariance_type=NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN;self.pub['gnss'].publish(m)
    def wheels(self):
        _,_,_,v,w=self.state();l,r=wheel_speeds(v,w,self.track,self.radius); dt=1/self.wheel_rate; vs=[l,l,r,r];m=JointState();m.header.stamp=self.stamp();m.name=WHEELS;m.velocity=[q+self.rng.gauss(0,self.encoder_noise) for q in vs];self.angles=[a+q*dt for a,q in zip(self.angles,vs)];m.position=[encoder_ticks(a,self.resolution)*2*math.pi/self.resolution for a in self.angles];self.pub['wheels'].publish(m)
    def camera(self):
        if self.rng.random()<self.camera_loss:return
        stamp=self.stamp(); disp=max(1,int(self.fx*self.baseline/3.0));l,r=stereo_pixels(self.width,self.height,disp)
        for key,data,frame in [('left',l,'stereo_left_optical_frame'),('right',r,'stereo_right_optical_frame')]:
            m=Image();m.header.stamp=stamp;m.header.frame_id=frame;m.height=self.height;m.width=self.width;m.encoding='mono8';m.step=self.width;m.data=data;self.pub[key].publish(m)
        for key,frame,tx in [('left_info','stereo_left_optical_frame',0.0),('right_info','stereo_right_optical_frame',-self.fx*self.baseline)]:
            m=CameraInfo();m.header.stamp=stamp;m.header.frame_id=frame;m.width=self.width;m.height=self.height;m.k=[self.fx,0.0,float(self.width)/2,0.0,self.fx,float(self.height)/2,0.0,0.0,1.0];m.p=[self.fx,0.0,float(self.width)/2,tx,0.0,self.fx,float(self.height)/2,0.0,0.0,0.0,1.0,0.0];self.pub[key].publish(m)
def main():
 rclpy.init();n=SensorSimulator()
 try:rclpy.spin(n)
 finally:n.destroy_node();rclpy.try_shutdown()
