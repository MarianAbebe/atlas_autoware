"""Pure deterministic geometry and measurement helpers used by nodes and tests."""
import math
import random

WHEELS = ['front_left_wheel', 'rear_left_wheel', 'front_right_wheel', 'rear_right_wheel']

def require_positive(name, value):
    if not math.isfinite(value) or value <= 0:
        raise ValueError('%s must be finite and positive' % name)

def local_to_geodetic(x_m, y_m, lat0_deg, lon0_deg):
    """Local ENU approximation, suitable only for small synthetic areas."""
    require_positive('earth radius', 6378137.0)
    lat = lat0_deg + math.degrees(y_m / 6378137.0)
    lon = lon0_deg + math.degrees(x_m / (6378137.0 * math.cos(math.radians(lat0_deg))))
    return lat, lon

def trajectory(t):
    # Smooth circle: valid non-zero angular acceleration/velocity inputs.
    radius, speed = 2.0, 0.35
    omega = speed / radius
    return radius * math.sin(omega*t), radius * (1-math.cos(omega*t)), omega*t, speed, omega

def wheel_speeds(linear, angular, track, radius):
    require_positive('track width', track); require_positive('wheel radius', radius)
    return ((linear-angular*track/2)/radius, (linear+angular*track/2)/radius)

def encoder_ticks(angle_rad, resolution):
    if int(resolution) <= 0: raise ValueError('encoder resolution must be positive')
    return round(angle_rad * int(resolution) / (2*math.pi))

def cloud_points(x, y, yaw, max_range, rng, noise):
    """Static 3-D landmarks expressed in the instantaneous LiDAR frame.

    Ordering is azimuth order, so callers can attach monotonically increasing
    per-point offsets.  Unlike the old fixture, points are not world-frame
    coordinates mislabeled as `lidar_link`.
    """
    require_positive('lidar range', max_range)
    world=[]
    # Four walls, a vertical pillar, and an elevated crossbar create non-planar
    # constraints during translation and yaw rotation.
    for q in range(-24, 25, 2):
        for z in (0.0, 0.5, 1.2, 2.0):
            world.extend(((6.0,q/4,z),(-6.0,q/4,z),(q/4,6.0,z),(q/4,-6.0,z)))
    for z in [i/5 for i in range(11)]:
        for a in range(16): world.append((1.4+.35*math.cos(a*math.pi/8), 1.0+.35*math.sin(a*math.pi/8), z))
    world.extend(((-2.0,q/5,1.8) for q in range(-10,11)))
    c,s=math.cos(yaw),math.sin(yaw); points=[]
    for wx,wy,wz in world:
        dx,dy=wx-x,wy-y; lx,ly=c*dx+s*dy,-s*dx+c*dy
        if .2 < math.hypot(lx,ly) <= max_range:
            points.append((lx+rng.gauss(0,noise),ly+rng.gauss(0,noise),wz+rng.gauss(0,noise)))
    return sorted(points, key=lambda p: math.atan2(p[1],p[0]))

def scan_points(scan_start, scan_period, max_range, rng, noise):
    """Return (LiDAR-frame XYZ, acquisition offset seconds) for one scan.

    Static landmarks are assigned acquisition time by their bearing at scan
    start. Their coordinates are then reprojected from the robot pose at that
    acquisition time. This is a deterministic first-order rotating-scan model;
    it does not model Mid-360 beam timing, divergence, occlusion, or returns.
    """
    return scan_point_cloud(scan_start,scan_period,max_range,rng,noise)

def scan_point_cloud(scan_start, scan_period, max_range, rng, noise):
    """Return acquisition-time-reprojected XYZ and offsets in seconds."""
    require_positive('scan period', scan_period)
    world=[]
    for q in range(-24,25,2):
        for z in (0.0,0.5,1.2,2.0): world.extend(((6.0,q/4,z),(-6.0,q/4,z),(q/4,6.0,z),(q/4,-6.0,z)))
    for z in [i/5 for i in range(11)]:
        for a in range(16): world.append((1.4+.35*math.cos(a*math.pi/8),1.0+.35*math.sin(a*math.pi/8),z))
    world.extend(((-2.0,q/5,1.8) for q in range(-10,11)))
    x0,y0,yaw0,_,_=trajectory(scan_start); c0,s0=math.cos(yaw0),math.sin(yaw0)
    candidates=[]
    for landmark in world:
        dx,dy=landmark[0]-x0,landmark[1]-y0; lx=c0*dx+s0*dy;ly=-s0*dx+c0*dy
        bearing=math.atan2(ly,lx)
        offset=((bearing+math.pi)%(2*math.pi))/(2*math.pi)*scan_period
        candidates.append((offset,landmark))
    result=[]
    for offset,(wx,wy,wz) in sorted(candidates):
        x,y,yaw,_,_=trajectory(scan_start+offset);dx,dy=wx-x,wy-y;c,s=math.cos(yaw),math.sin(yaw);lx=c*dx+s*dy;ly=-s*dx+c*dy
        if .2 < math.hypot(lx,ly)<=max_range:
            result.append(((lx+rng.gauss(0,noise),ly+rng.gauss(0,noise),wz+rng.gauss(0,noise)),offset))
    return result

def stereo_pixels(width, height, disparity):
    if width <= 8 or height <= 8 or disparity <= 0 or disparity >= width: raise ValueError('invalid stereo dimensions/disparity')
    left=bytearray(width*height); right=bytearray(width*height)
    for v in range(height):
        for u in range(width):
            value = (u*7 + v*13 + ((u//12 + v//10) % 2)*80) % 256
            left[v*width+u]=value
            if u >= disparity: right[v*width+u-disparity]=value
    return bytes(left), bytes(right)
