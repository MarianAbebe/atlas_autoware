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
    """Sparse rays against a 6 m square and two fixed pillars."""
    require_positive('lidar range', max_range)
    points=[]
    for i in range(48):
        a = yaw + 2*math.pi*i/48
        # deterministic radial synthetic environment, provides 3D structure
        r = min(max_range, 3.0 + 0.7*math.sin(3*a) + 0.25*math.cos(7*a))
        for z in (-0.15, 0.2, 0.55):
            rr = max(0.1, r + rng.gauss(0, noise))
            points.append((x + rr*math.cos(a), y + rr*math.sin(a), z + rng.gauss(0, noise)))
    return points

def stereo_pixels(width, height, disparity):
    if width <= 8 or height <= 8 or disparity <= 0 or disparity >= width: raise ValueError('invalid stereo dimensions/disparity')
    left=bytearray(width*height); right=bytearray(width*height)
    for v in range(height):
        for u in range(width):
            value = (u*7 + v*13 + ((u//12 + v//10) % 2)*80) % 256
            left[v*width+u]=value
            if u >= disparity: right[v*width+u-disparity]=value
    return bytes(left), bytes(right)
