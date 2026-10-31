import math, random
import pytest
from atlas_sensor_sim.model import *
def test_geodetic_and_wheels():
 lat,lon=local_to_geodetic(10,20,0,0); assert lat>0 and lon>0
 l,r=wheel_speeds(1,0.2,.6,.15); assert r>l; assert encoder_ticks(2*math.pi,2048)==2048
def test_stereo_disparity_and_cloud_validity():
 left,right=stereo_pixels(80,60,4); assert left != right and left[20*80+30] == right[20*80+26]
 pts=cloud_points(0,0,0,10,random.Random(42),.01); assert len(pts)>500 and all(math.isfinite(v) for p in pts for v in p)
def test_reproducibility_and_validation():
 assert cloud_points(0,0,0,10,random.Random(7),.1)==cloud_points(0,0,0,10,random.Random(7),.1)
 with pytest.raises(ValueError): wheel_speeds(1,0,0,.1)
 with pytest.raises(ValueError): stereo_pixels(8,10,2)
def test_cloud_changes_with_acquisition_time():
    """A moving sensor must observe different geometry over a scan."""

    rng = random.Random(42)

    scan_start = 2.0
    scan_duration = 0.1

    x0, y0, yaw0, _, _ = trajectory(scan_start)
    x1, y1, yaw1, _, _ = trajectory(scan_start + scan_duration)

    points_start = cloud_points(
        x0, y0, yaw0, 12.0, rng, 0.0
    )

    points_end = cloud_points(
        x1, y1, yaw1, 12.0, rng, 0.0
    )

    assert points_start != points_end


def test_scan_offsets_are_ordered_bounded_and_match_acquisition_pose():
    scan_start,period=2.0,.1
    events=scan_point_cloud(scan_start,period,12.0,random.Random(8),0.0)
    offsets=[offset for _,offset in events]
    assert offsets==sorted(offsets) and offsets[0]>=0 and offsets[-1]<period
    scan_header_time=100.0
    publish_time=scan_header_time+period
    assert all(scan_header_time+offset<=publish_time for offset in offsets)
    landmarks=[]
    for q in range(-24,25,2):
        for z in (0.0,0.5,1.2,2.0):landmarks.extend(((6.0,q/4,z),(-6.0,q/4,z),(q/4,6.0,z),(q/4,-6.0,z)))
    for z in [i/5 for i in range(11)]:
        for a in range(16):landmarks.append((1.4+.35*math.cos(a*math.pi/8),1.0+.35*math.sin(a*math.pi/8),z))
    landmarks.extend((-2.0,q/5,1.8) for q in range(-10,11))
    for point,offset in events[::max(1,len(events)//20)]:
        x,y,yaw,_,_=trajectory(scan_start+offset);c,s=math.cos(yaw),math.sin(yaw)
        world=(x+c*point[0]-s*point[1],y+s*point[0]+c*point[1],point[2])
        assert min(math.dist(world,lm) for lm in landmarks)<1e-8


def test_scan_geometry_depends_on_vehicle_motion_but_static_world():
    a=scan_point_cloud(1.0,.1,12.0,random.Random(42),0.0)
    b=scan_point_cloud(2.0,.1,12.0,random.Random(42),0.0)
    assert a!=b
    assert all(0<=offset<=.1 for _,offset in a+b)
