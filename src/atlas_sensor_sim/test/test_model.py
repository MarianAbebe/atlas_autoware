import math, random
import pytest
from atlas_sensor_sim.model import *
def test_geodetic_and_wheels():
 lat,lon=local_to_geodetic(10,20,0,0); assert lat>0 and lon>0
 l,r=wheel_speeds(1,0.2,.6,.15); assert r>l; assert encoder_ticks(2*math.pi,2048)==2048
def test_stereo_disparity_and_cloud_validity():
 left,right=stereo_pixels(80,60,4); assert left != right and left[20*80+30] == right[20*80+26]
 pts=cloud_points(0,0,0,10,random.Random(42),.01); assert len(pts)==144 and all(math.isfinite(v) for p in pts for v in p)
def test_reproducibility_and_validation():
 assert cloud_points(0,0,0,10,random.Random(7),.1)==cloud_points(0,0,0,10,random.Random(7),.1)
 with pytest.raises(ValueError): wheel_speeds(1,0,0,.1)
 with pytest.raises(ValueError): stereo_pixels(8,10,2)
