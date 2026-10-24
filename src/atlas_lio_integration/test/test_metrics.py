import math
from atlas_lio_integration.metrics import pose_error,angle_error,finite_pose
def test_pose_error():
 assert pose_error((1,2,.2),(1,2,.2))==(0.0,0.0)
 assert abs(angle_error(math.pi-.1,-math.pi+.1)-.2)<1e-9
 assert finite_pose((0,1,2)) and not finite_pose((0,float('nan'),2))
