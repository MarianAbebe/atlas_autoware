import math
def finite_pose(p): return all(math.isfinite(v) for v in p)
def yaw_from_quaternion(z,w): return math.atan2(2*w*z, 1-2*z*z)
def angle_error(a,b): return abs(math.atan2(math.sin(a-b),math.cos(a-b)))
def pose_error(estimate, truth):
    """Planar error used only by offline/evaluation process, never estimator input."""
    ex,ey,eyaw=estimate; tx,ty,tyaw=truth
    return math.hypot(ex-tx,ey-ty),angle_error(eyaw,tyaw)
