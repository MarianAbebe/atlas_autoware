import math
def finite_pose(p): return all(math.isfinite(v) for v in p)
def yaw_from_quaternion(z,w): return math.atan2(2*w*z, 1-2*z*z)
def angle_error(a,b): return abs(math.atan2(math.sin(a-b),math.cos(a-b)))
def pose_error(estimate, truth):
    """Return XY, Z, 3D and yaw error for evaluator-only pose tuples."""
    if len(estimate)==3 and len(truth)==3:  # retained legacy helper contract
        ex,ey,eyaw=estimate;tx,ty,tyaw=truth
        return math.hypot(ex-tx,ey-ty),angle_error(eyaw,tyaw)
    ex,ey,ez,eyaw=estimate; tx,ty,tz,tyaw=truth
    xy=math.hypot(ex-tx,ey-ty); z=abs(ez-tz)
    return xy,z,math.hypot(xy,z),angle_error(eyaw,tyaw)
