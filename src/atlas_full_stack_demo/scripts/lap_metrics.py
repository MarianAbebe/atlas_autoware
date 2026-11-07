"""Pure-Python geometric metrics for a closed route."""
import math


def prepare_route(points, require_closed=True, closure_tolerance_m=0.25):
    if len(points) < 3:
        raise ValueError('closed route needs at least three points')
    xy = [(float(p[0]), float(p[1])) for p in points]
    lengths = [math.dist(a, b) for a, b in zip(xy, xy[1:])]
    if any(not math.isfinite(v) or v <= 1e-6 for v in lengths):
        raise ValueError('route has invalid or duplicate consecutive points')
    cumulative = [0.0]
    for length in lengths:
        cumulative.append(cumulative[-1] + length)
    if require_closed and math.dist(xy[0], xy[-1]) > closure_tolerance_m:
        raise ValueError('route endpoints are not closed within 0.25 m')
    return xy, lengths, cumulative, cumulative[-1]


def extract_closed_prefix(points, route_length_m, closure_tolerance_m=0.25):
    """Return the first route_length_m of an open route, including its end.

    The synthetic Lanelet mission includes a short terminal spur so its goal
    differs from the start. Lap metrics project only onto the closed prefix.
    """
    xy = [(float(p[0]), float(p[1])) for p in points]
    if len(xy) < 3 or not math.isfinite(route_length_m) or route_length_m <= 0:
        raise ValueError('route prefix requires finite points and positive length')
    lengths = [math.dist(a, b) for a, b in zip(xy, xy[1:])]
    if any(not math.isfinite(v) or v <= 1e-6 for v in lengths):
        raise ValueError('route has invalid or duplicate consecutive points')
    cumulative = [0.0]
    for length in lengths:
        cumulative.append(cumulative[-1] + length)
    if route_length_m > cumulative[-1] + 1e-6:
        raise ValueError('closed prefix length exceeds route coverage')
    prefix = [p for p, s in zip(xy, cumulative) if s < route_length_m - 1e-9]
    index = next((i for i, s in enumerate(cumulative[1:], 1) if s >= route_length_m), None)
    if index is None:
        raise ValueError('closed prefix endpoint is unavailable')
    span = cumulative[index] - cumulative[index - 1]
    ratio = (route_length_m - cumulative[index - 1]) / span
    a, b = xy[index - 1], xy[index]
    endpoint = (a[0] + ratio * (b[0] - a[0]), a[1] + ratio * (b[1] - a[1]))
    prefix.append(endpoint)
    if math.dist(prefix[0], prefix[-1]) > closure_tolerance_m:
        raise ValueError('route prefix does not close within tolerance')
    return prefix


def extract_loop_prefix(points, route_length_m, maximum_seam_gap_m=8.5):
    """Extract the single-pass loop mission path whose endpoints are distinct.

    A small physical connector joins the finish branch back to the start for
    later laps. Keeping that connector outside this prefix avoids duplicating
    the exact start pose at the mission terminal, which confused global
    nearest-point tracking in the prior run.
    """
    if not math.isfinite(maximum_seam_gap_m) or maximum_seam_gap_m <= 0.0:
        raise ValueError('maximum seam gap must be positive and finite')
    xy = [(float(p[0]), float(p[1])) for p in points]
    if len(xy) < 3 or not math.isfinite(route_length_m) or route_length_m <= 0.0:
        raise ValueError('loop prefix requires finite points and positive length')
    lengths = [math.dist(a, b) for a, b in zip(xy, xy[1:])]
    if any(not math.isfinite(v) or v <= 1e-6 for v in lengths):
        raise ValueError('route has invalid or duplicate consecutive points')
    cumulative = [0.0]
    for length in lengths:
        cumulative.append(cumulative[-1] + length)
    if route_length_m > cumulative[-1] + 1e-6:
        raise ValueError('loop prefix length exceeds route coverage')
    prefix = [p for p, s in zip(xy, cumulative) if s < route_length_m - 1e-9]
    index = next((i for i, s in enumerate(cumulative[1:], 1) if s >= route_length_m), None)
    if index is None:
        raise ValueError('loop prefix endpoint is unavailable')
    ratio = (route_length_m-cumulative[index-1])/lengths[index-1]
    a, b = xy[index-1], xy[index]
    prefix.append((a[0]+ratio*(b[0]-a[0]), a[1]+ratio*(b[1]-a[1])))
    seam_gap = math.dist(prefix[0], prefix[-1])
    if seam_gap <= 1e-6 or seam_gap > maximum_seam_gap_m:
        raise ValueError(f'loop seam gap {seam_gap:.3f} m is invalid')
    return prefix


def project_pose(x, y, route, lengths, cumulative):
    best = None
    for i, (a, b) in enumerate(zip(route, route[1:])):
        dx, dy = b[0] - a[0], b[1] - a[1]
        t = max(0.0, min(1.0, ((x-a[0])*dx + (y-a[1])*dy) / (lengths[i]**2)))
        px, py = a[0] + t*dx, a[1] + t*dy
        distance = math.hypot(x-px, y-py)
        yaw = math.atan2(dy, dx)
        if best is None or distance < best[0]:
            best = (distance, cumulative[i] + t*lengths[i], yaw)
    return best


def unwrap_progress(previous_s, current_s, route_length):
    delta = current_s - previous_s
    if delta < -0.5 * route_length:
        delta += route_length
    elif delta > 0.5 * route_length:
        delta -= route_length
    return delta


def angle_error(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


def nearest_trajectory_reference(points, position_xy, ahead_count=3):
    """Summarize the closest controller reference and following points.

    ``points`` is an iterable of trajectory points with ``pose.position``,
    ``pose.orientation`` and ``longitudinal_velocity_mps`` fields. This is a
    diagnostic only: it does not change trajectory selection or control.
    """
    if not points or len(position_xy) < 2 or ahead_count < 0:
        return None
    x, y = float(position_xy[0]), float(position_xy[1])
    if not math.isfinite(x) or not math.isfinite(y):
        return None
    best_index, best_distance_sq = None, math.inf
    for index, point in enumerate(points):
        px, py = float(point.pose.position.x), float(point.pose.position.y)
        distance_sq = (px-x)**2 + (py-y)**2
        if math.isfinite(distance_sq) and distance_sq < best_distance_sq:
            best_index, best_distance_sq = index, distance_sq
    if best_index is None:
        return None
    selected = []
    for point in points[best_index:min(len(points), best_index+ahead_count+1)]:
        pose = point.pose
        q = pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z+q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z))
        values = (pose.position.x, pose.position.y, yaw, point.longitudinal_velocity_mps)
        if not all(math.isfinite(float(value)) for value in values):
            return None
        selected.append({
            'position': [float(pose.position.x), float(pose.position.y)],
            'yaw_rad': yaw,
            'velocity_mps': float(point.longitudinal_velocity_mps),
        })
    return {
        'nearest_index': best_index,
        'distance_m': math.sqrt(best_distance_sq),
        'points': selected,
    }


def terminal_goal_stopped(position_xy, speed_xy, goal_xyz, distance_tolerance_m=0.75,
                          speed_tolerance_mps=0.1):
    """Check terminal position and planar speed; lap progress is separate."""
    return (math.dist(position_xy, goal_xyz[:2]) <= distance_tolerance_m and
            math.isfinite(speed_xy) and speed_xy <= speed_tolerance_mps)
