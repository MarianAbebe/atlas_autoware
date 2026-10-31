import math
import pytest
from atlas_lio_integration.metrics import pose_error,angle_error,finite_pose
def test_pose_error():
 assert pose_error((1,2,.2),(1,2,.2))==(0.0,0.0)
 assert abs(angle_error(math.pi-.1,-math.pi+.1)-.2)<1e-9
 assert finite_pose((0,1,2)) and not finite_pose((0,float('nan'),2))


def test_evaluator_accepts_accurate_localization():
    from atlas_lio_integration.evaluator import Evaluator

    evaluator = Evaluator.__new__(Evaluator)
    evaluator.errors = [(0.14, 0.08)] * 442
    evaluator.odom_times = [i / 30 for i in range(442)]

    result = evaluator.result()

    assert result["result"] == "PASS"


def test_evaluator_rejects_large_localization_error():
    from atlas_lio_integration.evaluator import Evaluator

    evaluator = Evaluator.__new__(Evaluator)
    evaluator.errors = [(52.39, 1.28)] * 442
    evaluator.odom_times = [i / 30 for i in range(442)]

    result = evaluator.result()

    assert result["result"] == "FAIL"


def test_diagnostic_status_constants_are_bytes():
    """ROS 2 Humble diagnostic levels must not be compared with integers."""
    from diagnostic_msgs.msg import DiagnosticStatus

    status = DiagnosticStatus()
    status.level = DiagnosticStatus.OK

    assert status.level == DiagnosticStatus.OK
    assert status.level != 0

    status.level = DiagnosticStatus.ERROR

    assert status.level == DiagnosticStatus.ERROR
    assert status.level != 2


def test_evaluator_accepts_fresh_ground_truth():
    from atlas_lio_integration.evaluator import Evaluator
    from nav_msgs.msg import Odometry

    evaluator = Evaluator.__new__(Evaluator)
    evaluator.truth = [(9.98, 1.0, 2.0, 0.0, 0.0), (10.02, 1.0, 2.0, 0.0, 0.0)]
    evaluator.errors = []
    evaluator.odom_times = []
    evaluator.samples = []
    evaluator.alignment = None
    evaluator.max_bracket = .11
    evaluator.estimate_frame = 'odom'
    evaluator.truth_frame = 'map'
    evaluator.invalid_estimates = evaluator.invalid_truth = evaluator.frame_rejections = evaluator.rejected_matches = 0

    msg = Odometry()
    msg.header.stamp.sec = 10
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_link'
    msg.pose.pose.position.x = 1.0
    msg.pose.pose.position.y = 2.0
    msg.pose.pose.orientation.w = 1.0

    evaluator.odom(msg)

    assert len(evaluator.errors) == 1
    assert evaluator.errors[0][0] == 0.0


def test_evaluator_rejects_stale_ground_truth():
    from atlas_lio_integration.evaluator import Evaluator
    from nav_msgs.msg import Odometry

    evaluator = Evaluator.__new__(Evaluator)
    evaluator.truth = [(10.20, 1.0, 2.0, 0.0, 0.0), (10.25, 1.0, 2.0, 0.0, 0.0)]
    evaluator.errors = []
    evaluator.odom_times = []
    evaluator.rejected_matches = 0
    evaluator.samples = []
    evaluator.max_bracket = .11
    evaluator.estimate_frame = 'odom'

    msg = Odometry()
    msg.header.stamp.sec = 10
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_link'
    msg.pose.pose.orientation.w = 1.0

    evaluator.odom(msg)

    assert evaluator.errors == []
    assert evaluator.odom_times == []
    assert evaluator.rejected_matches == 1


def test_evaluator_accepts_timestamp_within_tolerance():
    from atlas_lio_integration.evaluator import Evaluator
    from nav_msgs.msg import Odometry

    evaluator = Evaluator.__new__(Evaluator)
    evaluator.truth = [(10.0, 0.0, 0.0, 0.0, 0.0), (10.049, 0.0, 0.0, 0.0, 0.0)]
    evaluator.errors = []
    evaluator.odom_times = []
    evaluator.samples = []
    evaluator.alignment = None
    evaluator.max_bracket = .11
    evaluator.estimate_frame = 'odom'
    evaluator.truth_frame = 'map'
    evaluator.invalid_estimates = evaluator.invalid_truth = evaluator.frame_rejections = evaluator.rejected_matches = 0

    msg = Odometry()
    msg.header.stamp.sec = 10
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_link'
    msg.pose.pose.orientation.w = 1.0

    evaluator.odom(msg)

    assert len(evaluator.errors) == 1


def _configured_evaluator():
    from atlas_lio_integration.evaluator import Evaluator
    e=Evaluator.__new__(Evaluator);e.truth=[];e.samples=[];e.errors=[];e.odom_times=[];e.rejected_matches=0;e.invalid_estimates=0;e.invalid_truth=0;e.frame_rejections=0;e.timestamp_rejections={'invalid':0,'duplicate':0,'regressing':0};e.last_truth_stamp=None;e.last_estimate_stamp=None;e.alignment=None;e.max_bracket=.11;e.truth_frame='map';e.estimate_frame='odom';e.estimate_child_frame='base_link'
    return e


def _odom(t,x,y,z,yaw=0.0,frame='odom'):
    from nav_msgs.msg import Odometry
    m=Odometry();m.header.stamp.sec=int(t);m.header.stamp.nanosec=int((t-int(t))*1e9);m.header.frame_id=frame;m.child_frame_id='base_link';m.pose.pose.position.x=float(x);m.pose.pose.position.y=float(y);m.pose.pose.position.z=float(z);m.pose.pose.orientation.z=float(math.sin(yaw/2));m.pose.pose.orientation.w=float(math.cos(yaw/2));return m


def test_ground_truth_interpolation_and_initial_frame_alignment():
    from geometry_msgs.msg import PoseStamped
    from atlas_lio_integration.evaluator import interpolate_truth
    truth=[(1.0,10.0,2.0,1.0,math.radians(179)),(1.1,10.0,3.0,2.0,math.radians(-179))]
    interp=interpolate_truth(truth,1.05)
    assert interp[0]==(10.0,2.5,1.5)
    assert abs(abs(2*math.atan2(interp[1][2],interp[1][3]))-math.pi)<.02
    e=_configured_evaluator()
    for t,x,y,z,yaw in [(1.0,10,2,1,math.radians(179)),(1.1,10,3,2,math.radians(-179))]:
        m=PoseStamped();m.header.frame_id='map';m.header.stamp.sec=int(t);m.header.stamp.nanosec=int((t-int(t))*1e9);m.pose.position.x=float(x);m.pose.position.y=float(y);m.pose.position.z=float(z);m.pose.orientation.z=float(math.sin(yaw/2));m.pose.orientation.w=float(math.cos(yaw/2));e.gt(m)
    # Synthetic estimator frame is translated and yaw-rotated relative to map.
    e.odom(_odom(1.05,2,1,.5,math.pi/2))
    assert e.samples and e.samples[0][7][2] < 1e-8


def test_evaluator_reports_3d_relative_metrics_and_no_absolute_claim():
    e=_configured_evaluator()
    for t in (1.0,1.1,1.2):e.truth.append((t,t,2*t,0.2*t,0.1*t))
    e.odom(_odom(1.05,0,0,0,0));e.odom(_odom(1.15,.1,.2,.04,.01))
    result=e.result()
    assert result['accuracy_mode']=='fixed_initial_pose_se3_aligned'
    assert all(math.isfinite(result[k]) for k in ('xy_rmse_m','z_rmse_m','3d_rmse_m','yaw_rmse_rad'))
    assert result['xy_rmse_m']>0 and result['z_rmse_m']>0 and result['3d_rmse_m']>result['xy_rmse_m']
    assert result['relative_translation_error_rmse_m'] != result['3d_rmse_m']


def test_evaluator_rejects_nonfinite_and_wrong_frame():
    e=_configured_evaluator();e.truth=[(1.0,0,0,0,0),(1.1,0,0,0,0)]
    e.odom(_odom(1.05,float('nan'),0,0));e.odom(_odom(1.05,0,0,0,frame='map'))
    assert e.invalid_estimates==1 and e.frame_rejections==1 and not e.samples


def test_evaluator_rejects_invalid_duplicate_and_regressing_estimator_stamps():
    e = _configured_evaluator()
    e.truth = [(1.0,0.,0.,0.,0.), (1.1,0.1,0.,0.,0.), (1.2,0.2,0.,0.,0.)]
    first = _odom(1.05,0.,0.,0.)
    e.odom(first)
    e.odom(_odom(1.05,0.,0.,0.))
    e.odom(_odom(1.04,0.,0.,0.))
    invalid = _odom(1.15,0.15,0.,0.)
    invalid.header.stamp.nanosec = 1_000_000_000
    e.odom(invalid)
    assert len(e.samples) == 1
    assert e.timestamp_rejections == {'invalid':1,'duplicate':1,'regressing':1}
    assert e.result()['match_rate'] == 1.0


def test_interpolation_rejects_extrapolation_and_wide_sync_gap():
    from atlas_lio_integration.evaluator import interpolate_truth
    assert interpolate_truth([(1,0,0,0,0),(1.2,1,1,1,0)],1.1,.11) is None
    assert interpolate_truth([(1,0,0,0,0),(1.1,1,1,1,0)],.99) is None


def test_quaternion_slerp_interpolates_full_3d_orientation():
    from atlas_lio_integration.evaluator import interpolate_truth
    half_angle = math.pi / 4.0
    samples = [(1.0, (0.,0.,0.), (0.,0.,0.,1.)),
               (1.1, (0.,0.,0.), (math.sin(half_angle),0.,0.,math.cos(half_angle)))]
    position, orientation = interpolate_truth(samples, 1.05)
    assert position == (0.,0.,0.)
    assert orientation[0] == pytest.approx(math.sin(math.pi/8.0))
    assert orientation[3] == pytest.approx(math.cos(math.pi/8.0))


def test_initial_se3_alignment_keeps_injected_xyz_and_yaw_drift_visible():
    e = _configured_evaluator()
    e.truth = [(1.0,1.0,0.0,0.0,0.0), (1.1,1.1,0.0,0.0,0.0),
               (1.2,1.2,0.0,0.0,0.0)]
    e.odom(_odom(1.05,4.0,2.0,0.0,0.0))
    e.odom(_odom(1.15,4.08,2.0,0.03,0.04))
    result = e.result()
    assert result['xy_rmse_m'] > 0.0
    assert result['z_rmse_m'] == pytest.approx(0.03/math.sqrt(2.0))
    assert result['yaw_rmse_rad'] == pytest.approx(0.04/math.sqrt(2.0))
    assert result['relative_translation_error_rmse_m'] == pytest.approx(math.sqrt(.02**2+.03**2))
    assert result['relative_rotation_error_rmse_rad'] == pytest.approx(0.04)


def test_evaluator_accepts_high_match_rate():
    from atlas_lio_integration.evaluator import Evaluator

    evaluator = Evaluator.__new__(Evaluator)
    evaluator.errors = [(0.14, 0.08)] * 190
    evaluator.odom_times = [i / 30 for i in range(190)]
    evaluator.rejected_matches = 10

    result = evaluator.result()

    assert result["result"] == "PASS"
    assert result["match_rate"] == 0.95
    assert result["rejected_matches"] == 10


def test_evaluator_rejects_low_match_rate():
    from atlas_lio_integration.evaluator import Evaluator

    evaluator = Evaluator.__new__(Evaluator)
    evaluator.errors = [(0.14, 0.08)] * 150
    evaluator.odom_times = [i / 30 for i in range(150)]
    evaluator.rejected_matches = 50

    result = evaluator.result()

    assert result["result"] == "FAIL"
    assert result["match_rate"] == 0.75
    assert result["rejected_matches"] == 50
