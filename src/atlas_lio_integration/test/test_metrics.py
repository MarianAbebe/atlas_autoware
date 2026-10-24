import math
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
