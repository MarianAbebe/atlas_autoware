#!/usr/bin/env python3
import math
import unittest

from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

from ego_pose_marker import make_ego_markers, marker_rejection_reason


def odom(yaw=0.0):
    msg = Odometry()
    msg.header.frame_id = 'map'
    msg.header.stamp.sec = 42
    msg.header.stamp.nanosec = 123
    msg.child_frame_id = 'base_link'
    msg.pose.pose.position.x = 4.0
    msg.pose.pose.position.y = -2.0
    msg.pose.pose.position.z = 0.1
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return msg


class EgoPoseMarkerTest(unittest.TestCase):
    def test_marker_uses_ekf_stamp_and_map_frame(self):
        marker_array = make_ego_markers(odom())
        self.assertEqual(len(marker_array.markers), 2)
        self.assertTrue(all(m.header.frame_id == 'map' for m in marker_array.markers))
        self.assertTrue(all(m.header.stamp.sec == 42 and m.header.stamp.nanosec == 123
                            for m in marker_array.markers))

    def test_body_size_and_front_indicator(self):
        body, front = make_ego_markers(odom()).markers
        self.assertEqual(body.type, Marker.CUBE)
        self.assertAlmostEqual(body.scale.x, 3.1)
        self.assertAlmostEqual(body.scale.y, 1.5)
        self.assertEqual(front.type, Marker.ARROW)
        self.assertEqual(len(front.points), 2)
        self.assertGreater(front.points[1].x, front.points[0].x)

    def test_heading_arrow_rotates_with_ekf_yaw(self):
        front = make_ego_markers(odom(math.pi / 2)).markers[1]
        self.assertAlmostEqual(front.points[1].x - front.points[0].x, 0.0, places=6)
        self.assertGreater(front.points[1].y - front.points[0].y, 1.19)

    def test_unsupported_frames_are_rejected(self):
        msg = odom()
        msg.header.frame_id = 'odom'
        self.assertIsNone(make_ego_markers(msg))
        self.assertIn('frame mismatch', marker_rejection_reason(msg))
        msg = odom()
        msg.child_frame_id = 'hidden_truth_base'
        self.assertIsNone(make_ego_markers(msg))

    def test_invalid_pose_is_rejected(self):
        msg = odom()
        msg.pose.pose.position.x = float('nan')
        self.assertIsNone(make_ego_markers(msg))
        self.assertEqual(marker_rejection_reason(msg), 'nonfinite EKF pose')
        msg = odom()
        msg.pose.pose.orientation.w = 0.0
        self.assertIsNone(make_ego_markers(msg))
        self.assertEqual(marker_rejection_reason(msg), 'zero-norm EKF orientation quaternion')


if __name__ == '__main__':
    unittest.main()
