"""Simulation-only, bounded command provenance and explicit completion barrier.

Recording precedes forwarding in the relay callback. Pulling a snapshot avoids
losing provenance when the independent validator's DDS reader falls behind.
It never waits in the actuation path. Overflow fails validation, not actuation.
"""
import json
import time
from std_srvs.srv import Trigger


def payload(msg):
    return [msg.stamp.sec + msg.stamp.nanosec * 1e-9,
            msg.lateral.steering_tire_angle, msg.longitudinal.velocity,
            msg.longitudinal.acceleration]


class CommandJournal:
    def __init__(self, capacity=20000):
        self.capacity = capacity
        self.rows = []
        self.overflow = False

    def record(self, msg):
        if len(self.rows) >= self.capacity:
            self.overflow = True
        else:
            self.rows.append([time.monotonic(), *payload(msg)])

    def snapshot(self, request, response):
        response.success = not self.overflow
        response.message = json.dumps(dict(rows=self.rows, overflow=self.overflow))
        return response


def recording_barrier(node, client, delivered, timeout=5.):
    """Wait for the service response callback and all measured delivered payloads.

    Caller closes physical measurement first. Only provenance is fetched here;
    cleanup/extra callbacks cannot extend driving or stopped-hold evidence.
    """
    expected = {tuple(row[1:]) for row in delivered}
    deadline = time.monotonic() + timeout
    import rclpy
    while time.monotonic() < deadline:
        if not client.service_is_ready():
            rclpy.spin_once(node, timeout_sec=min(.02, max(0., deadline-time.monotonic())))
            continue
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=max(0., deadline-time.monotonic()))
        if not future.done() or future.result() is None:
            break
        response = future.result()
        data = json.loads(response.message)
        if not response.success or data['overflow']:
            return dict(passed=False, reason='journal_overflow', rows=data['rows'])
        if expected <= {tuple(row[1:]) for row in data['rows']}:
            return dict(passed=True, reason='all_delivered_recorded', rows=data['rows'])
        # A delivered payload must already have been recorded before forwarding.
        # A completed snapshot missing it is a provenance failure, not a race.
        return dict(passed=False, reason='delivered_payload_absent', rows=data['rows'])
    return dict(passed=False, reason='recording_barrier_timeout', rows=[])
