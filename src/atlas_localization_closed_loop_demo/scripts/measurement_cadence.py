"""Bound nonessential evaluator work without changing any ROS clock or input."""


class MeasurementCadence:
    def __init__(self, interval=.5):
        if interval <= 0:
            raise ValueError('interval must be positive')
        self.interval = interval
        self.next_due = interval

    def due(self, elapsed):
        if elapsed < self.next_due:
            return False
        # Skip missed samples: resource observation must never catch up in bursts.
        self.next_due = elapsed + self.interval
        return True
