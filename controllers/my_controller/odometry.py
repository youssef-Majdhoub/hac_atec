import math


class Odometry:
    def __init__(self, base_link, left_encoders, right_encoders, wheel_radius):
        self.base = base_link
        self.left = left_encoders
        self.right = right_encoders
        self.r = wheel_radius
        self.prev_l = None
        self.prev_r = None

    @staticmethod
    def _mean(encoders):
        return sum(e.getValue() for e in encoders) / len(encoders)

    def update(self, roll, pitch, yaw):
        self.base.roll, self.base.pitch, self.base.yaw = roll, pitch, yaw
        l = self._mean(self.left)
        r = self._mean(self.right)
        if self.prev_l is None:
            self.prev_l, self.prev_r = l, r
            return
        dist = self.r * ((l - self.prev_l) + (r - self.prev_r)) / 2.0
        self.prev_l, self.prev_r = l, r
        horiz = dist * math.cos(pitch)
        self.base.x += horiz * math.cos(yaw)
        self.base.y += horiz * math.sin(yaw)
