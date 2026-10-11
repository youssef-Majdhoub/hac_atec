import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Mount:
    """Sensor offset from base_link, in meters (x forward, y left, z up)."""

    x: float
    y: float
    z: float


class BaseLink:
    """Robot reference frame. Its pose lives in the odom frame."""

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def sensor_to_base(self, p, mount):
        # Sensors are assumed aligned with base_link axes (no rotation).
        return (p[0] + mount.x, p[1] + mount.y, p[2] + mount.z)

    def base_to_odom(self, p):
        # 2D rotation by yaw plus translation. Roll and pitch are ignored.
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (self.x + c * p[0] - s * p[1], self.y + s * p[0] + c * p[1], p[2])

    def pose(self):
        return (self.x, self.y, self.yaw)
