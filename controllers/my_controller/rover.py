import math
import numpy as np
from controller import Robot, Node
from frames import BaseLink, Mount
from odometry import Odometry


class Rover:
    WHEEL_RADIUS = 0.11  # meters, verify with the GPS/odom ratio
    # Offsets from base_link. Must match the world file.
    MOUNTS = {
        "imu": Mount(0.0, 0.0, 0.0),
        "gps": Mount(0.0, 0.0, 0.0),
        "lidar": Mount(0.0, 0.0, 0.45),
        "camera": Mount(0.25, 0.0, 0.3),
    }

    def __init__(self):
        self.robot = Robot()
        self.ts = int(self.robot.getBasicTimeStep())
        self.step_count = 0
        self.base = BaseLink()

        self.camera = self._enable("camera", self.ts * 4)
        self.lidar = self._enable("lidar", self.ts * 4)
        self.lidar.enablePointCloud()
        self.imu = self._enable("imu", self.ts)
        self.accel = self._enable("accel", self.ts)
        self.gyro = self._enable("gyro", self.ts)
        self.gps = self._enable("gps", self.ts)
        self.gps0 = None

        self.encoders, self.motors = [], []
        for i in range(self.robot.getNumberOfDevices()):
            d = self.robot.getDeviceByIndex(i)
            if d.getNodeType() == Node.POSITION_SENSOR:
                d.enable(self.ts)
                self.encoders.append(d)
            elif d.getNodeType() == Node.ROTATIONAL_MOTOR:
                d.setPosition(float("inf"))
                d.setVelocity(0.0)
                self.motors.append(d)

        left = [e for e in self.encoders if "left" in e.getName().lower()]
        right = [e for e in self.encoders if "right" in e.getName().lower()]
        assert left and right, "no left/right encoders found"
        self.odom = Odometry(self.base, left, right, self.WHEEL_RADIUS)

    def _enable(self, name, period):
        d = self.robot.getDevice(name)
        d.enable(period)
        return d

    # --- simulation ---
    def step(self):
        if self.robot.step(self.ts) == -1:
            return False
        self.step_count += 1
        roll, pitch, yaw = self.imu.getRollPitchYaw()
        self.odom.update(roll, pitch, yaw)
        if self.gps0 is None:
            self.gps0 = tuple(self.gps.getValues()[:2])
        return True

    def drive(self, left, right):
        """Wheel speeds in rad/s."""
        for m in self.motors:
            m.setVelocity(left if "left" in m.getName().lower() else right)

    # --- sensor readers ---
    def encoder_angles(self):
        return np.array([e.getValue() for e in self.encoders], dtype=np.float32)

    def imu_vector(self):
        return np.array(
            list(self.imu.getRollPitchYaw())
            + list(self.accel.getValues())
            + list(self.gyro.getValues()),
            dtype=np.float32,
        )

    def lidar_ranges(self):
        return np.array(self.lidar.getRangeImage(), dtype=np.float32).reshape(
            self.lidar.getNumberOfLayers(), self.lidar.getHorizontalResolution()
        )

    def lidar_points_base(self):
        m = self.MOUNTS["lidar"]
        pts = []
        for p in self.lidar.getPointCloud():
            if math.isfinite(p.x) and math.isfinite(p.y) and math.isfinite(p.z):
                pts.append(self.base.sensor_to_base((p.x, p.y, p.z), m))
        return np.array(pts, dtype=np.float32).reshape(-1, 3)

    def camera_image(self):
        w, h = self.camera.getWidth(), self.camera.getHeight()
        img = np.frombuffer(self.camera.getImage(), dtype=np.uint8).reshape(h, w, 4)
        return img[:, :, [2, 1, 0]]  # BGRA -> RGB

    def ground_truth(self):
        gx, gy, _ = self.gps.getValues()
        return (gx - self.gps0[0], gy - self.gps0[1])

    def observation(self):
        """Dict of arrays, or None until the camera and lidar have data."""

        if self.step_count < 5:
            return None
        return {
            "camera": self.camera_image(),
            "lidar": self.lidar_ranges(),
            "imu": self.imu_vector(),
            "encoders": self.encoder_angles(),
            "odom": np.array(self.base.pose(), dtype=np.float32),
            "time": self.robot.getTime(),
        }

    def status(self):
        tx, ty = self.ground_truth()
        x, y, yaw = self.base.pose()
        err = math.hypot(x - tx, y - ty)
        return (
            f"odom=({x:.2f}, {y:.2f}, yaw {yaw:.2f}) "
            f"gps=({tx:.2f}, {ty:.2f}) err={err:.2f}"
        )
