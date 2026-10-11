import math
import json
import numpy as np
from controller import Robot, Node
from frames import BaseLink, Mount
from odometry import Odometry
from yaw_source import GyroYaw
import sensor_models as sm


class Rover:
    WHEEL_RADIUS = 0.11  # m, verify with the GPS/odom ratio
    WHEEL_TRACK = 0.4  # m, effective track width, calibrate
    MAX_WHEEL_ACCEL = 4.0  # rad/s^2
    # Offsets from base_link. Must match the world file.
    MOUNTS = {
        "imu": Mount(0.0, 0.0, 0.0),
        "gps": Mount(0.0, 0.0, 0.0),
        "lidar": Mount(0.0, 0.0, 0.45),
        "camera": Mount(0.25, 0.0, 0.3),
    }

    def __init__(self):
        self.robot = Robot()
        self.telemetry = self.robot.getDevice("telemetry")
        self.ts = int(self.robot.getBasicTimeStep())
        self.dt = self.ts / 1000.0
        self.step_count = 0
        self.base = BaseLink()

        self.camera = self._enable("camera", self.ts * 4)
        self.lidar = self._enable("lidar", self.ts * 4)
        self.lidar.enablePointCloud()
        self.imu = self._enable("imu", self.ts)  # tilt and simulation truth
        self.gps = self._enable("gps", self.ts)  # fused position and evaluation
        self.gps0 = None

        c = sm.CFG
        self.gyro = sm.NoisyVector3(
            self._enable("gyro", self.ts),
            self.dt,
            c.gyro_bias_init,
            c.gyro_bias_walk,
            c.gyro_noise,
        )
        self.accel = sm.NoisyVector3(
            self._enable("accel", self.ts),
            self.dt,
            c.accel_bias_init,
            0.0,
            c.accel_noise,
        )

        raw_encoders, self.motors = [], []
        for i in range(self.robot.getNumberOfDevices()):
            d = self.robot.getDeviceByIndex(i)
            if d.getNodeType() == Node.POSITION_SENSOR:
                d.enable(self.ts)
                raw_encoders.append(d)
            elif d.getNodeType() == Node.ROTATIONAL_MOTOR:
                d.setPosition(float("inf"))
                d.setVelocity(0.0)
                self.motors.append(d)
        self.encoders = [sm.NoisyEncoder(e) for e in raw_encoders]

        left = [e for e in self.encoders if "left" in e.getName().lower()]
        right = [e for e in self.encoders if "right" in e.getName().lower()]
        assert left and right, "no left/right encoders found"
        self.odom = Odometry(
            self.base,
            left,
            right,
            self.WHEEL_RADIUS,
            self.WHEEL_TRACK,
            self.dt,
        )

        self.gyro_yaw = GyroYaw(self.gyro, self.dt)
        self.yaw_true = 0.0
        self.target_left = self.target_right = 0.0
        self.cmd_left = self.cmd_right = 0.0

    def _enable(self, name, period):
        d = self.robot.getDevice(name)
        d.enable(period)
        return d

    # --- simulation ---
    def step(self):
        if self.robot.step(self.ts) == -1:
            return False
        self.step_count += 1

        dv = self.MAX_WHEEL_ACCEL * self.dt
        self.cmd_left += max(-dv, min(dv, self.target_left - self.cmd_left))
        self.cmd_right += max(-dv, min(dv, self.target_right - self.cmd_right))
        self.drive(self.cmd_left, self.cmd_right)

        self.gyro.update()
        self.accel.update()
        roll, pitch, self.yaw_true = self.imu.getRollPitchYaw()
        roll, pitch = sm.noisy_tilt(roll, pitch)
        gps_values = self.gps.getValues()
        if self.gps0 is None:
            self.gps0 = tuple(gps_values[:2])
        gps_position = (gps_values[0] - self.gps0[0], gps_values[1] - self.gps0[1])
        if self.step_count == 1:
            self.gyro_yaw.yaw = self.yaw_true  # known starting heading
        yaw = self.gyro_yaw.update()
        self.odom.update(
            roll,
            pitch,
            yaw,
            gyro_z=self.gyro.getValues()[2],
            acceleration=self.accel.getValues(),
            commanded_speed=(self.target_left + self.target_right)
            * self.WHEEL_RADIUS
            / 2.0,
            gps_position=gps_position,
        )
        self._publish_telemetry()
        return True

    def _publish_telemetry(self):
        payload = {
            "step": self.step_count,
            "time": self.robot.getTime(),
            "pose": self.base.pose(),
            "covariance": self.odom.pose_covariance,
            "health": self.odom.health,
            "mode": self.odom.mode,
            "diagnostics": self.odom.diagnostics,
        }
        self.telemetry.send(json.dumps(payload).encode("utf-8"))

    def drive(self, left, right):
        """Wheel speeds in rad/s, clamped to each motor's limit."""
        for m in self.motors:
            v = left if "left" in m.getName().lower() else right
            lim = m.getMaxVelocity()
            m.setVelocity(max(-lim, min(lim, v)))

    def set_velocity(self, v, w):
        """v: forward m/s, w: turn rate rad/s (positive = left)."""
        half = self.WHEEL_TRACK / 2.0
        self.target_left = (v - w * half) / self.WHEEL_RADIUS
        self.target_right = (v + w * half) / self.WHEEL_RADIUS

    # --- sensor readers (all noisy, what a real robot would see) ---
    def encoder_angles(self):
        return np.array([e.getValue() for e in self.encoders], dtype=np.float32)

    def imu_vector(self):
        """Estimated roll, pitch, yaw + noisy accel + noisy gyro."""
        b = self.base
        return np.array(
            [b.roll, b.pitch, b.yaw]
            + list(self.accel.getValues())
            + list(self.gyro.getValues()),
            dtype=np.float32,
        )

    def lidar_ranges(self):
        r = np.array(self.lidar.getRangeImage(), dtype=np.float32).reshape(
            self.lidar.getNumberOfLayers(), self.lidar.getHorizontalResolution()
        )
        return sm.noisy_ranges(r)

    def lidar_points_base(self):
        pts = np.array(
            [(p.x, p.y, p.z) for p in self.lidar.getPointCloud()], dtype=np.float32
        ).reshape(-1, 3)
        pts = pts[np.isfinite(pts).all(axis=1)]
        pts = sm.noisy_points(pts)
        m = self.MOUNTS["lidar"]
        return pts + np.array([m.x, m.y, m.z], dtype=np.float32)

    def camera_image(self):
        w, h = self.camera.getWidth(), self.camera.getHeight()
        img = np.frombuffer(self.camera.getImage(), dtype=np.uint8).reshape(h, w, 4)
        return sm.noisy_image(img[:, :, [2, 1, 0]])  # BGRA -> RGB

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
            "odom_covariance": np.array(self.odom.pose_covariance, dtype=np.float32),
            "odom_health": dict(self.odom.health),
            "odom_mode": self.odom.mode,
            "odom_diagnostics": dict(self.odom.diagnostics),
            "time": self.robot.getTime(),
        }

    def status(self):
        tx, ty = self.ground_truth()
        x, y, yaw = self.base.pose()
        err = math.hypot(x - tx, y - ty)
        d = getattr(self.odom, "distance", 0.0)
        drift = 100.0 * err / d if d > 0.5 else float("nan")
        dy = self.base.yaw - self.yaw_true
        yaw_err = math.degrees(math.atan2(math.sin(dy), math.cos(dy)))
        return (
            f"odom=({x:.2f}, {y:.2f}, yaw {yaw:.2f}) "
            f"gps=({tx:.2f}, {ty:.2f}) err={err:.2f} yaw_err={yaw_err:.1f}deg "
            f"dist={d:.1f} drift={drift:.1f}% "
            f"skipped={getattr(self.odom, 'skipped', 0)} "
            f"mode={self.odom.mode} health={self.odom.health}"
        )
