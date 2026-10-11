import math


class Odometry:
    MAX_STEP = 0.1  # m, largest plausible movement in one step
    YAW_RESIDUAL_LIMIT = 0.35  # rad/s, placeholder 3-sigma gate
    GPS_RESIDUAL_LIMIT = 3.0  # normalized innovation gate
    GPS_VARIANCE = 0.04  # m^2, simulated GPS measurement variance
    PITCH_SUSPECT_LIMIT = math.radians(20.0)
    STUCK_COMMAND_LIMIT = 0.05  # m/s
    STUCK_ACCEL_LIMIT = 0.15  # m/s^2 after gravity removal
    FAILURE_STEPS = 8
    RECOVERY_STEPS = 120

    def __init__(
        self,
        base_link,
        left_encoders,
        right_encoders,
        wheel_radius,
        wheel_track=0.4,
        dt=0.016,
    ):
        self.base = base_link
        self.left = left_encoders
        self.right = right_encoders
        self.r = wheel_radius
        self.track = wheel_track
        self.dt = dt
        self.prev_l = None
        self.prev_r = None
        self.distance = 0.0  # total path length, m
        self.skipped = 0  # steps rejected as bad data
        self.health = {
            "encoders": "HEALTHY",
            "gyro": "HEALTHY",
            "accelerometer": "HEALTHY",
            "gps": "HEALTHY",
        }
        self.mode = "NOMINAL"
        self.pose_covariance = [0.0, 0.0, 0.0]
        self.diagnostics = {
            "wheel_speed": 0.0,
            "wheel_yaw_rate": 0.0,
            "yaw_residual": 0.0,
            "pitch_suspect": False,
            "stuck_suspect": False,
            "gps_residual": 0.0,
            "gps_weight": 0.0,
        }
        self._flags = {
            "encoders": 0,
            "gyro": 0,
            "accelerometer": 0,
            "gps": 0,
        }
        self._agreements = {
            "encoders": 0,
            "gyro": 0,
            "accelerometer": 0,
            "gps": 0,
        }

    @staticmethod
    def _mean(encoders):
        return sum(e.getValue() for e in encoders) / len(encoders)

    def _update_health(self, name, flagged):
        if flagged:
            self._flags[name] += 1
            self._agreements[name] = 0
            if self._flags[name] >= self.FAILURE_STEPS:
                self.health[name] = "FAILED"
            elif self.health[name] != "FAILED":
                self.health[name] = "SUSPECT"
        else:
            self._flags[name] = 0
            self._agreements[name] += 1
            if self._agreements[name] >= self.RECOVERY_STEPS:
                self.health[name] = "HEALTHY"

    def _refresh_mode(self):
        failed_motion_sources = sum(
            self.health[name] == "FAILED"
            for name in ("encoders", "gyro", "gps")
        )
        suspect = any(value != "HEALTHY" for value in self.health.values())
        self.mode = (
            "STOPPED"
            if failed_motion_sources == 3
            else ("DEGRADED" if suspect else "NOMINAL")
        )

    def update(
        self,
        roll,
        pitch,
        yaw,
        gyro_z=None,
        acceleration=None,
        commanded_speed=0.0,
        gps_position=None,
    ):
        """Update odometry and diagnostics.

        The diagnostics are deliberately conservative: without lidar or camera
        motion estimates, a suspected slip increases uncertainty but cannot
        correct the encoder position.
        """
        if not all(math.isfinite(a) for a in (roll, pitch, yaw)):
            self.skipped += 1
            self._update_health("gyro", True)
            self._refresh_mode()
            return False
        l = self._mean(self.left)
        r = self._mean(self.right)
        if not (math.isfinite(l) and math.isfinite(r)):
            self.skipped += 1
            self._update_health("encoders", True)
            self._refresh_mode()
            return False
        self.base.roll, self.base.pitch, self.base.yaw = roll, pitch, yaw
        if self.prev_l is None:
            self.prev_l, self.prev_r = l, r
            self._refresh_mode()
            return True
        dist = self.r * ((l - self.prev_l) + (r - self.prev_r)) / 2.0
        wheel_yaw_rate = self.r * ((r - self.prev_r) - (l - self.prev_l))
        wheel_yaw_rate /= self.track * self.dt
        self.prev_l, self.prev_r = l, r
        if abs(dist) > self.MAX_STEP:
            self.skipped += 1
            self._update_health("encoders", True)
            self._refresh_mode()
            return False

        wheel_speed = dist / self.dt
        yaw_residual = 0.0 if gyro_z is None else wheel_yaw_rate - gyro_z
        yaw_flagged = gyro_z is not None and (
            not math.isfinite(gyro_z)
            or abs(yaw_residual) > self.YAW_RESIDUAL_LIMIT
        )
        pitch_suspect = (
            abs(pitch) > self.PITCH_SUSPECT_LIMIT
            and abs(wheel_speed) > self.STUCK_COMMAND_LIMIT
        )
        stuck_suspect = False
        if acceleration is not None and abs(commanded_speed) > self.STUCK_COMMAND_LIMIT:
            values = tuple(acceleration)
            if len(values) == 3 and all(math.isfinite(a) for a in values):
                magnitude = math.sqrt(sum(a * a for a in values))
                stuck_suspect = abs(magnitude - 9.81) < self.STUCK_ACCEL_LIMIT

        self._update_health("encoders", pitch_suspect or yaw_flagged)
        self._update_health("gyro", yaw_flagged)
        self._update_health("accelerometer", stuck_suspect)
        self.diagnostics.update(
            wheel_speed=wheel_speed,
            wheel_yaw_rate=wheel_yaw_rate,
            yaw_residual=yaw_residual,
            pitch_suspect=pitch_suspect,
            stuck_suspect=stuck_suspect,
        )

        horiz = dist * math.cos(pitch)
        predicted_x = self.base.x + horiz * math.cos(yaw)
        predicted_y = self.base.y + horiz * math.sin(yaw)
        self.distance += abs(dist)
        uncertainty = 1.0 + (3.0 if pitch_suspect else 0.0)
        uncertainty += 5.0 if self.health["encoders"] != "HEALTHY" else 0.0
        if stuck_suspect:
            uncertainty += 5.0
        self.pose_covariance[0] += uncertainty * self.dt
        self.pose_covariance[1] += uncertainty * self.dt
        self.pose_covariance[2] += (0.1 + abs(yaw_residual)) * self.dt

        gps_residual = 0.0
        gps_weight = 0.0
        gps_flagged = False
        if gps_position is not None:
            gps_values = tuple(gps_position)
            if len(gps_values) == 2 and all(math.isfinite(value) for value in gps_values):
                gx, gy = gps_values
                dx, dy = gx - predicted_x, gy - predicted_y
                innovation_variance = (
                    max(self.pose_covariance[0], 1e-6) + self.GPS_VARIANCE
                )
                gps_residual = math.hypot(dx, dy) / math.sqrt(innovation_variance)
                gps_flagged = gps_residual > self.GPS_RESIDUAL_LIMIT
                self._update_health("gps", gps_flagged)
                if not gps_flagged and self.health["gps"] != "FAILED":
                    gps_weight = max(self.pose_covariance[0], 1e-6)
                    gps_weight /= innovation_variance
                    self.base.x = predicted_x + gps_weight * dx
                    self.base.y = predicted_y + gps_weight * dy
                    self.pose_covariance[0] *= 1.0 - gps_weight
                    self.pose_covariance[1] *= 1.0 - gps_weight
                else:
                    self.base.x, self.base.y = predicted_x, predicted_y
            else:
                self._update_health("gps", True)
                self.base.x, self.base.y = predicted_x, predicted_y
        else:
            self.base.x, self.base.y = predicted_x, predicted_y

        self.diagnostics.update(gps_residual=gps_residual, gps_weight=gps_weight)
        self._refresh_mode()
        return True
