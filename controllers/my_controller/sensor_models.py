import math
from dataclasses import dataclass
import numpy as np


@dataclass
class NoiseConfig:
    enabled: bool = True
    seed: int = 1234
    # wheel encoders
    enc_counts_per_rev: int = 2048
    enc_noise_counts: float = 0.3
    # gyro (rad/s): initial bias spread, bias random walk per sqrt(s), white noise
    gyro_bias_init: float = 0.001
    gyro_bias_walk: float = 0.00002
    gyro_noise: float = 0.001
    # accelerometer (m/s^2)
    accel_bias_init: float = 0.02
    accel_noise: float = 0.05
    # roll/pitch measurement noise (rad)
    tilt_noise: float = 0.002
    # lidar
    lidar_range_noise_abs: float = 0.02  # m
    lidar_range_noise_rel: float = 0.01  # fraction of range
    lidar_dropout: float = 0.02  # probability a ray returns nothing
    # camera (gray levels, 0-255)
    cam_noise_std: float = 3.0


CFG = NoiseConfig()
RNG = np.random.default_rng(CFG.seed)

# Fault switches for testing, set from the controller at any time.
FAULTS = {"camera_blind": False, "lidar_blind": False}


class NoisyEncoder:
    """PositionSensor wrapper: quantization, noise, scale and freeze faults."""

    def __init__(self, sensor):
        self.sensor = sensor
        self.q = 2.0 * math.pi / CFG.enc_counts_per_rev
        self.noise = CFG.enc_noise_counts * self.q
        self.scale = 1.0  # fault hook: above 1 means it over-reads
        self.frozen = False  # fault hook: encoder stuck
        self._prev = None
        self._acc = 0.0

    def getName(self):
        return self.sensor.getName()

    def getValue(self):
        raw = self.sensor.getValue()
        if not CFG.enabled or not math.isfinite(raw):
            return raw
        if self._prev is None:
            self._prev, self._acc = raw, raw
        if not self.frozen:
            self._acc += (raw - self._prev) * self.scale
        self._prev = raw
        noise = float(RNG.normal(0.0, self.noise))
        return round((self._acc + noise) / self.q) * self.q


class NoisyVector3:
    """Gyro/accelerometer wrapper: bias, bias random walk and white noise.
    Call update() once per step; getValues() returns the cached sample."""

    def __init__(self, sensor, dt, bias_init, bias_walk, noise):
        self.sensor = sensor
        self.dt = dt
        self.walk = bias_walk
        self.noise = noise
        self.bias = RNG.normal(0.0, bias_init, 3) if CFG.enabled else np.zeros(3)
        self.extra_bias = np.zeros(3)  # fault hook: sudden bias jump
        self.value = [0.0, 0.0, 0.0]

    def update(self):
        raw = np.array(self.sensor.getValues(), dtype=float)
        if CFG.enabled and np.all(np.isfinite(raw)):
            self.bias += RNG.normal(0.0, self.walk * math.sqrt(self.dt), 3)
            raw = raw + self.bias + self.extra_bias + RNG.normal(0.0, self.noise, 3)
        self.value = raw.tolist()
        return self.value

    def getValues(self):
        return self.value


def noisy_tilt(roll, pitch):
    if not CFG.enabled:
        return roll, pitch
    return (
        roll + float(RNG.normal(0.0, CFG.tilt_noise)),
        pitch + float(RNG.normal(0.0, CFG.tilt_noise)),
    )


def noisy_ranges(ranges):
    """ranges: (layers, rays) float array in meters."""
    if FAULTS["lidar_blind"]:
        return np.full_like(ranges, np.inf)
    if not CFG.enabled:
        return ranges
    finite = np.isfinite(ranges)
    sigma = CFG.lidar_range_noise_abs + CFG.lidar_range_noise_rel * np.where(
        finite, ranges, 0.0
    )
    out = np.maximum(ranges + RNG.normal(0.0, 1.0, ranges.shape) * sigma, 0.0)
    out[RNG.random(ranges.shape) < CFG.lidar_dropout] = np.inf
    return out.astype(np.float32)


def noisy_points(pts):
    """pts: (N, 3) in the lidar frame. Noise is applied along each ray."""
    if FAULTS["lidar_blind"]:
        return np.zeros((0, 3), dtype=np.float32)
    if not CFG.enabled or len(pts) == 0:
        return pts
    r = np.linalg.norm(pts, axis=1)
    keep = (RNG.random(len(pts)) >= CFG.lidar_dropout) & (r > 1e-6)
    pts, r = pts[keep], r[keep]
    sigma = CFG.lidar_range_noise_abs + CFG.lidar_range_noise_rel * r
    scale = (r + RNG.normal(0.0, 1.0, len(r)) * sigma) / r
    return (pts * scale[:, None]).astype(np.float32)


def noisy_image(img):
    """img: (H, W, 3) uint8."""
    if FAULTS["camera_blind"]:
        return np.zeros_like(img)
    if not CFG.enabled:
        return img
    noise = RNG.normal(0.0, CFG.cam_noise_std, img.shape).astype(np.float32)
    return np.clip(img.astype(np.float32) + noise, 0, 255).astype(np.uint8)
