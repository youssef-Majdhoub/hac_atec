class GyroYaw:
    """Yaw from integrating the (noisy) gyro z axis."""

    def __init__(self, gyro, dt):
        self.gyro = gyro
        self.dt = dt
        self.yaw = 0.0

    def update(self):
        self.yaw += self.gyro.getValues()[2] * self.dt
        return self.yaw
