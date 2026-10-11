import random


class Policy:
    def act(self, obs):
        """obs: dict from Rover.observation(). Returns (v in m/s, w in rad/s)."""
        raise NotImplementedError


class RandomWalkPolicy(Policy):
    """Placeholder for data collection: holds a random command for a while."""

    def __init__(self, hold_steps=40):
        self.hold = hold_steps
        self.count = 0
        self.cmd = (0.0, 0.0)

    def act(self, obs):
        if self.count <= 0:
            self.cmd = (random.uniform(0.1, 0.4), random.uniform(-0.4, 0.4))
            self.count = random.randint(self.hold // 2, self.hold)
        self.count -= 1
        return self.cmd
