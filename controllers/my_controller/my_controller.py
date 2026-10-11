from rover import Rover
from policy import RandomWalkPolicy

rover = Rover()
policy = RandomWalkPolicy()
POLICY_EVERY = 4  # steps, matches the camera/lidar rate (about 15.6 Hz)

while rover.step():
    if rover.step_count % POLICY_EVERY == 0:
        obs = rover.observation()
        if obs is not None:
            v, w = policy.act(obs)
            rover.set_velocity(v, w)
    if rover.step_count % 60 == 0:
        print(rover.status())
