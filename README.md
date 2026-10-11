# Rover Webots Project

This project contains a Webots simulation of a **Pioneer 3-AT rover** driving in a crater-like terrain. The controller is written in Python and provides a small rover API for reading sensors, estimating motion, and sending movement commands.

## Technologies used

- **Webots R2025a** for the robot and terrain simulation
- **Python** for the controller and navigation policy
- **Webots Python API** (`controller.Robot`, devices, motors, and sensors)
- **NumPy** for sensor data represented as arrays
- **Differential-drive odometry** based on the left and right wheel encoders

## Project structure

```text
.
├── crater_world.wbt                         # Webots world
├── policy.py                                # Navigation policy
└── controllers/
    └── my_controller/
        ├── my_controller.py                 # Main controller loop
        ├── rover.py                         # Rover hardware/sensor abstraction
        ├── odometry.py                      # Wheel-encoder odometry
        └── frames.py                        # Coordinate frames and sensor mounts
```

## Rover hardware

The simulated robot is a **Pioneer3at**. Its controller has access to:

| Device | Configuration | Purpose |
|---|---|---|
| RGB camera | 640 × 480 pixels, 1.2 rad field of view | Visual perception |
| 3D lidar | 256 horizontal samples, 16 layers, 180° field of view | Obstacle and terrain perception |
| Inertial unit | Roll, pitch, and yaw | Orientation estimation |
| Accelerometer | Three-axis acceleration | Motion and vibration information |
| Gyroscope | Three-axis angular velocity | Rotation information |
| GPS | Three-dimensional position | Ground-truth position in simulation |
| Wheel encoders | Left and right position sensors | Wheel displacement and odometry |
| Wheel motors | Rotational motors | Differential-drive motion |

The Webots world uses a basic simulation time step of **16 ms**. The camera and lidar are updated every four simulation steps, approximately **15.6 Hz**. The IMU, accelerometer, gyroscope, GPS, and encoders are updated every simulation step.

## Controller inputs

The main controller obtains its input from `Rover.observation()`. After the first five simulation steps, it returns a dictionary containing:

| Key | Type / shape | Description |
|---|---|---|
| `camera` | `numpy.ndarray`, `(480, 640, 3)` | RGB camera image |
| `lidar` | `numpy.ndarray`, `(16, 256)` | Lidar range image in meters |
| `imu` | `numpy.ndarray`, `(9,)` | Roll, pitch, yaw, acceleration `(x, y, z)`, and angular velocity `(x, y, z)` |
| `encoders` | `numpy.ndarray` | Wheel encoder angles in radians |
| `odom` | `numpy.ndarray`, `(3,)` | Estimated `(x, y, yaw)` pose |
| `time` | `float` | Current Webots simulation time in seconds |

`Rover.observation()` returns `None` during initialization until sensor data is available.

The rover also exposes:

- `lidar_points_base()`: finite lidar point-cloud coordinates transformed into the rover base frame
- `ground_truth()`: GPS displacement from the initial position, used for evaluation
- `status()`: estimated odometry, GPS displacement, and odometry error

## Coordinate frames

The rover uses a `base_link` frame:

- `x`: forward
- `y`: left
- `z`: up

The lidar is mounted at `(0.0, 0.0, 0.45)` meters relative to `base_link`.

The camera is mounted at `(0.25, 0.0, 0.30)` meters relative to `base_link`.

The current frame conversion assumes the sensors have no rotational offset relative to `base_link`. Odometry tracks planar position and yaw, while roll and pitch are read from the inertial unit.

## Current capabilities

The rover can currently:

1. Read synchronized visual, lidar, inertial, GPS, and wheel-encoder data.
2. Convert sensor measurements into NumPy arrays suitable for a policy or machine-learning model.
3. Build a 2D odometry estimate from the left and right wheel encoders.
4. Compensate forward odometry for the rover's pitch.
5. Transform lidar point-cloud data into the rover base frame.
6. Compare odometry against simulated GPS ground truth.
7. Generate random forward and angular commands through `RandomWalkPolicy`.

The current policy is a **data-collection placeholder**. It holds a randomly generated linear velocity between `0.1` and `0.4 m/s` and an angular velocity between `-0.4` and `0.4 rad/s` for a short random duration. It does not yet perform obstacle avoidance, mapping, localization, or goal-directed navigation.

## Main control loop

The controller follows this general sequence:

```text
Start Webots
    ↓
Read sensors
    ↓
Update IMU and wheel odometry
    ↓
Create observation
    ↓
Run navigation policy
    ↓
Send linear and angular velocity command
```

The policy is evaluated every four controller steps so that policy updates match the camera and lidar update rate.

## Running the simulation

1. Install Webots R2025a.
2. Open [`crater_world.wbt`](./crater_world.wbt) in Webots.
3. Start the simulation.
4. Webots loads the controller named `my_controller`.

The Python environment must provide:

```text
numpy
controller
```

The `controller` module is provided by Webots. NumPy is used by `rover.py`.

## Known implementation notes

- `my_controller.py` calls `rover.set_velocity(v, w)`, but `Rover` currently does not define that method. A velocity-command adapter still needs to be added before the main loop can drive the motors with `(v, w)` commands.
- `policy.py` is located at the repository root, while `my_controller.py` imports it as `from policy import RandomWalkPolicy`. The controller search path must include the repository root, or `policy.py` should be moved/copied into the controller directory.
- GPS is used as simulation ground truth for evaluation. It should not be treated as a substitute for a real rover's production localization system.
- The wheel radius is currently configured as `0.11 m` and should be verified against the simulated robot model.

## Future improvements

- Add a complete `set_velocity(linear, angular)` differential-drive conversion.
- Add obstacle avoidance using lidar ranges.
- Add camera-based terrain or obstacle classification.
- Add waypoint or goal-directed navigation.
- Add sensor fusion between odometry, IMU, and GPS.
- Record observations and actions for training a navigation policy.
- Add tests for odometry, frame transforms, and command conversion.
