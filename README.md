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
├── create_creator.py                        # Terrain/world generator
├── policy.py                                # Navigation policy
├── tests/
│   └── test_odometry.py                     # Odometry and fusion tests
└── controllers/
    ├── dashboard/
    │   └── dashboard.py                      # Live Webots telemetry HUD
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
| `odom_covariance` | `numpy.ndarray`, `(3,)` | Diagonal uncertainty estimate for `(x, y, yaw)` |
| `odom_health` | `dict` | `HEALTHY`, `SUSPECT`, or `FAILED` status for encoder, gyro, accelerometer, and GPS checks |
| `odom_mode` | `str` | `NOMINAL`, `DEGRADED`, or `STOPPED` |
| `odom_diagnostics` | `dict` | Wheel speed, wheel yaw rate, residuals, source weights, and slip/stuck flags |
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
2. Add configurable noise to sensor readings.
3. Inject test faults such as encoder over-reading, encoder freezing, camera blindness, lidar blindness, and sensor bias.
4. Integrate the noisy gyro z-axis to estimate yaw.
5. Build a 2D odometry estimate from the left and right wheel encoders.
6. Compensate forward odometry for the rover's pitch.
7. Transform lidar point-cloud data into the rover base frame.
8. Compare odometry against simulated GPS ground truth.
9. Generate random forward and angular commands through `RandomWalkPolicy`.

The current policy is a **data-collection placeholder**. It holds a randomly generated linear velocity between `0.1` and `0.4 m/s` and an angular velocity between `-0.4` and `0.4 rad/s` for a short random duration. It does not yet perform obstacle avoidance, mapping, localization, or goal-directed navigation.

### Sensor fusion: current implementation

The rover now uses a predict-correct fusion cycle in `Odometry.update()`. It is a
small 2D estimator rather than a full SLAM system:

1. **Prediction from encoders:** left and right encoder angle changes are converted
   to wheel travel. Their average gives forward travel, and their difference gives
   the wheel-implied yaw rate. Forward travel is projected horizontally using IMU
   pitch, then projected into world `x/y` using the gyro-integrated yaw.
2. **Heading from the gyro:** the z-axis gyro is integrated by `GyroYaw`. The
   encoder yaw rate is not blindly fused into the pose; it is compared against
   the gyro rate. A residual above `0.35 rad/s` flags both encoder and gyro checks
   as suspicious because the two sources disagree.
3. **GPS position correction:** the GPS position is converted to displacement from
   the initial GPS reading. The predicted position is compared with GPS. If the
   normalized innovation is within the `3.0` gate, GPS corrects the predicted
   position:

   ```text
   innovation = gps_position - predicted_position
   S = predicted_variance + gps_variance
   K = predicted_variance / S
   fused_position = predicted_position + K * innovation
   fused_variance = (1 - K) * predicted_variance
   ```

   The current simulated GPS variance is `0.04 m²`. Therefore, a source with
   higher predicted uncertainty receives a larger GPS correction. A GPS
   measurement outside the gate is rejected for that update and increases the
   GPS health counter.
4. **Uncertainty propagation:** every prediction increases the diagonal
   `(x, y, yaw)` variance. Pitch suspicion, encoder suspicion, and an apparent
   stuck condition increase that growth. Accepted GPS measurements reduce the
   x/y variance.
5. **Health state machine:** each source has consecutive fault and agreement
   counters. A source becomes `SUSPECT` on a fault, `FAILED` after 8 consecutive
   faults, and returns to `HEALTHY` after 120 consecutive agreements. This
   hysteresis prevents rapid trust switching.
6. **Operating mode:** `NOMINAL` means all checks are healthy. `DEGRADED` means
   at least one check is suspect or failed. `STOPPED` is reported when encoders,
   gyro, and GPS are all failed; the controller-level command stop still needs
   to be wired in.

The output contains the fused pose, a diagonal covariance estimate, health
states, the operating mode, and diagnostics including `gps_residual` and
`gps_weight`. This makes uncertainty and source quality available to the AI
policy instead of hiding them.

### Fusion limitations and next sources

GPS is currently the independent translation source. Lidar and camera data are
still returned to the policy, but they are **not yet motion estimates**:

- lidar does not yet perform scan-to-scan registration;
- the camera does not yet perform visual odometry or feature tracking;
- there is no inverse-variance vote between encoder, lidar, and camera motion;
- accelerometer data is currently a stuck/slip diagnostic, not integrated for
  position because integration would drift rapidly without bias estimation;
- GPS is a Webots measurement and is used as a simulated localization source,
  not as a production GPS model.

Adding lidar scan matching and visual odometry will provide independent
translation estimates that can replace or reduce reliance on GPS, especially
when GPS is unavailable.

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

The rover starts near the **south edge** of the crater at approximately
`(x=0, y=-45, z=3.03)`, facing north after a 180-degree heading change. This position is generated by
`create_creator.py` from the terrain height at the spawn point. Its initial
rotation changes yaw only; roll and pitch remain zero so the wheels face the
ground. The rover is placed above the terrain rather than inside it, and
slightly inside the `y=-50` boundary.

## Live Webots dashboard

The world includes a Supervisor-based dashboard. It uses a Webots `Emitter`
on the rover and a `Receiver` on the Supervisor:

```text
Rover controller
    └── sends JSON telemetry every simulation step
            ↓ channel 1
Supervisor dashboard
    └── draws labels over the Webots 3D view
```

The overlay displays:

- current operating mode: `NOMINAL`, `DEGRADED`, or `STOPPED`;
- simulation step and time;
- fused `(x, y, yaw)` pose;
- x/y/yaw covariance values;
- health for encoders, gyro, accelerometer, and GPS;
- yaw and GPS residuals;
- GPS correction weight;
- pitch-slip and stuck flags.

To view it, open the project folder in Webots, open `crater_world.wbt`, and
start the simulation. The console should show both `my_controller` and
`dashboard` starting successfully. If Webots reports that either controller
directory cannot be found, open the repository as a Webots project rather than
opening only the `.wbt` file.

The dashboard currently shows live telemetry. Automatic fault-scenario
execution and pass/fail counters are the next layer; the existing detailed
Python tests remain runnable independently with the command below.

The Python environment must provide:

```text
numpy
controller
```

The `controller` module is provided by Webots. NumPy is used by `rover.py`.

## Known implementation notes

- `set_velocity(v, w)` converts linear and angular velocity commands into left and right wheel speeds. The wheel track is currently an approximate calibration value of `0.4 m`.
- GPS is used as simulation ground truth for evaluation. It should not be treated as a substitute for a real rover's production localization system.
- The wheel radius is currently configured as `0.11 m` and should be verified against the simulated robot model.

## Future improvements

- Add lidar scan matching and camera motion estimation.
- Fuse encoder, lidar, and camera translation using uncertainty-based weights.
- Add a controller-level stop command when all usable motion sources are unavailable.
- Add obstacle avoidance using lidar ranges.
- Add camera-based terrain or obstacle classification.
- Add waypoint or goal-directed navigation.
- Record observations and actions for training a navigation policy.
- Add tests for odometry, frame transforms, and command conversion.

## Running the odometry tests

The focused tests are in [`tests/test_odometry.py`](./tests/test_odometry.py).
They use fake wheel encoders and a fake base frame, so they do not require
Webots. From the repository root, run the standard suite with:

```text
python -m unittest discover -s tests -p "test_*.py" -v
```

For the detailed per-test report, run:

```text
python tests/test_odometry.py
```

The suite covers normal forward and reverse motion, the first sample,
pitch compensation, exact and over-limit motion steps, non-finite tilt,
encoder and gyro values, malformed and outlier GPS measurements, GPS
covariance correction, yaw residual detection, zero and reverse command stuck
checks, health failure/recovery hysteresis, and `STOPPED` mode transitions.
Each test reports its full test ID and the runner prints a final count of
passes, failures, and errors.
