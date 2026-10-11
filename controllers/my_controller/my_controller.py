from controller import Robot, Node

robot = Robot()
ts = int(robot.getBasicTimeStep())

print("Devices on this robot:")
for i in range(robot.getNumberOfDevices()):
    print("  ", robot.getDeviceByIndex(i).getName())

camera = robot.getDevice("camera")
lidar = robot.getDevice("lidar")
imu = robot.getDevice("imu")
accel = robot.getDevice("accel")
gyro = robot.getDevice("gyro")
encoders = []
motors = []
for i in range(robot.getNumberOfDevices()):
    d = robot.getDeviceByIndex(i)
    if d.getNodeType() == Node.POSITION_SENSOR:
        d.enable(ts)
        encoders.append(d)
    elif d.getNodeType() == Node.ROTATIONAL_MOTOR:
        d.setPosition(float("inf"))
        d.setVelocity(2.0)
        motors.append(d)
print("encoders:", [e.getName() for e in encoders])
print("motors:", [m.getName() for m in motors])
camera.enable(ts * 4)
lidar.enable(ts * 4)
imu.enable(ts)
accel.enable(ts)
gyro.enable(ts)

step = 0
while robot.step(ts) != -1:
    if step % 60 == 0:
        ranges = [r for r in lidar.getRangeImage() if r != float("inf")]
        print("rpy:", imu.getRollPitchYaw())
        print("accel:", accel.getValues(), "gyro:", gyro.getValues())
        print(
            "lidar points:",
            len(ranges),
            "nearest:",
            round(min(ranges), 2) if ranges else None,
        )
    step += 1
