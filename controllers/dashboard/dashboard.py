import json
from controller import Supervisor


supervisor = Supervisor()
step = int(supervisor.getBasicTimeStep())
receiver = supervisor.getDevice("telemetry")
receiver.enable(step)


def color_for_mode(mode):
    return 0x33CC66 if mode == "NOMINAL" else (0xFFCC33 if mode == "DEGRADED" else 0xFF3333)


def draw_dashboard(data):
    pose = data.get("pose", [0.0, 0.0, 0.0])
    covariance = data.get("covariance", [0.0, 0.0, 0.0])
    diagnostics = data.get("diagnostics", {})
    health = data.get("health", {})
    mode = data.get("mode", "NO DATA")
    lines = [
        "ROVER ODOMETRY DASHBOARD",
        f"MODE: {mode}",
        f"STEP: {data.get('step', 0)}  TIME: {data.get('time', 0.0):.1f}s",
        f"POSE: x={pose[0]:.2f} y={pose[1]:.2f} yaw={pose[2]:.2f}",
        f"COV:  x={covariance[0]:.3f} y={covariance[1]:.3f} yaw={covariance[2]:.3f}",
        "HEALTH: " + "  ".join(f"{name}={value}" for name, value in health.items()),
        f"YAW RESIDUAL: {diagnostics.get('yaw_residual', 0.0):.3f}",
        f"GPS RESIDUAL: {diagnostics.get('gps_residual', 0.0):.3f}",
        f"GPS WEIGHT: {diagnostics.get('gps_weight', 0.0):.3f}",
        f"SLIP: {diagnostics.get('pitch_suspect', False)}  "
        f"STUCK: {diagnostics.get('stuck_suspect', False)}",
        "TESTS: live telemetry | fault scenarios: pending",
    ]
    for index, line in enumerate(lines):
        supervisor.setLabel(
            index,
            line,
            0.02,
            0.03 + index * 0.035,
            0.9,
            color_for_mode(mode) if index == 1 else 0xFFFFFF,
            0.0,
            "Arial",
        )


while supervisor.step(step) != -1:
    while receiver.getQueueLength():
        try:
            draw_dashboard(json.loads(receiver.getData().decode("utf-8")))
        finally:
            receiver.nextPacket()
