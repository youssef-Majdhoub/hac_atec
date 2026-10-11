import math
import sys
import unittest
from pathlib import Path


CONTROLLER_DIR = Path(__file__).parents[1] / "controllers" / "my_controller"
sys.path.insert(0, str(CONTROLLER_DIR))

from odometry import Odometry  # noqa: E402


class FakeEncoder:
    def __init__(self, name):
        self.name = name
        self.value = 0.0

    def getValue(self):
        return self.value


class FakeBase:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0


def make_odometry():
    left = FakeEncoder("left")
    right = FakeEncoder("right")
    odometry = Odometry(FakeBase(), [left], [right], 0.11, 0.4, 0.016)
    return odometry, left, right


class OdometryTests(unittest.TestCase):
    def assert_health(self, odometry, expected, message=""):
        self.assertEqual(odometry.health, expected, message or odometry.diagnostics)

    def test_forward_prediction_uses_wheel_distance(self):
        odometry, left, right = make_odometry()
        self.assertTrue(
            odometry.update(0.0, 0.0, 0.0, gps_position=(0.0, 0.0)),
            odometry.diagnostics,
        )

        left.value = right.value = 0.1
        self.assertTrue(
            odometry.update(0.0, 0.0, 0.0, gyro_z=0.0, gps_position=None)
        )

        self.assertAlmostEqual(odometry.base.x, 0.011, places=6)
        self.assertAlmostEqual(odometry.base.y, 0.0, places=6)

    def test_gps_correction_reduces_position_error_and_variance(self):
        odometry, left, right = make_odometry()
        odometry.update(0.0, 0.0, 0.0, gps_position=(0.0, 0.0))
        left.value = right.value = 0.1
        odometry.update(0.0, 0.0, 0.0, gyro_z=0.0, gps_position=(0.0, 0.0))

        self.assertLess(odometry.base.x, 0.011)
        self.assertGreater(odometry.diagnostics["gps_weight"], 0.0)
        self.assertLess(odometry.pose_covariance[0], 0.016)

    def test_yaw_residual_marks_encoder_and_gyro_suspect(self):
        odometry, left, right = make_odometry()
        odometry.update(0.0, 0.0, 0.0)
        left.value = 0.0
        right.value = 0.1
        odometry.update(0.0, 0.0, 0.0, gyro_z=0.0)

        self.assertGreater(
            abs(odometry.diagnostics["yaw_residual"]),
            odometry.YAW_RESIDUAL_LIMIT,
        )
        self.assertEqual(odometry.health["encoders"], "SUSPECT")
        self.assertEqual(odometry.health["gyro"], "SUSPECT")

    def test_repeated_flags_fail_source_and_degrade_mode(self):
        odometry, left, right = make_odometry()
        odometry.update(0.0, 0.0, 0.0)
        for _ in range(odometry.FAILURE_STEPS):
            left.value += 0.0
            right.value += 0.1
            odometry.update(0.0, 0.0, 0.0, gyro_z=0.0)

        self.assertEqual(odometry.health["encoders"], "FAILED")
        self.assertEqual(odometry.health["gyro"], "FAILED")
        self.assertEqual(odometry.mode, "DEGRADED")

    def test_pitch_and_stuck_checks_increase_uncertainty(self):
        odometry, left, right = make_odometry()
        odometry.update(0.0, 0.0, 0.0)
        left.value = right.value = 0.05
        odometry.update(
            0.0,
            math.radians(25.0),
            0.0,
            gyro_z=0.0,
            acceleration=(0.0, 0.0, 9.81),
            commanded_speed=0.3,
        )

        self.assertTrue(odometry.diagnostics["pitch_suspect"])
        self.assertTrue(odometry.diagnostics["stuck_suspect"])
        self.assertGreater(odometry.pose_covariance[0], 0.016)

    def test_first_sample_initializes_without_motion(self):
        odometry, _, _ = make_odometry()

        self.assertTrue(odometry.update(0.0, 0.0, 0.0))
        self.assertEqual((odometry.base.x, odometry.base.y), (0.0, 0.0))
        self.assertEqual(odometry.distance, 0.0)
        self.assertEqual(odometry.pose_covariance, [0.0, 0.0, 0.0])

    def test_reverse_motion_has_negative_distance_but_positive_path_length(self):
        odometry, left, right = make_odometry()
        odometry.update(0.0, 0.0, 0.0)
        left.value = right.value = -0.1

        self.assertTrue(odometry.update(0.0, 0.0, 0.0, gyro_z=0.0))
        self.assertAlmostEqual(odometry.base.x, -0.011, places=6)
        self.assertAlmostEqual(odometry.distance, 0.011, places=6)

    def test_pitch_compensation_reduces_horizontal_distance(self):
        odometry, left, right = make_odometry()
        odometry.update(0.0, 0.0, 0.0)
        left.value = right.value = 0.1

        odometry.update(0.0, math.pi / 3.0, 0.0, gyro_z=0.0)

        self.assertAlmostEqual(odometry.base.x, 0.0055, places=6)
        self.assertAlmostEqual(odometry.distance, 0.011, places=6)

    def test_exact_maximum_step_is_accepted(self):
        odometry, left, right = make_odometry()
        odometry.update(0.0, 0.0, 0.0)
        wheel_angle = odometry.MAX_STEP / odometry.r
        left.value = right.value = wheel_angle

        self.assertTrue(odometry.update(0.0, 0.0, 0.0, gyro_z=0.0))
        self.assertEqual(odometry.skipped, 0)

    def test_step_over_maximum_is_rejected_without_pose_change(self):
        odometry, left, right = make_odometry()
        odometry.update(0.0, 0.0, 0.0)
        left.value = right.value = (odometry.MAX_STEP + 1e-6) / odometry.r

        self.assertFalse(odometry.update(0.0, 0.0, 0.0, gyro_z=0.0))
        self.assertEqual((odometry.base.x, odometry.base.y), (0.0, 0.0))
        self.assertEqual(odometry.skipped, 1)

    def test_nonfinite_tilt_is_rejected_and_marks_gyro(self):
        odometry, _, _ = make_odometry()

        self.assertFalse(odometry.update(0.0, math.nan, 0.0))
        self.assertEqual(odometry.skipped, 1)
        self.assertEqual(odometry.health["gyro"], "SUSPECT")

    def test_nonfinite_encoder_is_rejected_and_marks_encoder(self):
        odometry, left, _ = make_odometry()
        left.value = math.inf

        self.assertFalse(odometry.update(0.0, 0.0, 0.0))
        self.assertEqual(odometry.skipped, 1)
        self.assertEqual(odometry.health["encoders"], "SUSPECT")

    def test_nonfinite_gyro_is_rejected_as_a_gyro_fault(self):
        odometry, left, right = make_odometry()
        odometry.update(0.0, 0.0, 0.0)
        left.value = right.value = 0.1

        odometry.update(0.0, 0.0, 0.0, gyro_z=math.nan)

        self.assertEqual(odometry.health["gyro"], "SUSPECT")
        self.assertTrue(math.isnan(odometry.diagnostics["yaw_residual"]))

    def test_invalid_gps_shape_is_rejected_without_exception(self):
        odometry, left, right = make_odometry()
        odometry.update(0.0, 0.0, 0.0, gps_position=(0.0, 0.0))
        left.value = right.value = 0.1

        self.assertTrue(
            odometry.update(0.0, 0.0, 0.0, gyro_z=0.0, gps_position=(1.0,))
        )
        self.assertEqual(odometry.health["gps"], "SUSPECT")
        self.assertEqual(odometry.diagnostics["gps_weight"], 0.0)

    def test_gps_outlier_is_rejected_and_prediction_is_preserved(self):
        odometry, left, right = make_odometry()
        odometry.update(0.0, 0.0, 0.0, gps_position=(0.0, 0.0))
        left.value = right.value = 0.1

        odometry.update(0.0, 0.0, 0.0, gyro_z=0.0, gps_position=(100.0, 100.0))

        self.assertEqual(odometry.diagnostics["gps_weight"], 0.0)
        self.assertGreater(
            odometry.diagnostics["gps_residual"], odometry.GPS_RESIDUAL_LIMIT
        )
        self.assertAlmostEqual(odometry.base.x, 0.011, places=6)
        self.assertEqual(odometry.health["gps"], "SUSPECT")

    def test_stuck_check_ignores_zero_command_and_detects_reverse_command(self):
        odometry, left, right = make_odometry()
        odometry.update(0.0, 0.0, 0.0)
        left.value = right.value = 0.05

        odometry.update(
            0.0,
            0.0,
            0.0,
            gyro_z=0.0,
            acceleration=(0.0, 0.0, 9.81),
            commanded_speed=0.0,
        )
        self.assertFalse(odometry.diagnostics["stuck_suspect"])

        left.value = right.value = 0.1
        odometry.update(
            0.0,
            0.0,
            0.0,
            gyro_z=0.0,
            acceleration=(0.0, 0.0, 9.81),
            commanded_speed=-0.3,
        )
        self.assertTrue(odometry.diagnostics["stuck_suspect"])

    def test_health_recovery_requires_all_recovery_steps(self):
        odometry, left, right = make_odometry()
        odometry.update(0.0, 0.0, 0.0)
        for _ in range(odometry.FAILURE_STEPS):
            right.value += 0.1
            odometry.update(0.0, 0.0, 0.0, gyro_z=0.0)
        self.assertEqual(odometry.health["gyro"], "FAILED")

        for _ in range(odometry.RECOVERY_STEPS - 1):
            right.value += 0.0
            odometry.update(0.0, 0.0, 0.0, gyro_z=0.0)
        self.assertEqual(odometry.health["gyro"], "FAILED")

        right.value += 0.0
        odometry.update(0.0, 0.0, 0.0, gyro_z=0.0)
        self.assertEqual(odometry.health["gyro"], "HEALTHY")

    def test_stop_mode_requires_all_motion_sources_failed(self):
        odometry, left, right = make_odometry()
        odometry.update(0.0, 0.0, 0.0)
        for _ in range(odometry.FAILURE_STEPS):
            left.value = math.nan
            odometry.update(0.0, 0.0, 0.0)
        left.value = 0.0

        for _ in range(odometry.FAILURE_STEPS):
            odometry.update(math.nan, 0.0, 0.0)

        for _ in range(odometry.FAILURE_STEPS):
            odometry.update(0.0, 0.0, 0.0, gps_position=(math.nan, 0.0))

        self.assertEqual(odometry.health["encoders"], "FAILED")
        self.assertEqual(odometry.health["gyro"], "FAILED")
        self.assertEqual(odometry.health["gps"], "FAILED")
        self.assertEqual(odometry.mode, "STOPPED")


class DetailedTextTestResult(unittest.TextTestResult):
    def addSuccess(self, test):
        super().addSuccess(test)
        print(f"REPORT PASS: {test.id()}")

    def addFailure(self, test, err):
        print(f"REPORT FAIL: {test.id()}")
        super().addFailure(test, err)

    def addError(self, test, err):
        print(f"REPORT ERROR: {test.id()}")
        super().addError(test, err)


class DetailedTextTestRunner(unittest.TextTestRunner):
    resultclass = DetailedTextTestResult


if __name__ == "__main__":
    result = DetailedTextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(OdometryTests)
    )
    print(
        "REPORT SUMMARY: "
        f"ran={result.testsRun} failures={len(result.failures)} "
        f"errors={len(result.errors)}"
    )
    raise SystemExit(not result.wasSuccessful())
