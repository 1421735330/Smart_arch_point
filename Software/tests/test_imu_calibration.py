from __future__ import annotations

from datetime import datetime
import unittest

from imu_calibration import ImuCalibrator
from imu_protocol import ImuSample


def sample(
    timestamp: float,
    accel: tuple[float, float, float],
    gyro: tuple[float, float, float],
) -> ImuSample:
    return ImuSample(timestamp, datetime.now(), 25.0, accel, gyro, (0, 0, 0, 0, 0, 0, 0))


class ImuCalibratorTests(unittest.TestCase):
    def test_stationary_biases_are_estimated_and_applied(self) -> None:
        calibrator = ImuCalibrator(duration_s=1.0, minimum_samples=20)
        calibrator.start()
        result = None
        for index in range(51):
            result = calibrator.add_sample(
                sample(index * 0.02, (0.02, -0.01, 1.03), (1.0, -2.0, 0.5))
            )

        self.assertIsNotNone(result)
        assert result is not None
        self.assertTrue(result.success)
        corrected = calibrator.apply(sample(2.0, (0.02, -0.01, 1.03), (1.0, -2.0, 0.5)))
        for actual, expected in zip(corrected.accel_g, (0.0, 0.0, 1.0)):
            self.assertAlmostEqual(actual, expected)
        for value in corrected.gyro_dps:
            self.assertAlmostEqual(value, 0.0)

    def test_moving_board_is_rejected(self) -> None:
        calibrator = ImuCalibrator(duration_s=1.0, minimum_samples=20)
        calibrator.start()
        result = None
        for index in range(51):
            moving_ax = 0.1 if index % 2 else -0.1
            result = calibrator.add_sample(
                sample(index * 0.02, (moving_ax, 0.0, 1.0), (0.0, 0.0, 0.0))
            )

        self.assertIsNotNone(result)
        assert result is not None
        self.assertFalse(result.success)
        self.assertFalse(calibrator.is_calibrated)

    def test_wrong_orientation_is_rejected(self) -> None:
        calibrator = ImuCalibrator(duration_s=0.5, minimum_samples=5)
        calibrator.start()
        result = None
        for index in range(6):
            result = calibrator.add_sample(
                sample(index * 0.1, (1.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            )

        self.assertIsNotNone(result)
        assert result is not None
        self.assertFalse(result.success)
        self.assertIn("+Z", result.message)


if __name__ == "__main__":
    unittest.main()
