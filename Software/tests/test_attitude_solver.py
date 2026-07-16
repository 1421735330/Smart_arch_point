from __future__ import annotations

from datetime import datetime
import unittest

from attitude_solver import AttitudeEstimator
from imu_protocol import ImuSample


def make_sample(
    timestamp: float,
    accel: tuple[float, float, float],
    gyro: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> ImuSample:
    return ImuSample(timestamp, datetime.now(), 25.0, accel, gyro, (0, 0, 0, 0, 0, 0, 0))


class AttitudeEstimatorTests(unittest.TestCase):
    def test_level_stationary_board(self) -> None:
        state = AttitudeEstimator().update(make_sample(1.0, (0.0, 0.0, 1.0)))

        self.assertAlmostEqual(state.roll_deg, 0.0)
        self.assertAlmostEqual(state.pitch_deg, 0.0)
        self.assertEqual(state.motion, "静止")
        self.assertEqual(state.orientation, "板面朝上 (+Z)")

    def test_accelerometer_initializes_tilt(self) -> None:
        state = AttitudeEstimator().update(make_sample(1.0, (-1.0, 0.0, 0.0)))

        self.assertAlmostEqual(state.pitch_deg, 90.0)
        self.assertEqual(state.orientation, "-X 轴朝上")

    def test_gyroscope_integrates_relative_yaw(self) -> None:
        estimator = AttitudeEstimator()
        estimator.update(make_sample(1.0, (0.0, 0.0, 1.0)))
        state = estimator.update(make_sample(1.1, (0.0, 0.0, 1.0), (0.0, 0.0, 90.0)))

        self.assertAlmostEqual(state.yaw_deg, 9.0)

    def test_long_sample_gap_is_capped(self) -> None:
        estimator = AttitudeEstimator()
        estimator.update(make_sample(1.0, (0.0, 0.0, 1.0)))
        state = estimator.update(make_sample(5.0, (0.0, 0.0, 1.0), (0.0, 0.0, 100.0)))

        self.assertAlmostEqual(state.yaw_deg, 10.0)

    def test_fast_rotation_is_identified(self) -> None:
        state = AttitudeEstimator().update(
            make_sample(1.0, (0.0, 0.0, 1.0), (0.0, 130.0, 0.0))
        )

        self.assertEqual(state.motion, "快速转动")


if __name__ == "__main__":
    unittest.main()
