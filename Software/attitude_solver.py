"""Dependency-free 6-axis attitude and motion-state estimation."""

from __future__ import annotations

from dataclasses import dataclass
import math

from imu_protocol import ImuSample


@dataclass(frozen=True, slots=True)
class BoardState:
    received_at: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    accel_magnitude_g: float
    angular_rate_dps: float
    motion: str
    orientation: str


class AttitudeEstimator:
    """Fuse accelerometer tilt with integrated gyroscope rotation.

    Roll and pitch are stabilized by gravity through a complementary filter.
    Yaw is relative because a 6-axis IMU has no absolute heading reference.
    """

    def __init__(self, gyro_weight: float = 0.98) -> None:
        if not 0.0 <= gyro_weight <= 1.0:
            raise ValueError("gyro_weight must be between 0 and 1")
        self.gyro_weight = gyro_weight
        self.reset()

    def reset(self) -> None:
        self._last_time: float | None = None
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0

    def update(self, sample: ImuSample) -> BoardState:
        ax, ay, az = sample.accel_g
        gx, gy, gz = sample.gyro_dps
        accel_magnitude = math.sqrt(ax * ax + ay * ay + az * az)
        angular_rate = math.sqrt(gx * gx + gy * gy + gz * gz)

        accel_roll = math.degrees(math.atan2(ay, az))
        accel_pitch = math.degrees(math.atan2(-ax, math.hypot(ay, az)))

        if self._last_time is None:
            self._roll = accel_roll
            self._pitch = accel_pitch
        else:
            # Ignore duplicated timestamps and cap long gaps so reconnection does
            # not create an unrealistic attitude jump.
            dt = min(max(sample.received_at - self._last_time, 0.0), 0.1)
            gyro_roll = self._roll + gx * dt
            gyro_pitch = self._pitch + gy * dt
            self._yaw = self._wrap_angle(self._yaw + gz * dt)

            # Gravity is a trustworthy tilt reference only when the measured
            # acceleration magnitude is reasonably close to 1 g.
            if 0.80 <= accel_magnitude <= 1.20:
                alpha = self.gyro_weight
                self._roll = alpha * gyro_roll + (1.0 - alpha) * accel_roll
                self._pitch = alpha * gyro_pitch + (1.0 - alpha) * accel_pitch
            else:
                self._roll = gyro_roll
                self._pitch = gyro_pitch

        self._last_time = sample.received_at
        self._roll = self._wrap_angle(self._roll)
        self._pitch = self._wrap_angle(self._pitch)

        return BoardState(
            received_at=sample.received_at,
            roll_deg=self._roll,
            pitch_deg=self._pitch,
            yaw_deg=self._yaw,
            accel_magnitude_g=accel_magnitude,
            angular_rate_dps=angular_rate,
            motion=self._classify_motion(accel_magnitude, angular_rate),
            orientation=self._classify_orientation(ax, ay, az, accel_magnitude),
        )

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        return (angle + 180.0) % 360.0 - 180.0

    @staticmethod
    def _classify_motion(accel_magnitude: float, angular_rate: float) -> str:
        acceleration_error = abs(accel_magnitude - 1.0)
        if acceleration_error < 0.06 and angular_rate < 3.0:
            return "静止"
        if angular_rate >= 120.0:
            return "快速转动"
        if acceleration_error >= 0.50:
            return "剧烈运动"
        if acceleration_error < 0.20 and angular_rate < 25.0:
            return "轻微运动"
        return "运动中"

    @staticmethod
    def _classify_orientation(ax: float, ay: float, az: float, magnitude: float) -> str:
        if magnitude < 0.65 or magnitude > 1.35:
            return "动态中，朝向暂不可靠"

        components = ((abs(ax), ax, "X"), (abs(ay), ay, "Y"), (abs(az), az, "Z"))
        _absolute, signed, axis = max(components)
        sign = "+" if signed >= 0 else "-"
        if axis == "Z":
            surface = "板面朝上" if sign == "+" else "板面朝下"
            return f"{surface} ({sign}Z)"
        return f"{sign}{axis} 轴朝上"
