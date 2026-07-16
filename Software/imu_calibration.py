"""Stationary flat-position calibration for the six-axis IMU."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
import statistics

from imu_protocol import ImuSample


@dataclass(frozen=True, slots=True)
class CalibrationResult:
    success: bool
    message: str
    sample_count: int
    accel_bias_g: tuple[float, float, float] | None = None
    gyro_bias_dps: tuple[float, float, float] | None = None


class ImuCalibrator:
    """Estimate sensor offsets while the board lies still with +Z upward."""

    def __init__(self, duration_s: float = 2.0, minimum_samples: int = 20) -> None:
        self.duration_s = duration_s
        self.minimum_samples = minimum_samples
        self.accel_bias_g = (0.0, 0.0, 0.0)
        self.gyro_bias_dps = (0.0, 0.0, 0.0)
        self.is_calibrated = False
        self._samples: list[ImuSample] = []
        self._start_time: float | None = None
        self._elapsed = 0.0
        self._collecting = False

    @property
    def collecting(self) -> bool:
        return self._collecting

    @property
    def progress(self) -> float:
        if not self.collecting:
            return 0.0
        return min(self._elapsed / self.duration_s, 1.0)

    def reset(self) -> None:
        self.cancel()
        self.accel_bias_g = (0.0, 0.0, 0.0)
        self.gyro_bias_dps = (0.0, 0.0, 0.0)
        self.is_calibrated = False

    def start(self) -> None:
        self._samples = []
        self._start_time = None
        self._elapsed = 0.0
        self._collecting = True

    def cancel(self) -> None:
        self._samples = []
        self._start_time = None
        self._elapsed = 0.0
        self._collecting = False

    def add_sample(self, sample: ImuSample) -> CalibrationResult | None:
        if not self.collecting:
            return None
        # start() arms collection; the first sample supplies the monotonic epoch.
        if self._start_time is None:
            self._start_time = sample.received_at

        self._samples.append(sample)
        self._elapsed = max(sample.received_at - self._start_time, 0.0)
        if self._elapsed < self.duration_s:
            return None

        samples = self._samples
        self.cancel()
        return self._finish(samples)

    def apply(self, sample: ImuSample) -> ImuSample:
        accel = tuple(value - bias for value, bias in zip(sample.accel_g, self.accel_bias_g))
        gyro = tuple(value - bias for value, bias in zip(sample.gyro_dps, self.gyro_bias_dps))
        return replace(sample, accel_g=accel, gyro_dps=gyro)

    def _finish(self, samples: list[ImuSample]) -> CalibrationResult:
        count = len(samples)
        if count < self.minimum_samples:
            return CalibrationResult(False, f"样本不足（{count}/{self.minimum_samples}）", count)

        accel_axes = tuple([sample.accel_g[axis] for sample in samples] for axis in range(3))
        gyro_axes = tuple([sample.gyro_dps[axis] for sample in samples] for axis in range(3))
        accel_mean = tuple(statistics.fmean(axis) for axis in accel_axes)
        gyro_mean = tuple(statistics.fmean(axis) for axis in gyro_axes)
        accel_noise = max(statistics.pstdev(axis) for axis in accel_axes)
        gyro_noise = max(statistics.pstdev(axis) for axis in gyro_axes)
        gravity = math.sqrt(sum(value * value for value in accel_mean))
        mean_rotation = math.sqrt(sum(value * value for value in gyro_mean))

        if not 0.85 <= gravity <= 1.15:
            return CalibrationResult(False, f"重力幅值异常（{gravity:.3f} g），请将电路板静止平放", count)
        if accel_noise > 0.035 or gyro_noise > 1.5 or mean_rotation > 10.0:
            return CalibrationResult(False, "检测到移动或转动，请保持电路板完全静止后重试", count)
        if accel_mean[2] < 0.75:
            return CalibrationResult(False, "方向不正确，请将电路板 +Z 板面朝上平放", count)

        accel_bias = (accel_mean[0], accel_mean[1], accel_mean[2] - 1.0)
        gyro_bias = gyro_mean
        self.accel_bias_g = accel_bias
        self.gyro_bias_dps = gyro_bias
        self.is_calibrated = True
        return CalibrationResult(True, f"校准完成，共采集 {count} 个样本", count, accel_bias, gyro_bias)
