"""Smart Place Point IMU frame decoding.

The firmware sends a fixed 17-byte frame through the BLE notify characteristic:
AA 55 + seven signed big-endian int16 values + uint8 additive checksum.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import struct
import time


FRAME_HEADER = b"\xAA\x55"
FRAME_LENGTH = 17
ACCEL_SCALE = 2048.0
GYRO_SCALE = 16.4
TEMP_SCALE = 132.48
TEMP_OFFSET = 25.0


@dataclass(frozen=True, slots=True)
class ImuSample:
    received_at: float
    wall_time: datetime
    temperature_c: float
    accel_g: tuple[float, float, float]
    gyro_dps: tuple[float, float, float]
    raw: tuple[int, int, int, int, int, int, int]


class FrameParser:
    """Incrementally parse arbitrary BLE notification chunks."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.good_frames = 0
        self.checksum_errors = 0
        self.discarded_bytes = 0

    def reset(self, *, reset_statistics: bool = False) -> None:
        self._buffer.clear()
        if reset_statistics:
            self.good_frames = 0
            self.checksum_errors = 0
            self.discarded_bytes = 0

    def feed(self, data: bytes | bytearray | memoryview) -> list[ImuSample]:
        if data:
            self._buffer.extend(data)

        samples: list[ImuSample] = []
        while True:
            header_index = self._buffer.find(FRAME_HEADER)
            if header_index < 0:
                # Keep a trailing 0xAA because it may be the first header byte.
                keep = 1 if self._buffer.endswith(FRAME_HEADER[:1]) else 0
                discarded = len(self._buffer) - keep
                if discarded:
                    del self._buffer[:discarded]
                    self.discarded_bytes += discarded
                break

            if header_index:
                del self._buffer[:header_index]
                self.discarded_bytes += header_index

            if len(self._buffer) < FRAME_LENGTH:
                break

            candidate = self._buffer[:FRAME_LENGTH]
            expected_checksum = sum(candidate[2:16]) & 0xFF
            if candidate[16] != expected_checksum:
                # Drop one byte only, then search again. This recovers even if a
                # valid frame starts inside the rejected 17-byte window.
                del self._buffer[0]
                self.checksum_errors += 1
                self.discarded_bytes += 1
                continue

            raw = struct.unpack(">7h", candidate[2:16])
            now = time.perf_counter()
            samples.append(
                ImuSample(
                    received_at=now,
                    wall_time=datetime.now().astimezone(),
                    temperature_c=(raw[0] / TEMP_SCALE) + TEMP_OFFSET,
                    accel_g=(
                        raw[1] / ACCEL_SCALE,
                        raw[2] / ACCEL_SCALE,
                        raw[3] / ACCEL_SCALE,
                    ),
                    gyro_dps=(
                        raw[4] / GYRO_SCALE,
                        raw[5] / GYRO_SCALE,
                        raw[6] / GYRO_SCALE,
                    ),
                    raw=raw,
                )
            )
            self.good_frames += 1
            del self._buffer[:FRAME_LENGTH]

        return samples


def encode_test_frame(values: tuple[int, int, int, int, int, int, int]) -> bytes:
    """Encode a frame for tests and the built-in visual demo."""

    payload = struct.pack(">7h", *values)
    return FRAME_HEADER + payload + bytes((sum(payload) & 0xFF,))
