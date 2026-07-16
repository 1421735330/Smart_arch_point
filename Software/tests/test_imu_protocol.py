from __future__ import annotations

import unittest

from imu_protocol import FrameParser, encode_test_frame


class FrameParserTests(unittest.TestCase):
    VALUES = (100, 2048, -1024, 512, 164, -328, 0)

    def test_decodes_and_scales_complete_frame(self) -> None:
        parser = FrameParser()
        samples = parser.feed(encode_test_frame(self.VALUES))

        self.assertEqual(len(samples), 1)
        sample = samples[0]
        self.assertEqual(sample.raw, self.VALUES)
        self.assertAlmostEqual(sample.temperature_c, 25 + 100 / 132.48)
        self.assertEqual(sample.accel_g, (1.0, -0.5, 0.25))
        self.assertEqual(sample.gyro_dps, (10.0, -20.0, 0.0))
        self.assertEqual(parser.good_frames, 1)

    def test_reassembles_split_notifications(self) -> None:
        parser = FrameParser()
        frame = encode_test_frame(self.VALUES)

        self.assertEqual(parser.feed(frame[:1]), [])
        self.assertEqual(parser.feed(frame[1:9]), [])
        self.assertEqual(len(parser.feed(frame[9:])), 1)

    def test_parses_multiple_frames_in_one_notification(self) -> None:
        parser = FrameParser()
        frame = encode_test_frame(self.VALUES)

        samples = parser.feed(frame + frame)

        self.assertEqual(len(samples), 2)
        self.assertEqual(parser.good_frames, 2)

    def test_skips_noise_and_recovers_after_bad_checksum(self) -> None:
        parser = FrameParser()
        bad = bytearray(encode_test_frame(self.VALUES))
        bad[-1] ^= 0x01
        valid = encode_test_frame(self.VALUES)

        samples = parser.feed(b"noise" + bad + valid)

        self.assertEqual(len(samples), 1)
        self.assertEqual(samples[0].raw, self.VALUES)
        self.assertEqual(parser.checksum_errors, 1)
        self.assertGreaterEqual(parser.discarded_bytes, 6)

    def test_keeps_partial_header_after_noise(self) -> None:
        parser = FrameParser()
        frame = encode_test_frame(self.VALUES)

        self.assertEqual(parser.feed(b"abc\xaa"), [])
        samples = parser.feed(frame[1:])

        self.assertEqual(len(samples), 1)


if __name__ == "__main__":
    unittest.main()
