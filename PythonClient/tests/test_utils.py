"""Offline unit tests for the AirSim Python client helpers.

These tests do not need Unreal Engine, a running simulator or OpenCV: they only
exercise the pure NumPy utility functions in ``airsim.utils``. Run them with::

    cd PythonClient
    python -m unittest discover -s tests -v
"""

import math
import os
import struct
import tempfile
import unittest

import numpy as np

from airsim import utils


class StringArrayTests(unittest.TestCase):
    def test_uint8_array_from_bytes(self):
        array = utils.string_to_uint8_array(b'\x00\x7f\xff')
        self.assertEqual(array.dtype, np.uint8)
        self.assertEqual(array.tolist(), [0, 127, 255])
        # Callers historically received a writable array from np.fromstring.
        self.assertTrue(array.flags.writeable)

    def test_float_array_from_bytes(self):
        payload = struct.pack('<4f', 1.5, -2.25, 0.0, 42.0)
        array = utils.string_to_float_array(payload)
        self.assertEqual(array.dtype, np.float32)
        np.testing.assert_allclose(array, [1.5, -2.25, 0.0, 42.0], rtol=0, atol=1e-6)
        self.assertTrue(array.flags.writeable)

    def test_empty_payload(self):
        self.assertEqual(utils.string_to_uint8_array(b'').size, 0)
        self.assertEqual(utils.string_to_float_array(b'').size, 0)

    def test_list_to_2d_float_array(self):
        array = utils.list_to_2d_float_array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0], 3, 2)
        self.assertEqual(array.shape, (2, 3))
        self.assertEqual(array.dtype, np.float32)
        np.testing.assert_allclose(array[1], [4.0, 5.0, 6.0])

    def test_get_pfm_array_uses_response_shape(self):
        class Response:
            width = 2
            height = 2
            image_data_float = [0.5, 1.5, 2.5, 3.5]

        array = utils.get_pfm_array(Response())
        self.assertEqual(array.shape, (2, 2))
        np.testing.assert_allclose(array, [[0.5, 1.5], [2.5, 3.5]])


class QuaternionTests(unittest.TestCase):
    def test_round_trip_keeps_angles(self):
        for pitch, roll, yaw in (
            (0.0, 0.0, 0.0),
            (0.1, -0.2, 0.3),
            (math.radians(20.0), math.radians(-15.0), math.radians(170.0)),
        ):
            quaternion = utils.to_quaternion(pitch, roll, yaw)
            norm = math.sqrt(
                quaternion.w_val ** 2
                + quaternion.x_val ** 2
                + quaternion.y_val ** 2
                + quaternion.z_val ** 2
            )
            self.assertAlmostEqual(norm, 1.0, places=9)
            out_pitch, out_roll, out_yaw = utils.to_eularian_angles(quaternion)
            self.assertAlmostEqual(out_pitch, pitch, places=6)
            self.assertAlmostEqual(out_roll, roll, places=6)
            self.assertAlmostEqual(math.cos(out_yaw), math.cos(yaw), places=6)
            self.assertAlmostEqual(math.sin(out_yaw), math.sin(yaw), places=6)


class PfmTests(unittest.TestCase):
    def test_grayscale_round_trip(self):
        image = np.arange(6, dtype=np.float32).reshape(2, 3) / 4.0
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, 'gray.pfm')
            utils.write_pfm(path, image)
            loaded, scale = utils.read_pfm(path)
        self.assertEqual(loaded.shape, image.shape)
        np.testing.assert_allclose(loaded, image, rtol=0, atol=1e-6)
        self.assertAlmostEqual(scale, 1.0, places=6)

    def test_color_round_trip(self):
        image = np.arange(2 * 2 * 3, dtype=np.float32).reshape(2, 2, 3) / 8.0
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, 'color.pfm')
            utils.write_pfm(path, image, scale=2.0)
            loaded, scale = utils.read_pfm(path)
        self.assertEqual(loaded.shape, image.shape)
        np.testing.assert_allclose(loaded, image, rtol=0, atol=1e-6)
        self.assertAlmostEqual(scale, 2.0, places=6)

    def test_write_pfm_rejects_bad_dtype(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(Exception):
                utils.write_pfm(os.path.join(tmp, 'bad.pfm'), np.zeros((2, 2), np.float64))

    def test_write_pfm_rejects_bad_shape(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(Exception):
                utils.write_pfm(os.path.join(tmp, 'bad.pfm'), np.zeros((2, 2, 2), np.float32))


if __name__ == '__main__':
    unittest.main()
