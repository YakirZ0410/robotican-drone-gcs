"""Renderer behavior tests will be added during Task 2."""

import unittest
import numpy as np

from drone_gcs.model import DroneState
from drone_gcs.render import render_frame


class TestRenderFrame(unittest.TestCase):

    def test_frame_shape_and_dtype(self):
        state = DroneState()

        frame = render_frame(state)

        self.assertEqual(frame.shape, (480, 640, 3))
        self.assertEqual(frame.dtype, np.uint8)

    def test_invalid_dimensions(self):
        state = DroneState()

        with self.assertRaises(ValueError):
            render_frame(state, width=0)

        with self.assertRaises(ValueError):
            render_frame(state, height=-1)

    def test_deterministic_rendering(self):
        state = DroneState(x=2.0, y=3.0)

        frame1 = render_frame(state)
        frame2 = render_frame(state)

        np.testing.assert_array_equal(frame1, frame2)

    def test_movement_changes_frame(self):
        initial_state = DroneState(x=0.0, y=0.0)
        forward_state = DroneState(x=1.0, y=0.0)
        left_state = DroneState(x=0.0, y=1.0)

        initial_frame = render_frame(initial_state)
        forward_frame = render_frame(forward_state)
        left_frame = render_frame(left_state)

        self.assertFalse(
            np.array_equal(initial_frame, forward_frame)
        )

        self.assertFalse(
            np.array_equal(initial_frame, left_frame)
        )


if __name__ == "__main__":
    unittest.main()