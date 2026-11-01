"""Behavioral tests for the drone's deterministic movement model."""

import math
import unittest

from drone_gcs.model import (
    DroneState,
    VelocityCommand,
    advance_state,
    apply_command,
    current_speed,
)


class DroneModelTests(unittest.TestCase):
    def test_diagonal_motion_uses_both_axes(self):
        state = apply_command(DroneState(), VelocityCommand(vx=0.1, vy=1.0, seq=1))

        moved = advance_state(state, dt_s=1.0)

        self.assertAlmostEqual(moved.x, 0.1)  # forward
        self.assertAlmostEqual(moved.y, 1.0)  # left
        self.assertEqual(moved.last_seq, 1)

    def test_movement_scales_with_elapsed_time(self):
        initial = apply_command(DroneState(), VelocityCommand(vx=0.5, vy=-0.25, seq=1))

        one_step = advance_state(initial, dt_s=2.0)
        two_steps = advance_state(advance_state(initial, dt_s=1.0), dt_s=1.0)

        self.assertAlmostEqual(one_step.x, 1.0)
        self.assertAlmostEqual(one_step.y, -0.5)
        self.assertAlmostEqual(one_step.x, two_steps.x)
        self.assertAlmostEqual(one_step.y, two_steps.y)
        self.assertAlmostEqual(one_step.battery_pct, two_steps.battery_pct)

    def test_positive_vx_only_moves_forward(self):
        state = apply_command(DroneState(), VelocityCommand(vx=1.0, vy=0.0, seq=1))

        moved = advance_state(state, dt_s=2.0)

        self.assertEqual((moved.x, moved.y), (2.0, 0.0))

    def test_positive_vy_only_moves_left(self):
        state = apply_command(DroneState(), VelocityCommand(vx=0.0, vy=1.0, seq=1))

        moved = advance_state(state, dt_s=2.0)

        self.assertEqual((moved.x, moved.y), (0.0, 2.0))

    def test_clamps_finite_velocity_to_each_axis_limit(self):
        state = apply_command(DroneState(), VelocityCommand(vx=2.0, vy=-5.0, seq=1))

        self.assertEqual((state.vx, state.vy), (1.0, -1.0))

    def test_rejects_non_finite_velocity(self):
        for value in (math.nan, math.inf, -math.inf):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    apply_command(DroneState(), VelocityCommand(vx=value, vy=0.0, seq=1))

    def test_rejects_invalid_elapsed_time(self):
        for value in (-1.0, math.nan, math.inf):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    advance_state(DroneState(), dt_s=value)

    def test_battery_never_drops_below_zero(self):
        state = apply_command(DroneState(battery_pct=0.1), VelocityCommand(vx=1.0, vy=0.0, seq=1))

        drained = advance_state(state, dt_s=1000.0)

        self.assertEqual(drained.battery_pct, 0.0)

    def test_speed_is_magnitude_of_two_axes(self):
        state = apply_command(DroneState(), VelocityCommand(vx=1.0, vy=1.0, seq=1))

        self.assertAlmostEqual(current_speed(state), math.sqrt(2.0))

    def test_zero_command_stops_movement(self):
        state = apply_command(DroneState(), VelocityCommand(vx=1.0, vy=0.0, seq=1))
        moved = advance_state(state, dt_s=1.0)
        stopped = apply_command(moved, VelocityCommand(vx=0.0, vy=0.0, seq=2))

        after_stop = advance_state(stopped, dt_s=1.0)

        self.assertEqual((after_stop.x, after_stop.y), (moved.x, moved.y))
        self.assertEqual(current_speed(after_stop), 0.0)


if __name__ == "__main__":
    unittest.main()
