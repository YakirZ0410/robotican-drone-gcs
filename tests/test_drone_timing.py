"""Regressions for movement across command expiry and simulator stalls."""

from argparse import Namespace
import socket
import unittest
from unittest.mock import patch

from drone_gcs import drone
from drone_gcs.model import DroneState, VelocityCommand
from drone_gcs.protocol import encode_command


class TestMotionAcrossExpiry(unittest.TestCase):
    def test_motion_is_limited_to_remaining_command_validity(self):
        cases = (
            # last receipt, interval end, displacement, final velocity
            (1_000_000_000, 1_200_000_000, 0.2, 1.0),
            (1_000_000_000, 1_500_000_000, 0.5, 0.0),
            (1_000_000_000, 3_000_000_000, 0.5, 0.0),
            (800_000_000, 3_000_000_000, 0.3, 0.0),
            (0, 3_000_000_000, 0.0, 0.0),
            (None, 3_000_000_000, 0.0, 0.0),
        )
        for last_ns, now_ns, distance, final_vx in cases:
            with self.subTest(last_ns=last_ns, now_ns=now_ns):
                state = drone.advance_state_with_timeout(
                    DroneState(x=10.0, y=5.0, vx=1.0, vy=-0.5, last_seq=7),
                    last_update_ns=1_000_000_000, now_ns=now_ns,
                    last_command_ns=last_ns, timeout_ns=500_000_000,
                )
                self.assertAlmostEqual(state.x, 10.0 + distance)
                self.assertAlmostEqual(state.y, 5.0 - distance * 0.5)
                self.assertEqual((state.vx, state.vy), (final_vx, -0.5 * final_vx))
                self.assertEqual(state.last_seq, 7)

    def test_idle_drain_covers_full_gap_and_motion_drain_only_valid_time(self):
        state = drone.advance_state_with_timeout(
            DroneState(x=10.0, vx=1.0, last_seq=7),
            last_update_ns=1_000_000_000, now_ns=3_000_000_000,
            last_command_ns=1_000_000_000, timeout_ns=500_000_000,
        )
        self.assertAlmostEqual(state.x, 10.5)
        # Two seconds idle drain (0.004), half a unit motion drain (0.005).
        self.assertAlmostEqual(state.battery_pct, 99.991)

    def test_stationary_drone_still_drains_battery_for_entire_gap(self):
        state = drone.advance_state_with_timeout(
            DroneState(last_seq=7),
            last_update_ns=1_000_000_000, now_ns=3_000_000_000,
            last_command_ns=1_000_000_000, timeout_ns=500_000_000,
        )
        self.assertEqual((state.x, state.y), (0.0, 0.0))
        self.assertAlmostEqual(state.battery_pct, 99.996)


class TestSimulatorStall(unittest.TestCase):
    def test_two_second_stall_does_not_retroactively_extend_old_motion(self):
        # Exercise the actual simulator loop and UDP decoder. Only the clock
        # and rendering boundary are controlled to inject a repeatable stall.
        cases = (
            (None, 0.0, 0.5),
            (VelocityCommand(-1.0, 0.0, 2), -1.0, 0.4),
            (VelocityCommand(1.0, 0.0, 1), 1.0, 0.6),
        )
        for next_command, expected_vx, expected_later_x in cases:
            with self.subTest(next_command=next_command):
                with (
                    socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender,
                    socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as video,
                    socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as telemetry,
                ):
                    for receiver in (video, telemetry):
                        receiver.bind(("127.0.0.1", 0))
                    args = Namespace(
                        bind_host="127.0.0.1", gcs_host="127.0.0.1", command_port=0,
                        video_port=video.getsockname()[1],
                        telemetry_port=telemetry.getsockname()[1],
                        fps=10.0, width=160, height=120, jpeg_quality=65,
                        command_timeout_ms=500,
                    )
                    clock_ns = 1_000_000_000
                    snapshots = []
                    receive = drone.receive_commands
                    render = drone.render_frame
                    receive_count = 0

                    def inject_commands(command_socket, active, last_ns):
                        nonlocal receive_count
                        if receive_count == 0:
                            command = VelocityCommand(1.0, 0.0, 1)
                        elif receive_count == 1:
                            command = next_command
                        else:
                            command = None
                        receive_count += 1
                        if command is not None:
                            sender.sendto(encode_command(command), command_socket.getsockname())
                        return receive(command_socket, active, last_ns)

                    class EndSimulation(Exception):
                        pass

                    def render_with_stall(state, **kwargs):
                        nonlocal clock_ns
                        snapshots.append(state)
                        if len(snapshots) == 3:
                            raise EndSimulation
                        clock_ns += 2_000_000_000 if len(snapshots) == 1 else 100_000_000
                        return render(state, **kwargs)

                    with (
                        patch.object(drone.time, "monotonic_ns", side_effect=lambda: clock_ns),
                        patch.object(drone.time, "monotonic", side_effect=lambda: clock_ns / 1e9),
                        patch.object(drone.time, "sleep"),
                        patch.object(drone, "receive_commands", side_effect=inject_commands),
                        patch.object(drone, "render_frame", side_effect=render_with_stall),
                        self.assertRaises(EndSimulation),
                    ):
                        drone.run_simulator(args)

                resumed, later = snapshots[1:]
                self.assertAlmostEqual(resumed.x, 0.5)
                self.assertEqual(resumed.vx, expected_vx)
                self.assertAlmostEqual(resumed.battery_pct, 99.991)
                self.assertAlmostEqual(later.x, expected_later_x)
                self.assertEqual(resumed.last_seq, 2 if next_command and next_command.seq == 2 else 1)
