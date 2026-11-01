"""Render a top-down simulated camera view using OpenCV."""

import math

import cv2
import numpy as np

from drone_gcs.model import DroneState


PIXELS_PER_UNIT = 35
GRID_SPACING = 2

# Fixed landmarks: (world_x, world_y, BGR color, label).
LANDMARKS = (
    (4.0, 3.0, (80, 200, 80), "A"),
    (-3.0, -4.0, (50, 165, 245), "B"),
    (5.0, -4.0, (220, 180, 45), "C"),
)


def render_frame(
    state: DroneState,
    width: int = 640,
    height: int = 480,
) -> np.ndarray:
    """Return a BGR image of the world relative to the drone."""

    if (
        isinstance(width, bool)
        or not isinstance(width, int)
        or width <= 0
        or isinstance(height, bool)
        or not isinstance(height, int)
        or height <= 0
    ):
        raise ValueError("Image dimensions must be positive integers")

    if not (math.isfinite(state.x) and math.isfinite(state.y)):
        raise ValueError("Drone position must be finite")

    # Create a dark-green background.
    frame = np.full(
        (height, width, 3),
        (22, 32, 26),
        dtype=np.uint8,
    )

    center_x = width // 2
    center_y = height // 2

    # Draw a grid that moves relative to the drone.
    grid_step = GRID_SPACING * PIXELS_PER_UNIT

    start_x = int(
        (center_x + state.y * PIXELS_PER_UNIT) % grid_step
    )
    start_y = int(
        (center_y + state.x * PIXELS_PER_UNIT) % grid_step
    )

    grid_color = (47, 65, 51)

    for pixel_x in range(start_x, width, grid_step):
        cv2.line(
            frame,
            (pixel_x, 0),
            (pixel_x, height - 1),
            grid_color,
            1,
        )

    for pixel_y in range(start_y, height, grid_step):
        cv2.line(
            frame,
            (0, pixel_y),
            (width - 1, pixel_y),
            grid_color,
            1,
        )

    # Draw fixed landmarks in world coordinates.
    for world_x, world_y, color, label in LANDMARKS:
        screen_x = int(
            round(center_x + (state.y - world_y) * PIXELS_PER_UNIT)
        )
        screen_y = int(
            round(center_y + (state.x - world_x) * PIXELS_PER_UNIT)
        )

        if 0 <= screen_x < width and 0 <= screen_y < height:
            cv2.circle(
                frame,
                (screen_x, screen_y),
                12,
                color,
                -1,
            )

            cv2.putText(
                frame,
                label,
                (screen_x - 6, screen_y + 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 0),
                1,
                cv2.LINE_AA,
            )

    # The drone stays at the center of the camera view.
    cv2.arrowedLine(
        frame,
        (center_x, center_y + 20),
        (center_x, center_y - 28),
        (255, 255, 255),
        2,
        tipLength=0.28,
    )

    return frame
