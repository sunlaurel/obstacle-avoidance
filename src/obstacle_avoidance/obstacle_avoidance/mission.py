"""ROS-free waypoint sampling.

CUSTOMIZATION POINT (waypoint generation)
-----------------------------------------
Replace ``sample_waypoints`` with your own sampler. The ROS node
``waypoint_generator.py`` only publishes whatever this function returns.
"""

from __future__ import annotations

import math
import random

from obstacle_avoidance.obstacle_map import ROBOT_RADIUS, START_XY, is_free
from obstacle_avoidance.planner import OccupancyGrid, plan_path


def sample_waypoints(
    count: int,
    rng: random.Random,
    *,
    start: tuple[float, float] = START_XY,
    xy_min: float = -8.5,
    xy_max: float = 8.5,
    min_spacing: float = 3.0,
    min_start_dist: float = 2.5,
    inflation: float = ROBOT_RADIUS,
    max_tries: int = 400,
) -> list[tuple[float, float]]:
    """Rejection-sample free XY points that are A*-reachable in sequence."""
    grid = OccupancyGrid(inflation=inflation)
    waypoints: list[tuple[float, float]] = []
    prev = start
    attempts = 0
    while len(waypoints) < count and attempts < max_tries:
        attempts += 1
        x = rng.uniform(xy_min, xy_max)
        y = rng.uniform(xy_min, xy_max)
        if not is_free(x, y, inflation):
            continue
        if math.hypot(x - start[0], y - start[1]) < min_start_dist:
            continue
        if any(math.hypot(x - px, y - py) < min_spacing for px, py in waypoints):
            continue
        if not plan_path(prev, (x, y), grid):
            continue
        waypoints.append((x, y))
        prev = (x, y)
    return waypoints
