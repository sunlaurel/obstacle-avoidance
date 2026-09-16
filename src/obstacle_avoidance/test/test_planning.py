"""Unit tests that do not need Gazebo or a ROS daemon."""

from __future__ import annotations

import math
import random

from obstacle_avoidance.obstacle_map import START_XY, is_free, is_occupied, raycast
from obstacle_avoidance.planner import OccupancyGrid, plan_path, should_skip_unreachable_waypoint
from obstacle_avoidance.mission import sample_waypoints
from obstacle_avoidance.world_sdf import current_waypoint_dot_sdf, generate_world_sdf, world_path


def test_origin_is_free() -> None:
    assert is_free(*START_XY)


def test_wall_a_is_occupied() -> None:
    assert is_occupied(3.0, -2.75, inflation=0.0)
    assert is_occupied(3.0, 0.0, inflation=0.0)


def test_hill_and_ramp_are_occupied() -> None:
    assert is_occupied(0.0, 7.5, inflation=0.0)
    assert is_occupied(7.0, -6.5, inflation=0.0)


def test_corridor_south_of_wall_a_is_free() -> None:
    # ~3 m gap between the south perimeter and wall_a.
    assert is_free(3.0, -8.5, inflation=0.65)


def test_eastward_ray_hits_wall_a() -> None:
    hit = raycast(0.0, 0.0, 0.0, range_max=8.0, inflation=0.0, step=0.05)
    assert 2.5 < hit < 3.2


def test_astar_goes_around_wall() -> None:
    grid = OccupancyGrid()
    path = plan_path((0.0, 0.0), (6.0, 0.0), grid)
    assert path, "expected a path around the L-shaped wall"
    assert math.hypot(path[-1][0] - 6.0, path[-1][1] - 0.0) < 0.3
    # Must not clip through wall_a at x=3, y=0.
    for x, y in path:
        assert is_free(x, y, inflation=0.4)
        if abs(x - 3.0) < 0.3:
            assert abs(y) > 1.2 or y < -6.5


def test_sample_waypoints_are_free_and_reachable() -> None:
    rng = random.Random(7)
    wps = sample_waypoints(4, rng)
    assert len(wps) == 4
    prev = START_XY
    grid = OccupancyGrid()
    for wp in wps:
        assert is_free(*wp)
        assert plan_path(prev, wp, grid)
        prev = wp


def test_skip_unreachable_after_lingering_outside_goal() -> None:
    assert not should_skip_unreachable_waypoint(7.9, 8.0, 1.2, 0.55)
    assert should_skip_unreachable_waypoint(8.0, 8.0, 1.2, 0.55)
    assert not should_skip_unreachable_waypoint(30.0, 8.0, 0.4, 0.55)


def test_waypoint_dot_sdf_is_visual_only() -> None:
    sdf = current_waypoint_dot_sdf("current_waypoint_dot")
    assert "<collision" not in sdf
    assert "<static>true</static>" in sdf
    assert "<sphere>" in sdf
    assert 'name="current_waypoint_dot"' in sdf


def test_committed_world_matches_generator() -> None:
    committed = world_path()
    assert committed.is_file(), "run write_world() to create worlds/obstacle_course.sdf"
    generated = generate_world_sdf()
    assert committed.read_text() == generated
