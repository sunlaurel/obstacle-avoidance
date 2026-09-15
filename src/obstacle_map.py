"""Known obstacle geometry for the Gazebo world.

The rectangles here MUST match ``worlds/obstacle_course.sdf`` (that file is
generated from this module). The navigator and fake lidar use this map as a
stand-in for a real costmap / sensor.

CUSTOMIZATION POINT (obstacle / world definition)
-------------------------------------------------
Edit ``OBSTACLES`` to change walls, hills, and ramps. Keep ``occupy=True`` for
anything the vehicle must not drive through. Set ``occupy=False`` for purely
visual props. After changing this list, regenerate the world SDF::

    python3 -c "from obstacle_avoidance.world_sdf import write_world; write_world()"
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence


@dataclass(frozen=True)
class BoxObstacle:
    """Axis-aligned box in the XY plane, optional pitch for ramps."""

    name: str
    cx: float
    cy: float
    cz: float
    sx: float
    sy: float
    sz: float
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    kind: str = "wall"
    occupy: bool = True
    color: tuple[float, float, float, float] = (0.45, 0.45, 0.5, 1.0)


# Arena is 20 m x 20 m. Vehicle starts at the origin (0, 0).
ARENA_MIN = -10.0
ARENA_MAX = 10.0
WALL_THICKNESS = 0.3
WALL_HEIGHT = 1.5

# RGB-ish materials used in the generated SDF.
WALL_COLOR = (0.42, 0.44, 0.50, 1.0)
HILL_COLOR = (0.50, 0.33, 0.18, 1.0)
RAMP_COLOR = (0.78, 0.48, 0.22, 1.0)

OBSTACLES: tuple[BoxObstacle, ...] = (
    # Perimeter — keep the vehicle inside the course.
    BoxObstacle("wall_north", 0.0, 10.15, WALL_HEIGHT / 2, 20.6, WALL_THICKNESS, WALL_HEIGHT, kind="wall", color=WALL_COLOR),
    BoxObstacle("wall_south", 0.0, -10.15, WALL_HEIGHT / 2, 20.6, WALL_THICKNESS, WALL_HEIGHT, kind="wall", color=WALL_COLOR),
    BoxObstacle("wall_east", 10.15, 0.0, WALL_HEIGHT / 2, WALL_THICKNESS, 20.6, WALL_HEIGHT, kind="wall", color=WALL_COLOR),
    BoxObstacle("wall_west", -10.15, 0.0, WALL_HEIGHT / 2, WALL_THICKNESS, 20.6, WALL_HEIGHT, kind="wall", color=WALL_COLOR),
    # Interior walls: an L-shape east of the origin and a north-south wall to the west.
    # Southern gap around wall_a is ~3 m so an inflated vehicle still fits.
    BoxObstacle("wall_a", 3.0, -2.75, WALL_HEIGHT / 2, 0.4, 8.5, WALL_HEIGHT, kind="wall", color=WALL_COLOR),
    BoxObstacle("wall_b", 5.5, 1.5, WALL_HEIGHT / 2, 5.4, 0.4, WALL_HEIGHT, kind="wall", color=WALL_COLOR),
    BoxObstacle("wall_c", -5.0, 3.25, WALL_HEIGHT / 2, 0.4, 9.5, WALL_HEIGHT, kind="wall", color=WALL_COLOR),
    # Hill / mound the vehicle must go around (not over).
    BoxObstacle("hill", 0.0, 7.5, 0.6, 3.0, 3.0, 1.2, kind="hill", color=HILL_COLOR),
)

OCCUPIED: tuple[BoxObstacle, ...] = tuple(o for o in OBSTACLES if o.occupy)

# Half-width of the vehicle plus a little padding. Used to inflate obstacles
# when checking goals and when building the A* grid.
ROBOT_RADIUS = 0.65

START_XY = (0.0, 0.0)


def occupied_boxes() -> Sequence[BoxObstacle]:
    return OCCUPIED


def point_hits_box(x: float, y: float, box: BoxObstacle, inflation: float = 0.0) -> bool:
    """Return True if (x, y) is inside the (inflated) axis-aligned footprint."""
    return abs(x - box.cx) <= box.sx / 2.0 + inflation and abs(y - box.cy) <= box.sy / 2.0 + inflation


def is_occupied(x: float, y: float, inflation: float = ROBOT_RADIUS) -> bool:
    """True if a disk of ``inflation`` radius around (x, y) intersects a wall."""
    if x < ARENA_MIN + inflation or x > ARENA_MAX - inflation:
        return True
    if y < ARENA_MIN + inflation or y > ARENA_MAX - inflation:
        return True
    return any(point_hits_box(x, y, box, inflation) for box in OCCUPIED)


def is_free(x: float, y: float, inflation: float = ROBOT_RADIUS) -> bool:
    return not is_occupied(x, y, inflation)


def line_of_sight(x0: float, y0: float, x1: float, y1: float, inflation: float = ROBOT_RADIUS, step: float = 0.15) -> bool:
    """Sample the segment and reject it if any sample is in collision."""
    dx = x1 - x0
    dy = y1 - y0
    dist = math.hypot(dx, dy)
    if dist < 1e-6:
        return is_free(x0, y0, inflation)
    n = max(1, int(dist / step))
    for i in range(n + 1):
        t = i / n
        if not is_free(x0 + t * dx, y0 + t * dy, inflation):
            return False
    return True


def raycast(x: float, y: float, yaw: float, range_max: float, inflation: float = 0.0, step: float = 0.05) -> float:
    """Return the first hit distance along a 2D ray, or ``range_max`` if clear.

    CUSTOMIZATION POINT (sensing)
    -----------------------------
    This is a geometric stand-in for a lidar / depth camera. Swap in a real
    Gazebo ``gpu_lidar`` plugin and drop ``fake_lidar.py`` if you want sensor
    noise and unknown obstacles.
    """
    ca = math.cos(yaw)
    sa = math.sin(yaw)
    t = step
    while t <= range_max:
        px = x + t * ca
        py = y + t * sa
        if is_occupied(px, py, inflation):
            return t
        t += step
    return range_max


def iter_occupied() -> Iterable[BoxObstacle]:
    return OCCUPIED
