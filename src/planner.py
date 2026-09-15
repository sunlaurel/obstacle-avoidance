"""Tiny occupancy-grid A* and path-follow helpers.

CUSTOMIZATION POINT (global planner)
------------------------------------
Replace ``plan_path`` with Nav2, RRT, D* Lite, a learned planner, etc.
Inputs are start/goal XY plus the occupancy coming from ``obstacle_map``.
Output is a list of (x, y) waypoints the controller will track.

CUSTOMIZATION POINT (local controller)
--------------------------------------
Replace ``follow_path_cmd`` with DWA, pure-pursuit, a PID pose controller,
or whatever you want. It only needs to return (v, w) given pose + path.
"""

from __future__ import annotations

from collections import deque
import heapq
import math
from typing import Iterable, Sequence

from src.obstacle_map import ROBOT_RADIUS, is_free, line_of_sight


GridPoint = tuple[int, int]
PathXY = list[tuple[float, float]]


class OccupancyGrid:
    """Uniform Cartesian grid built from the hardcoded obstacle rectangles."""

    def __init__(
        self,
        xmin: float = -10.0,
        ymin: float = -10.0,
        xmax: float = 10.0,
        ymax: float = 10.0,
        resolution: float = 0.25,
        inflation: float = ROBOT_RADIUS,
    ) -> None:
        self.xmin = xmin
        self.ymin = ymin
        self.xmax = xmax
        self.ymax = ymax
        self.resolution = resolution
        self.inflation = inflation
        self.nx = int(round((xmax - xmin) / resolution))
        self.ny = int(round((ymax - ymin) / resolution))
        self.occ = [[False for _ in range(self.nx)] for _ in range(self.ny)]
        for iy in range(self.ny):
            for ix in range(self.nx):
                x, y = self.cell_center(ix, iy)
                self.occ[iy][ix] = not is_free(x, y, inflation)

    def in_bounds(self, ix: int, iy: int) -> bool:
        return 0 <= ix < self.nx and 0 <= iy < self.ny

    def is_free_cell(self, ix: int, iy: int) -> bool:
        return self.in_bounds(ix, iy) and not self.occ[iy][ix]

    def world_to_cell(self, x: float, y: float) -> GridPoint:
        ix = int((x - self.xmin) / self.resolution)
        iy = int((y - self.ymin) / self.resolution)
        return max(0, min(self.nx - 1, ix)), max(0, min(self.ny - 1, iy))

    def cell_center(self, ix: int, iy: int) -> tuple[float, float]:
        x = self.xmin + (ix + 0.5) * self.resolution
        y = self.ymin + (iy + 0.5) * self.resolution
        return x, y

    def nearest_free(self, x: float, y: float) -> GridPoint:
        start = self.world_to_cell(x, y)
        if self.is_free_cell(*start):
            return start
        q: deque[GridPoint] = deque([start])
        seen = {start}
        while q:
            ix, iy = q.popleft()
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                n = (ix + dx, iy + dy)
                if n in seen or not self.in_bounds(*n):
                    continue
                if self.is_free_cell(*n):
                    return n
                seen.add(n)
                q.append(n)
        return start


def _heuristic(a: GridPoint, b: GridPoint) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def plan_path(
    start: tuple[float, float],
    goal: tuple[float, float],
    grid: OccupancyGrid | None = None,
) -> PathXY:
    """A* on an 8-connected occupancy grid, then line-of-sight shortcut.

    Swap this function out for your own global planner. Returning an empty
    list means "no path" and the controller will sit and spin.
    """
    if grid is None:
        grid = OccupancyGrid()
    start_cell = grid.nearest_free(*start)
    goal_cell = grid.nearest_free(*goal)
    if not grid.is_free_cell(*start_cell) or not grid.is_free_cell(*goal_cell):
        return []

    open_heap: list[tuple[float, GridPoint]] = []
    heapq.heappush(open_heap, (0.0, start_cell))
    came_from: dict[GridPoint, GridPoint] = {}
    g_score: dict[GridPoint, float] = {start_cell: 0.0}
    closed: set[GridPoint] = set()
    neighbors: list[tuple[int, int, float]] = [
        (1, 0, 1.0),
        (-1, 0, 1.0),
        (0, 1, 1.0),
        (0, -1, 1.0),
        (1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)),
        (-1, -1, math.sqrt(2.0)),
    ]

    found = False
    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal_cell:
            found = True
            break
        closed.add(current)
        cx, cy = current
        for dx, dy, cost in neighbors:
            nxt = (cx + dx, cy + dy)
            if not grid.is_free_cell(*nxt):
                continue
            # Block diagonal corner-cutting through occupied cells.
            if dx != 0 and dy != 0:
                if not grid.is_free_cell(cx + dx, cy) or not grid.is_free_cell(cx, cy + dy):
                    continue
            tentative = g_score[current] + cost
            if tentative + 1e-9 < g_score.get(nxt, float("inf")):
                came_from[nxt] = current
                g_score[nxt] = tentative
                f = tentative + _heuristic(nxt, goal_cell)
                heapq.heappush(open_heap, (f, nxt))

    if not found:
        return []

    cells: list[GridPoint] = [goal_cell]
    cur = goal_cell
    while cur != start_cell:
        cur = came_from[cur]
        cells.append(cur)
    cells.reverse()

    raw: PathXY = [grid.cell_center(ix, iy) for ix, iy in cells]
    raw[0] = start
    raw[-1] = goal
    return shortcut_path(raw, inflation=grid.inflation)


def shortcut_path(path: Sequence[tuple[float, float]], inflation: float = ROBOT_RADIUS) -> PathXY:
    if len(path) <= 2:
        return list(path)
    out: PathXY = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1:
            if line_of_sight(path[i][0], path[i][1], path[j][0], path[j][1], inflation=inflation):
                break
            j -= 1
        out.append(path[j])
        i = j
    return out


def wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_quat(z: float, w: float) -> float:
    """Yaw from a planar quaternion (x=y=0)."""
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def lookahead_point(x: float, y: float, path: Sequence[tuple[float, float]], lookahead: float) -> tuple[float, float]:
    if not path:
        return x, y
    # Closest segment, then walk ``lookahead`` meters forward along the path.
    best_i = 0
    best_d = float("inf")
    for i, (px, py) in enumerate(path):
        d = math.hypot(px - x, py - y)
        if d < best_d:
            best_d = d
            best_i = i
    remaining = lookahead
    i = best_i
    cx, cy = path[i]
    while i < len(path) - 1 and remaining > 0.0:
        nx, ny = path[i + 1]
        seg = math.hypot(nx - cx, ny - cy)
        if seg < 1e-6:
            i += 1
            cx, cy = nx, ny
            continue
        if remaining <= seg:
            t = remaining / seg
            return cx + t * (nx - cx), cy + t * (ny - cy)
        remaining -= seg
        i += 1
        cx, cy = nx, ny
    return path[-1]


def follow_path_cmd(
    x: float,
    y: float,
    yaw: float,
    path: Sequence[tuple[float, float]],
    *,
    lookahead: float = 0.8,
    v_max: float = 0.7,
    w_max: float = 1.2,
    heading_slowdown: float = 0.7,
) -> tuple[float, float]:
    """Barebones heading controller toward a lookahead point.

    Returns (linear.x, angular.z). This is intentionally simple so it is
    obvious where to drop in your own local planner.
    """
    if not path:
        return 0.0, 0.0
    lx, ly = lookahead_point(x, y, path, lookahead)
    heading = math.atan2(ly - y, lx - x)
    err = wrap_angle(heading - yaw)
    w = max(-w_max, min(w_max, 2.2 * err))
    # Stop driving forward until we are roughly pointed the right way.
    if abs(err) > heading_slowdown:
        v = 0.0
    else:
        v = v_max * (1.0 - abs(err) / math.pi)
    goal_dist = math.hypot(path[-1][0] - x, path[-1][1] - y)
    if goal_dist < 1.0:
        v *= max(0.25, goal_dist)
    return v, w